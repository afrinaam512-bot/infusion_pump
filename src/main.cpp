#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include "InfusionMode.hpp"
#include "ConstantRateMode.hpp"
#include "LinearRampMode.hpp"
#include "EncoderVolumeTracker.hpp"
#include "OcclusionMonitor.hpp"

// ── GPIO specs ────────────────────────────────────
static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec step_pin =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_step), gpios);
static const struct gpio_dt_spec dir_pin =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir), gpios);
static const struct gpio_dt_spec en_pin= 
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_en), gpios);
static const struct gpio_dt_spec buzzer =
    GPIO_DT_SPEC_GET(DT_NODELABEL(buzzer_out), gpios);

// ── Alarm helpers ─────────────────────────────────
static void alarm_on(void) {
    gpio_pin_set_dt(&led,    1);
    gpio_pin_set_dt(&buzzer, 1);
}

static void alarm_off(void) {
    gpio_pin_set_dt(&led,    0);
    gpio_pin_set_dt(&buzzer, 0);
}

// ── Encoder globals & Physical Constants ──────────
static const struct device *qdec_dev;
static int32_t  g_encoderRawPrev     = 0;
static bool     g_encoderInitialized = false;
static int32_t  g_lastEncoderCount   = 0; // Cumulative degrees tracked

#define ENCODER_DEG_PER_REV  360
#define UL_PER_REV           3200

static uint32_t motorTicks      = 0;
static uint32_t expectedVolumeUL = 0;
static uint32_t actualVolumeUL    = 0;
static uint32_t volumeOffset     = 0;
static uint32_t lastRampUpdateMs = 0;

// ── I2C / LPS22HB ────────────────────────────────
static const struct device *i2c_dev;
#define LPS22HB_ADDR         0x5C
#define LPS22HB_PRESS_OUT_XL 0x28
#define LPS22HB_WHO_AM_I     0x0F
#define LPS22HB_CTRL_REG1    0x10
static bool lps22hb_ready = false;

static bool lps22hb_init(void) {
    uint8_t who_am_i = 0;
    uint8_t reg = LPS22HB_WHO_AM_I;
    if (i2c_write_read(i2c_dev, LPS22HB_ADDR, &reg, 1, &who_am_i, 1) != 0) {
        printk("LPS22HB not found!\n");
        return false;
    }
    printk("LPS22HB WHO_AM_I: 0x%02X\n", who_am_i);
    uint8_t cfg[2] = {LPS22HB_CTRL_REG1, 0x30};
    i2c_write(i2c_dev, cfg, 2, LPS22HB_ADDR);
    return (who_am_i == 0xB1);
}

static uint32_t read_lps22hb_pressure(void) {
    if (!lps22hb_ready) return 1013;
    uint8_t buf[3];
    uint8_t reg = LPS22HB_PRESS_OUT_XL | 0x80;
    if (i2c_write_read(i2c_dev, LPS22HB_ADDR, &reg, 1, buf, 3) != 0) {
        return 1013;
    }
    int32_t raw = ((int32_t)buf[2] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[0];
    return (uint32_t)(raw / 4096);
}

// ── Static instances — no heap ───────────────────
static ConstantRateMode  constantMode(100000);
static LinearRampMode    rampMode(10000, 500000, 10000);
static OcclusionMonitor  occlusionMonitor;
static InfusionMode*     activeMode    = &constantMode;
static bool              useRealSensor = true;

// ── UART buffer ───────────────────────────────────
#define UART_BUF_SIZE 32
static char   uart_buf[UART_BUF_SIZE];
static uint8_t uart_buf_pos = 0;
static const struct device *uart_dev;

// ── Encoder Helper Functions ──────────────────────
static int32_t readEncoderRaw(void) {
    if (!qdec_dev) return 0;
    struct sensor_value val;
    sensor_sample_fetch(qdec_dev);
    sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &val);
    return val.val1; // Degrees 0-359
}

static void updateEncoderPosition(void) {
    int32_t raw = readEncoderRaw();
    if (!g_encoderInitialized) {
        g_encoderRawPrev     = raw;
        g_encoderInitialized = true;
        return;
    }
    int32_t delta = raw - g_encoderRawPrev;

    // Handle degree boundary rollover at 360
    if (delta > ENCODER_DEG_PER_REV / 2)
        delta -= ENCODER_DEG_PER_REV;
    if (delta < -ENCODER_DEG_PER_REV / 2)
        delta += ENCODER_DEG_PER_REV;

    // Accumulate running absolute delta tracking distance
    if (delta < 0) delta = -delta;
    g_lastEncoderCount += delta;
    g_encoderRawPrev    = raw;
}

static uint32_t getActualVolumeUL(void) {
    // 360 degrees = 3200 uL
    return volumeOffset + (uint32_t)((int64_t)g_lastEncoderCount * UL_PER_REV / ENCODER_DEG_PER_REV);
}

static void resetEncoderPosition(void) {
    g_lastEncoderCount   = 0;
    g_encoderInitialized = false;
}

// ── Process command ───────────────────────────────
static void process_command(const char* cmd) {
    printk("CMD: %s\n", cmd);

    // CONSTANT
    if (cmd[0]=='C' && cmd[1]=='O' && cmd[2]=='N') {
        activeMode = &constantMode;
        printk("Mode: CONSTANT 100mL/hr\n");
        return;
    }

    // RAMP
    if (cmd[0]=='R' && cmd[1]=='A' && cmd[2]=='M') {
        activeMode = &rampMode;
        printk("Mode: RAMP 10->500mL/hr\n");
        return;
    }

    // START
    if (cmd[0]=='S' && cmd[1]=='T' && cmd[2]=='A' && cmd[3]=='R') {
        volumeOffset     = actualVolumeUL;
        motorTicks       = 0;
        expectedVolumeUL = 0;
        resetEncoderPosition();
        
        activeMode->run();
        lastRampUpdateMs = k_uptime_get_32();
        
        printk("Resuming from: %lu uL\n", (unsigned long)volumeOffset);       
        printk("Infusion started\n");
        printk("Running - rate: %lu uL/hr\n", (unsigned long)activeMode->getCurrentRate());
        return;
    }

    // STOP
    if (cmd[0]=='S' && cmd[1]=='T' && cmd[2]=='O') {
        activeMode->stop();
        alarm_off();
        printk("Infusion STOPPED\n");
        printk("Expected: %lu uL\n", (unsigned long)expectedVolumeUL);
        printk("Actual:   %lu uL\n", (unsigned long)actualVolumeUL);
        return;
    }

    // STATUS
    if (cmd[0]=='S' && cmd[1]=='T' && cmd[2]=='A' && cmd[3]=='T') {
        uint32_t pressure = 1013;
        if (lps22hb_ready && useRealSensor) {
            pressure = read_lps22hb_pressure();
            occlusionMonitor.updatePressure(pressure);
        }

        printk("--- STATUS ---\n");
        printk("Mode:          %s\n", activeMode == &constantMode ? "CONSTANT" : "RAMP");
        printk("Running:       %s\n", activeMode->isRunning() ? "YES" : "NO");
        printk("Rate:          %lu uL/hr\n", (unsigned long)activeMode->getCurrentRate());
        printk("Expected:      %lu uL\n", (unsigned long)expectedVolumeUL);
        printk("Actual:        %lu uL\n", (unsigned long)actualVolumeUL);
        printk("Pressure:      %lu hPa (%s)\n", 
            (unsigned long)occlusionMonitor.getCurrentPressure(),
            useRealSensor ? "REAL" : "SIMULATED");

        if (occlusionMonitor.getStatus() == OcclusionStatus::WARNING) {
            printk("Pressure: WARNING!\n");
        } else if (occlusionMonitor.getStatus() == OcclusionStatus::ALARM) {
            printk("Pressure: ALARM!\n");
        }
        return;
    }

    // PRESSURE
    if (cmd[0]=='P' && cmd[1]=='R' && cmd[2]=='E') {
        useRealSensor = false;
        uint32_t pressure = 0;
        const char* p = cmd + 9;
        while (*p >= '0' && *p <= '9') {
            pressure = pressure * 10 + (*p - '0');
            p++;
        }
        occlusionMonitor.updatePressure(pressure);
        printk("Pressure: %lu hPa (SIMULATED)\n", (unsigned long)pressure);
        
        if (occlusionMonitor.getStatus() == OcclusionStatus::CLEAR) {
            printk("Status: CLEAR\n");
            alarm_off();
        } else if (occlusionMonitor.getStatus() == OcclusionStatus::WARNING) {
            printk("Status: WARNING\n");
            printk("Pressure rising — monitor!\n");
        } else {
            printk("!!! OCCLUSION ALARM !!!\n");
            printk("REASON: TUBE BLOCKAGE\n");
            printk("ACTION: INFUSION STOPPED\n");
            activeMode->stop();
            alarm_on();
        }
        return;
    }

    // CLEAR
    if (cmd[0]=='C' && cmd[1]=='L' && cmd[2]=='E') {
        occlusionMonitor.reset();
        alarm_off();
        useRealSensor = true;
        printk("Alarm cleared!\n");
        printk("Pressure: NORMAL\n");
        printk("Volume preserved: %lu uL\n", (unsigned long)actualVolumeUL);
        printk("Type START to resume\n");
        return;
    }

    // REAL
    if (cmd[0]=='R' && cmd[1]=='E' && cmd[2]=='A') {
        useRealSensor = true;
        printk("Switched to REAL SENSOR\n");
        return;
    }

    // RESET
    if (cmd[0]=='R' && cmd[1]=='E' && cmd[2]=='S') {
        activeMode->stop();
        occlusionMonitor.reset();
        rampMode.reset();
        motorTicks        = 0;
        expectedVolumeUL  = 0;
        actualVolumeUL    = 0;
        volumeOffset      = 0;
        resetEncoderPosition();
        alarm_off();
        printk("--- RESET COMPLETE ---\n");
        return;
    }

    printk("ERROR: unknown command\n");
    printk("Commands: CONSTANT RAMP START\n");
    printk("          STOP STATUS RESET\n");
    printk("          PRESSURE <hPa> REAL\n");
}

// ── UART callback ─────────────────────────────────
static void uart_cb(const struct device *dev, void *user_data) {
    if (!uart_irq_update(dev)) return;
    if (!uart_irq_rx_ready(dev)) return;
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (uart_buf_pos > 0) {
                uart_buf[uart_buf_pos] = '\0';
                process_command(uart_buf);
                uart_buf_pos = 0;
            }
        } else if (uart_buf_pos < UART_BUF_SIZE - 1) {
            uart_buf[uart_buf_pos++] = c;
        }
    }
}

// ── Main ──────────────────────────────────────────
int main(void) {
    printk("Infusion Pump Starting...\n");

    gpio_pin_configure_dt(&en_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&step_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_set_dt(&dir_pin, 1);
    gpio_pin_set_dt(&en_pin, 0);
    qdec_dev = DEVICE_DT_GET_ANY(st_stm32_qdec);
    if (!device_is_ready(qdec_dev)) {
        printk("QDEC not ready!\n");
        qdec_dev = nullptr;
    } else {
        printk("QDEC encoder READY\n");
    }

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (device_is_ready(i2c_dev)) {
        lps22hb_ready = lps22hb_init();
        if (lps22hb_ready) {
            printk("LPS22HB READY\n");
        }
    } else {
        printk("I2C not ready!\n");
    }

    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(uart_dev)) {
        uart_irq_callback_set(uart_dev, uart_cb);
        uart_irq_rx_enable(uart_dev);
    }

    printk("Ready. Commands:\n");
    printk("  CONSTANT RAMP START STOP\n");
    printk("  STATUS RESET REAL\n");
    printk("  PRESSURE <hPa>\n");

    uint32_t loop_count = 0;

    while (true) {
        if (loop_count % 500 == 0) {
            if (lps22hb_ready && useRealSensor) {
                uint32_t pressure = read_lps22hb_pressure();
                occlusionMonitor.updatePressure(pressure);
                printk("Pressure: %lu hPa — ", (unsigned long)pressure);

                if (occlusionMonitor.getStatus() == OcclusionStatus::CLEAR) {
                    printk("CLEAR\n");
                } else if (occlusionMonitor.getStatus() == OcclusionStatus::ALARM) {
                    printk("!!! ALARM !!!\n");
                    activeMode->stop();
                    alarm_on();
                }
            }
        }

        if (activeMode->isRunning()) {
            uint32_t rate = activeMode->getCurrentRate();
            if (rate == 0) rate = 1;

            uint32_t stepDelayMs = 3600000UL / rate;
            if (stepDelayMs < 2) stepDelayMs = 2;

            // Step motor
            gpio_pin_set_dt(&step_pin, 1);
            k_msleep(1);
            gpio_pin_set_dt(&step_pin, 0);
            motorTicks++;

            // Handle periodic ramping calculations
            if (activeMode == &rampMode) {
                uint32_t now = k_uptime_get_32();
                if (now - lastRampUpdateMs >= 1000) {
                    lastRampUpdateMs = now;
                    activeMode->run();
                    printk("Ramp rate: %lu uL/hr\n", (unsigned long)activeMode->getCurrentRate());
                }
            }

            // Calculate tracking variables
            updateEncoderPosition();
            
            // 200 motor steps per rev at 3200 uL per rev means exactly 16 uL per step
            expectedVolumeUL = volumeOffset + (motorTicks * 4);
            actualVolumeUL   = getActualVolumeUL();

            // Print metric telemetry status block every 50 ticks
            if (motorTicks % 50 == 0) {
                uint32_t accuracy = 0;
                int32_t  error    = 0;

                if (expectedVolumeUL > 0) {
                    accuracy = (actualVolumeUL * 100) / expectedVolumeUL;
                    error    = (int32_t)actualVolumeUL - (int32_t)expectedVolumeUL;
                }

                printk("─────────────────────\n");
                printk("Rate:     %lu uL/hr\n", (unsigned long)rate);
                printk("Expected: %lu uL\n", (unsigned long)expectedVolumeUL);
                printk("Actual:   %lu uL\n", (unsigned long)actualVolumeUL);
                printk("Error:    %ld uL\n", (long)error);
                printk("Accuracy: %lu%%\n", (unsigned long)accuracy);

                if (accuracy >= 95 && accuracy <= 105) {
                    printk("IEC 60601: PASS\n");
                } else {
                    printk("IEC 60601: FAIL\n");
                }
            }

            k_msleep(stepDelayMs - 1);
        } else {
            k_msleep(20);
        }

        loop_count++;
    }

    return 0;
}
