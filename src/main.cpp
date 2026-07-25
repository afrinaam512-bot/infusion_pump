#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
// This header provides the Zephyr Sensor API, which is used to access the STM32 QDEC driver
// and read the encoder count.
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include "InfusionMode.hpp"
#include "ConstantRateMode.hpp"
#include "LinearRampMode.hpp"
#include "EncoderVolumeTracker.hpp"
#include "OcclusionMonitor.hpp"
// this gpio_dt_spec ia a zephyr structur ehat stores the all information about the gpio pin 
// struct - it is a user defined data type that groups multiple variable togtther
//const - the variable value cannot be modified after the  once it is created since the led is never changeable   
//static is a global variable in.cpp file  only this maincpp can access it 
// led- this a simpple variable name 
// ── GPIO specs ────────────────────────────────────
static const struct gpio_dt_spec led =
//It searches the Device Tree for the alias named led0.
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
     // these are zephyr api the fuction calls api then gpio driver then stm32  hardware 
    gpio_pin_set_dt(&led,    1);
    gpio_pin_set_dt(&buzzer, 1);
}

static void alarm_off(void) {
    gpio_pin_set_dt(&led,    0);
    gpio_pin_set_dt(&buzzer, 0);
} 



// ── Encoder globals & Physical Constants ──────────
//static → Variable is accessible only within main.cpp.
//const → The device object itself cannot be modified.
//struct device → Zephyr's device structure representing a hardware device.

//* → Pointer to the device.
//t stores the address of the QDEC (Quadrature Decoder) device.
static const struct device *qdec_dev;
//Stores the previous raw encoder count.
static int32_t  g_encoderRawPrev     = 0;
//after the first encoder is intialized then only it gets true before it running it retuuns false
static bool     g_encoderInitialized = false;
//it stores the last total  encoder count 
static int32_t  g_lastEncoderCount   = 0; // Cumulative degrees tracked
// it stores Stores the flow rate set by the user. = 100ml/hr 
static uint32_t userFlowRateUlHr  = 100000; // default 100mL/hr
// no dose rtaregt is set yet now 
static uint32_t userDoseTargetUL  = 0;// 0 = no target
//This is a macro, not a variable. for completerotation in degrees as 360  
#define ENCODER_DEG_PER_REV  360
// it delivers 3.2 ml for full rotation 
#define UL_PER_REV           3200
//Counts how many motor steps have been generated.
static uint32_t motorTicks      = 0;
//Stores the expected volume.
//This is the theoretical volume based on the number of motor steps.
static uint32_t expectedVolumeUL = 0;
//Stores the actual delivered volume.
//It is calculated using the encoder pulses.
static uint32_t actualVolumeUL    = 0;
// IT STORES CORRECTION VOLUME AND IT COMPENSATE DIFFERENCE BETWEEN  EXPECTED AND ACTUAL 
static uint32_t volumeOffset     = 0;
//Stores the last time the ramp speed was updated.
static uint32_t lastRampUpdateMs = 0;
//Stores the maximum ramp flow rate. 500ML
static uint32_t userMaxRampRateUlHr = 500000;


Power ON
    │
    ▼
Initialize I2C
    │
    ▼
Initialize LPS22HB
    │
    ▼
Check WHO_AM_I register
    │
    ▼
Configure sensor
    │
    ▼
Read pressure periodically
    │
    ▼
Pressure (hPa)
    │
    ▼
Occlusion detection
// ── I2C / LPS22HB ────────────────────────────────
static const struct device *i2c_dev;
//This is the I²C address of the pressure sensor.//When the STM32 wants to communicate with the LPS22HB, it sends:
#define LPS22HB_ADDR         0x5C
//This is a register address inside the sensor.
#define LPS22HB_PRESS_OUT_XL 0x28
//This is another register.
//Its purpose is to identify the sensor.
//Reading this register should return
#define LPS22HB_WHO_AM_I     0x0F
//This is the control register.
//Writing values into this register configures the sensor.
#define LPS22HB_CTRL_REG1    0x10
//Sensor not initialized.//  when it is intialized it gets true 
static bool lps22hb_ready = false;
// this function initializes the sensor it did three jobs detect sensor configure sensor return sucess / fsilure  
static bool lps22hb_init(void) 
//Creates an 8-bit variable.
//It will store the value read from the WHO_AM_I register.
    uint8_t who_am_i = 0;
//Stores
//0x0F
//This tells the sensor
//Read register 0x0F
    uint8_t reg = LPS22HB_WHO_AM_I;
// this is a zephyr i2c api (i2c_write_read) it performs read and write function 
// it write 0 x 0F > LPSR22HB > RETURNS > 0XB1 IT STORED IN AN WHO AM I 
// IF THE CONNECTION FILE LPS NOT FOUND 
    if (i2c_write_read(i2c_dev, LPS22HB_ADDR, &reg, 1, &who_am_i, 1) != 0) {
        printk("LPS22HB not found!\n");
        return false;
    }
    printk("LPS22HB WHO_AM_I: 0x%02X\n", who_am_i);
// IT CREATES TWO BYES REGISTER ADRES  AS 0X10 AND OTHER BYTE AS CTRL REG1 0X30 
    uint8_t cfg[2] = {LPS22HB_CTRL_REG1, 0x30};
// IT WRITES 0X3O INTO CTRL REG
    i2c_write(i2c_dev, cfg, 2, LPS22HB_ADDR);
    return (who_am_i == 0xB1);
}//"The LPS22HB stores each pressure measurement in three 8-bit registers: 
//PRESS_OUT_XL (0x28), PRESS_OUT_L (0x29), and PRESS_OUT_H (0x2A). Since each register is 8 bits, the total pressure data size is 24 bits (3 × 8 = 24). 
    //My code reads these three bytes over I²C and combines them into a single 24-bit raw pressure value before converting it to pressure in hPa."

static uint32_t read_lps22hb_pressure(void) // IT READ PRESSURE FROM SENSOR {
    if (!lps22hb_ready) return 1013;// IF THE SENSOR NOT READ IT RETURS 1013 BCZ IT A ATMOSPHERIC NORMAL VALUE 
    uint8_t buf[3];//Creates a buffer of three bytes because the pressure value is stored across three registers.
    uint8_t reg = LPS22HB_PRESS_OUT_XL | 0x80;//0X 80 ENABLES AUTO INCREAMET 
    if (i2c_write_read(i2c_dev, LPS22HB_ADDR, &reg, 1, buf, 3) != 0) {
        return 1013;
    }
// THESE COMBINES THE 24 BIT RAW VALUE 2^24
    int32_t raw = ((int32_t)buf[2] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[0];
// IF SUPPOSEIT RETUS BUF[1]=0X3F,0X56,0X45 COBINES RAW VALUE  = 0X3F5645 IT RETURNS THE DECIMAL AS raw = 0x3F5800
//Decimal = 4,151,296 IT IIS NOT PRESSUREHPA  VALUE IT A RAW  PRESSURE VALUE THAT NEED TO DIVIDE BY 4096 IT GIVEN BY DATA SHEET  THEN ONLY WE GET AS 1013 HPA 
    return (uint32_t)(raw / 4096);
}

// ── Static instances — no heap ───────────────────
//static → The object exists for the entire program execution.ConstantRateMode → This is the class (data type).
//constantMode → This is the object name
//(100000) → Calls the constructor with 100000 µL/hr.
static ConstantRateMode  constantMode(100000);// its the class that create s object that call the constructor which initializes the targetrate_
static LinearRampMode    rampMode(10000, 500000, 10000);// lits the class that creates the ramp mode object with calls constructor that passes stsrtrate_,endrate_,stepsize_
static OcclusionMonitor  occlusionMonitor;// ccalls class with object no paramete so it initialized  currentpressure_, status_, alarmcount_
static InfusionMode*     activeMode    = &constantMode;// it uses a pointer the pointer doesnt stores the object name its  
//stores the address of the object  here infusion mode object address is stored in   activmode variable here the infusionmode variable is constantMode and linearmode  
static bool              useRealSensor = true;// it returns real data or simulateed data 
// this is a preprocessor macro evry time it stores 32 charcters // the maximum number of characters that command contains 
// ── UART buffer ───────────────────────────────────
#define UART_BUF_SIZE 32
// it stores the uart  character 
static char   uart_buf[UART_BUF_SIZE]; 
//keep track of where the next character to be stored  
static uint8_t uart_buf_pos = 0;
// pointer to the uart hardware device that obtain from the zephyr 
static const struct device *uart_dev;
// how the uart means it is communication interface that share the to the two devices one character at a time// whenever the START IS TYPE 
// IT GOES ONE BY ONE CHARACTER THAT STORED N AN UART_BUF  IT DENDS TO UART STM32 HARDWARE THEN ZEPHYR UART DRIVER UART INTERRECPT, STORES CHARACTER IN UART_BUF[]
// WHEN/n  enter is reciver it goes into command processor funtion that startfunction pump state machine motor starts 
// ── Encoder Helper Functions ──────────────────────//These helper functions are responsible for reading the encoder position from the Zephyr QDEC driver.
static int32_t readEncoderRaw(void) { // this functonly used in main file noo any other // it reads the encoder raw data 
    if (!qdec_dev) return 0;// later it get initialized if the encoderis not then it reyurn 0 
    struct sensor_value val;// it is a zeohyr structure // it stores sensor data // val is the object 
    sensor_sample_fetch(qdec_dev);//This is a Zephyr Sensor API function it tells the qdec "Read the latest value from the hardware.
    sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &val);//this ask the driver for getting the rotation
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
  // SET_RATE <dose_mL> <rate_mL_per_hr>
// Example: SET_RATE 10 100
if (cmd[0]=='S' && cmd[1]=='E' && cmd[2]=='T') {
    uint32_t dose = 0;
    uint32_t rate = 0;
    const char* p = cmd + 9;

    // Parse dose (mL)
    while (*p >= '0' && *p <= '9') {
        dose = dose * 10 + (*p - '0');
        p++;
    }
    while (*p == ' ') p++;

    // Parse rate (mL/hr)
    while (*p >= '0' && *p <= '9') {
        rate = rate * 10 + (*p - '0');
        p++;
    }

    if (dose >= 1 && dose <= 500 &&
        rate >= 1 && rate <= 500) {
        userDoseTargetUL = dose * 1000;
        userFlowRateUlHr = rate * 1000;
        userMaxRampRateUlHr  = rate * 1000;
  rampMode = LinearRampMode(
            10000,
            userMaxRampRateUlHr,
            10000);



        // Update constant mode rate
        constantMode = ConstantRateMode(
            userFlowRateUlHr);

        printk("Dose target: %lu uL (%lu mL)\n",
            (unsigned long)userDoseTargetUL,
            (unsigned long)dose);
        printk("Flow rate:   %lu uL/hr (%lu mL/hr)\n",
            (unsigned long)userFlowRateUlHr,
            (unsigned long)rate);
    } else {
        printk("ERROR: dose 1-500mL rate 1-500mL/hr\n");
    }
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
START command
      │
      ▼
main() loop
      │
      ▼
gpio_pin_set_dt(step_pin, 1)
      │
      ▼
STEP pulse sent to TMC2209
      │
      ▼
Stepper motor moves one step
      │
      ▼
Encoder rotates
      │
      ▼
Volume is delivered
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
// Check target dose reached
if (userDoseTargetUL > 0 &&
    actualVolumeUL >= userDoseTargetUL) {
    activeMode->stop();
    alarm_on();
    printk("DOSE COMPLETE!\n");
    printk("Delivered: %lu uL\n",
        (unsigned long)actualVolumeUL);
    userDoseTargetUL = 0; // reset
}

    printk("Ready. Commands:\n");
    printk("  CONSTANT RAMP START STOP\n");
    printk("  STATUS RESET REAL\n");
    printk("  PRESSURE <hPa>\n");
   printk("  SET_RATE <mL> <mL/hr> → set dose and rate\n");
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
