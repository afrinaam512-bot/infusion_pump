#pragma once
#include <cstdint>

class InfusionMode {
public:
    static constexpr uint32_t MIN_RATE_UL_HR = 1000;
    static constexpr uint32_t MAX_RATE_UL_HR = 500000;

    InfusionMode();
    virtual ~InfusionMode() = default;

    void run();
    void stop();
    bool isRunning() const;
    uint32_t getCurrentRate() const;
    uint32_t getAlarmCount() const;

protected:
    virtual uint32_t computeTargetRate() = 0;
    virtual void applyRate(uint32_t rate) = 0;
    void checkAlarms();

    uint32_t currentRate_;

private:
    bool running_;
    uint32_t alarmCount_;
};
