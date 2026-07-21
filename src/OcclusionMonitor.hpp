#pragma once
#include <cstdint>

enum class OcclusionStatus {
    CLEAR,
    WARNING,
    ALARM
};

class OcclusionMonitor {
public:
    static constexpr uint32_t WARNING_THRESHOLD_HPA = 1014;
    static constexpr uint32_t ALARM_THRESHOLD_HPA   = 1020;

    OcclusionMonitor();

    void updatePressure(uint32_t pressureHpa);
    OcclusionStatus getStatus() const;
    uint32_t getCurrentPressure() const;
    bool isAlarming() const;
    uint32_t getAlarmCount() const;
    void reset();

private:
    uint32_t currentPressure_;
    OcclusionStatus status_;
    uint32_t alarmCount_;
};
