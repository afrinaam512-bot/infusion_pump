#include "OcclusionMonitor.hpp"

OcclusionMonitor::OcclusionMonitor()
    : currentPressure_(1013)
    , status_(OcclusionStatus::CLEAR)
    , alarmCount_(0) {}

void OcclusionMonitor::updatePressure(uint32_t pressureHpa) {
    currentPressure_ = pressureHpa;

    if (pressureHpa >= ALARM_THRESHOLD_HPA) {
        if (status_ != OcclusionStatus::ALARM) {
            alarmCount_++;
        }
        status_ = OcclusionStatus::ALARM;
    } else if (pressureHpa >= WARNING_THRESHOLD_HPA) {
        status_ = OcclusionStatus::WARNING;
    } else {
        status_ = OcclusionStatus::CLEAR;
    }
}

OcclusionStatus OcclusionMonitor::getStatus() const {
    return status_;
}

uint32_t OcclusionMonitor::getCurrentPressure() const {
    return currentPressure_;
}

bool OcclusionMonitor::isAlarming() const {
    return status_ == OcclusionStatus::ALARM;
}

uint32_t OcclusionMonitor::getAlarmCount() const {
    return alarmCount_;
}

void OcclusionMonitor::reset() {
    currentPressure_ = 1013;
    status_ = OcclusionStatus::CLEAR;
    alarmCount_ = 0;
}
