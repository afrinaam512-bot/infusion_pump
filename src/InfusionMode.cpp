#include "InfusionMode.hpp"

InfusionMode::InfusionMode()
    : currentRate_(0)
    , running_(false)
    , alarmCount_(0) {}

void InfusionMode::run() {
    running_ = true;
    uint32_t target = computeTargetRate();
    applyRate(target);
    checkAlarms();
}

void InfusionMode::stop() {
    running_ = false;
    currentRate_ = 0;
}

bool InfusionMode::isRunning() const {
    return running_;
}

uint32_t InfusionMode::getCurrentRate() const {
    return currentRate_;
}

uint32_t InfusionMode::getAlarmCount() const {
    return alarmCount_;
}

void InfusionMode::checkAlarms() {
    if (currentRate_ > MAX_RATE_UL_HR) {
        alarmCount_++;
    }
    if (currentRate_ < MIN_RATE_UL_HR && running_) {
        alarmCount_++;
    }
}
