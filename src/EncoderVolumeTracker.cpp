#include "EncoderVolumeTracker.hpp"

EncoderVolumeTracker::EncoderVolumeTracker()
    : pulseCount_(0)
    , totalPulses_(0) {}

void EncoderVolumeTracker::tick(bool directionForward) {
    if (directionForward) {
        pulseCount_++;
        totalPulses_++;
    } else {
        if (pulseCount_ > 0) {
            pulseCount_--;
        }
    }
}

uint32_t EncoderVolumeTracker::getVolumeUL() const {
    return (pulseCount_ * UL_PER_REV) / PULSES_PER_REV;
}

uint32_t EncoderVolumeTracker::getPulseCount() const {
    return pulseCount_;
}

void EncoderVolumeTracker::reset() {
    pulseCount_ = 0;
    totalPulses_ = 0;
}
