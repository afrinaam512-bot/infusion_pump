#include "LinearRampMode.hpp"

LinearRampMode::LinearRampMode(uint32_t startRateUlHr,
                                uint32_t endRateUlHr,
                                uint32_t stepUlHr)
    : startRate_(startRateUlHr)
    , endRate_(endRateUlHr)
    , stepSize_(stepUlHr)
    , currentTarget_(startRateUlHr) {}

uint32_t LinearRampMode::getStartRate() const {
    return startRate_;
}

uint32_t LinearRampMode::getEndRate() const {
    return endRate_;
}

uint32_t LinearRampMode::getStepSize() const {
    return stepSize_;
}

void LinearRampMode::reset() {
    currentTarget_ = startRate_;
}

uint32_t LinearRampMode::computeTargetRate() {
    uint32_t rate = currentTarget_;
    if (currentTarget_ + stepSize_ <= endRate_) {
        currentTarget_ += stepSize_;
    } else {
        currentTarget_ = endRate_;
    }
    return rate;
}

void LinearRampMode::applyRate(uint32_t rate) {
    currentRate_ = rate;
}
