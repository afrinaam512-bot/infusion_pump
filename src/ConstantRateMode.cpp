#include "ConstantRateMode.hpp"

ConstantRateMode::ConstantRateMode(uint32_t targetRateUlHr)
    : targetRate_(targetRateUlHr) {}

void ConstantRateMode::setRate(uint32_t rateUlHr) {
    targetRate_ = rateUlHr;
}

uint32_t ConstantRateMode::getTargetRate() const {
    return targetRate_;
}

uint32_t ConstantRateMode::computeTargetRate() {
    return targetRate_;
}

void ConstantRateMode::applyRate(uint32_t rate) {
    currentRate_ = rate;
}
