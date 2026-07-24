#include "ConstantRateMode.hpp"

// the funntion belongs to the constantratemode class and it uses member initializer 
ConstantRateMode::ConstantRateMode(uint32_t targetRateUlHr)
    : targetRate_(targetRateUlHr) {}
// it is belongs to constant rate mode class so the set rate functionn stores rate of motor
// this function is called whenever the new flow rate is selected 
void ConstantRateMode::setRate(uint32_t rateUlHr) {
    targetRate_ = rateUlHr;
}
// in get target rate() funnction that returns target rate 
uint32_t ConstantRateMode::getTargetRate() const {
    return targetRate_;
}
// comput target rate also returns target rate 
uint32_t ConstantRateMode::computeTargetRate() {
    return targetRate_;
}
// apply rate funtion it stores the rate into current rate

// Applies the computed rate by storing it in currentRate_.
// The motor control logic uses currentRate_ to generate
// the required stepping speed.
void ConstantRateMode::applyRate(uint32_t rate) {
    currentRate_ = rate;
}
