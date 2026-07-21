#pragma once
#include "InfusionMode.hpp"

class ConstantRateMode : public InfusionMode {
public:
    explicit ConstantRateMode(uint32_t targetRateUlHr);
    void setRate(uint32_t rateUlHr);
    uint32_t getTargetRate() const;

protected:
    uint32_t computeTargetRate() override;
    void applyRate(uint32_t rate) override;

private:
    uint32_t targetRate_;
};
