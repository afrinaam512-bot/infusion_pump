#pragma once
#include "InfusionMode.hpp"

class LinearRampMode : public InfusionMode {
public:
    LinearRampMode(uint32_t startRateUlHr,
                   uint32_t endRateUlHr,
                   uint32_t stepUlHr);

    uint32_t getStartRate() const;
    uint32_t getEndRate()   const;
    uint32_t getStepSize()  const;
    void reset();

protected:
    uint32_t computeTargetRate() override;
    void applyRate(uint32_t rate) override;

private:
    uint32_t startRate_;
    uint32_t endRate_;
    uint32_t stepSize_;
    uint32_t currentTarget_;
};
