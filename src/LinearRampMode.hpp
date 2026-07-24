#pragma once
#include "InfusionMode.hpp"
//  i comes from inherits rom that class 
class LinearRampMode : public InfusionMode {
public:
// it s a constructor // startRateulhr means the   start rate 
// these are function paasing wuth parameters 
    LinearRampMode(uint32_t startRateUlHr,
                   uint32_t endRateUlHr,// maximum rate 
                   uint32_t stepUlHr);// update amount rate of the every pulse 
// it also function  it returns the stsrt rate 
    uint32_t getStartRate() const;
//funtciion it returns the  max rate 
    uint32_t getEndRate()   const;
// it eturns and updte the next step 
    uint32_t getStepSize()  const;
// it getting reset 
    void reset();
// it only calls run() function 
protected:
    uint32_t computeTargetRate() override;
    void applyRate(uint32_t rate) override;

private:
// it a data variable that stores the value 
    uint32_t startRate_;
    uint32_t endRate_;
    uint32_t stepSize_;
    uint32_t currentTarget_;
};
