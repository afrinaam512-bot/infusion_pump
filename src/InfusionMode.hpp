#pragma once
#include <cstdint>
// parent class that inherits the  constant and linear ramp mode
 class InfusionMode {
// it  stores min ml/hr and max ml/hr variables beloe 1 ml the pump neverr runs 
public: 
    static constexpr uint32_t MIN_RATE_UL_HR = 1000;
    static constexpr uint32_t MAX_RATE_UL_HR = 500000;
// constructor which is used to initialize the variables that can be call by child class the ame of constructor should be class name 
// once the ocject is created  CHILD CLASSES IT BE INITIALIZED THE VARIABLES ARE currentrate_,running_,alarmcount_ variables  
InfusionMode();
//destructor used to delete the object and it is used virtual is needed then nly can be able to call the child classes
// and it is default now we didnt use thAT    
virtual ~InfusionMode() = default;
// PUBLIC FUNCTION 
//THIS RUN() ITS START INFUSING 
    void run();
//ITS STOP INFUSING
    void stop();
// IT CHECK AS MOTOR RUNNING AND RETURNS TRUE OR FALSE 
    bool isRunning() const;
//IT RETURNS CURRENT STATE 
uint32_t getCurrentRate() const;
// IT RETURN ALARM COUNT 
    uint32_t getAlarmCount() const;
// PROTECTED FUNCTION 
protected:
 //ITS ONLY CALL CHILD CLASSIt is marked virtual to support safe polymorphism
// COMPUTE TARGET RATE IT RETURNS TARGET RATE 
    virtual uint32_t computeTargetRate() = 0;
// IT BASICALLY currentrate=rate 
    virtual void applyRate(uint32_t rate) = 0;
// its checking checking below rate or above rate if yes  it gets alarmcount++
    void checkAlarms();
// it store current rate 
    uint32_t currentRate_;

private:
// it returns motor running or not 
    bool running_;
// it give counting alarm 
    uint32_t alarmCount_;
};
