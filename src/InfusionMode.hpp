#pragma once
#include <cstdint>
// parent class that inherits the  constant and linear ramp mode
 class InfusionMode {
// it  stores min ml/hr and max ml/hr variables beloe 1 ml the pump neverr runs 
public: 
    static constexpr uint32_t MIN_RATE_UL_HR = 1000;
    static constexpr uint32_t MAX_RATE_UL_HR = 500000;
// constructor which is used to initialize the  membervariables this constructor is automatically called whenever neededan infusion mode derived object is createds name 
// once the ocject is created  CHILD CLASSES IT BE INITIALIZED THE VARIABLES ARE currentrate_,running_,alarmcount_ variables  
//The child class doesn't call it directly. It is automatically called before the child constructor executes.
InfusionMode();
//destructor used to delete the object and it is used virtual is needed then nly can be able to call the child classes
// and it is default now we didnt use thAT Virtual ensures the correct derived class destructor is called when deleting through a base-class pointer.  
virtual ~InfusionMode() = default;
// PUBLIC FUNCTION 
//THIS RUN() ITS run() does not directly start the motor.

//It

//computes target rate
//applies the rate
//checks alarms
    void run();
//ITS STOP INFUSING
    void stop();
// IT CHECK AS MOTOR RUNNING AND RETURNS TRUE OR FALSE 
    bool isRunning() const;
//IT RETURNS CURRENT RATE
uint32_t getCurrentRate() const;
// IT RETURN THE NUMBER OF  ALARM COUNT 
    uint32_t getAlarmCount() const;
// PROTECTED FUNCTION 
protected:
 /// Pure virtual function.
// Each child class must implement how the
// target infusion rate is calculated.It is marked virtual to support safe polymorphism
// COMPUTE TARGET RATE IT RETURNS TARGET RATE 
    virtual uint32_t computeTargetRate() = 0;
// IT BASICALLY currentrate=rate 
    virtual void applyRate(uint32_t rate) = 0;
// its checking checking below rate or above rate if yes  it gets alarmcount++
    void checkAlarms();
// it store current rate 
    uint32_t currentRate_;

private:
// t stores whether the infusion mode is running.
    bool running_;
// it give counting alarm 
    uint32_t alarmCount_;
};
