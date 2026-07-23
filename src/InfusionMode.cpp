#include "InfusionMode.hpp"
// BY USING THIS FILE WE CAN ABLE TO INCLUSE DECLARING FUNCTIONS OTHERWISE THE IT DOESNT KNOW WHAT INFUSION MODE IS?

// Constructor.
// It is automatically called when an object of InfusionMode or its child class is created.
// It initializes the member variables.
InfusionMode::InfusionMode()
// NOW THW CURRENT RATE IS 0
: currentRate_(0)
// MOTOR IS NOT RUNNING 
    , running_(false)
// NOW NO ALARM SO IT IS 0
    , alarmCount_(0) {}
// THIS RUN() CALLAS MAIN.CPP THE ONLY IT RUNS  LIKE ACTIVE MODE 
void InfusionMode::run() 
 WHILE RUNNING IS TRUE THE PUMP IS STARTED 
{
    running_ = true;
    //  IT GETTING TARGET RAETE 
    uint32_t target = computeTargetRate(); 
    // APPLY RATE= TARGET RATE 
    applyRate(target);
    // ITS CHECKING THE RATE IS BELOW OR ABOVE THE RATE 
    checkAlarms();
}
// THIS FUCTIONS CALLS BY MAIN .CPP TS STOPS THE INFUSION 
void InfusionMode::stop() {
    WHEN IT FALSE IT STOPS 
    running_ = false;
    // HERE THE CURRENT RATE IS 0 
    currentRate_ = 0;
}
// IT CHECKING AND RETURNS THE MOTOR IS RUNNING OR NOT TRUE OR FDALSE 
bool InfusionMode::isRunning() const {
    return running_;
}
// IT RETURNS CURRENT RATE 
uint32_t InfusionMode::getCurrentRate() const {
    return currentRate_;
}
// IT RETURNS NUMBER OF ALARM COUNT DETECTED 
uint32_t InfusionMode::getAlarmCount() const {
    return alarmCount_;
}
// ITS CHECKING AS  THE CURRENT RATE IS ABOVE OR BELOW THE RATE IF IT YES ALARM IS INITIALIZED
void InfusionMode::checkAlarms() {
    if (currentRate_ > MAX_RATE_UL_HR) {
        alarmCount_++;
    }
    if (currentRate_ < MIN_RATE_UL_HR && running_) {
        alarmCount_++;
    }
}
