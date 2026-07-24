
#pragma once
#include <cstdint>
// encoder volume tracker class  
class EncoderVolumeTracker {
public:
    static constexpr uint32_t PULSES_PER_REV = 600;// encoder generates 600 pulses for 1 revolution
    static constexpr uint32_t UL_PER_REV     = 3200; // for 1 revolution in ml it delivers 3.2ml
//this constuctor initializes the member variable pulsescount_ and toltall pulses  
// Called whenever one encoder pulse is detected.
    // directionForward=true means forward rotation.
    // If forward, pulse count increases.
    // If reverse, pulse count decreases.
EncoderVolumeTracker();

// for every tick of one pulse get incremet
    void tick(bool directionForward = true);
//it returns volume 
    uint32_t getVolumeUL() const; //it returns the volume = (pulse count * 600) /3200
    uint32_t getPulseCount() const;
    void reset();

private:
    uint32_t pulseCount_;
    uint32_t totalPulses_;
};
Stepper motor rotates
        │
        ▼
Encoder shaft rotates
        │
        ▼
Encoder generates quadrature pulses
        │
        ▼
STM32 Timer (QDEC Hardware)
        │
        ▼
Zephyr QDEC Driver counts the pulses
        │
        ▼
Your code calls tick()
        │
        ▼
pulseCount_ increases
        │
        ▼
getVolumeUL()
        │
        ▼
Volume = (pulseCount × 3200) / 600
        │
        ▼
Actual infused volume (µL)
