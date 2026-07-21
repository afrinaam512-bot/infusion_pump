#pragma once
#include <cstdint>

class EncoderVolumeTracker {
public:
    static constexpr uint32_t PULSES_PER_REV = 600;
    static constexpr uint32_t UL_PER_REV     = 3200;

    EncoderVolumeTracker();

    void tick(bool directionForward = true);
    uint32_t getVolumeUL() const;
    uint32_t getPulseCount() const;
    void reset();

private:
    uint32_t pulseCount_;
    uint32_t totalPulses_;
};
