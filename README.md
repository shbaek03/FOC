# FOC_Prototype
This code is prototype of the FOC algorithm based on B-G431B-ESC1 development board with A2212 13T(1000KV) BLDC motor and AS5047 magnetic encoder.

I've used this code as the reference for the improved & organized module structure.

------

### The prototype code includes functions below.
- Main FOC algorithm
  - FOC calculation (including Park, Clarke transform)
  - SVPWM (Space-Vector PWM)
- Rotor position sensing
  - SPI magnetic encoder (Bit-Banging method)
  - Incremental magnetic encoder (HW supported ABZ method)
  - Sensorless (SMO, Sliding Mode Observer)
- Control logic
  - Cascade (Position & Velocity)
  - S-curve profile (Position & Velocity)
