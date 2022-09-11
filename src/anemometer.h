// definitions for anemometer stuff

// ROTOR 60/40 -> D = 60, Rrc = 40
#define A 0.4313f
#define B 0.0702f

const int N_PULSES_PER_REV = 3;
const float Ar = A * N_PULSES_PER_REV;

int current_second_rotations = 0;
int rotation_frequency = 0;
