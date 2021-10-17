#ifndef __WAVETABLE__
#define __WAVETABLE__
// Sample name: CP-80 EP C4
// Sample's base frequency: 262.2420997592405 Hz
// Sample's sample rate: 32000 Hz
#define WAVETABLE_LEN 8338
#define WAVETABLE_ATTACK_LEN 8216
#define WAVETABLE_LOOP_LEN 122

#ifndef __ASSEMBLER__
#include <stdint.h>
extern const int16_t WaveTable[WAVETABLE_LEN+1];
extern const uint16_t WaveTable_Increment[];
#else
.extern	WaveTable
.extern WaveTable_Increment
#endif

#endif