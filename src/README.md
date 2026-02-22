## Libraries
 *  All libraries included except those that come with arduino-pico
 
## Build
To build, I use Arduino IDE 2 with: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *  Generic RP2350
 *  CPU Speed: 240Mhz
 *  Optimize: -O3
 *  USB Stack: No USB
 *  Flash Size: 16MB (no FS)
 *  Chip Variant: "RP2350B"

## Some history
 *  0.1.240 start with MBPTRX code
 *  1.0.240 feature complete
 *  1.1.240 modify si5351 library to remove glitches above 100MHz
 *  1.2.240 set band defaults to ham frequencies
 *  1.3.240 add bandwidth setting to AM mode
 *  1.4.240 update CW processing to FS/4
