#ifndef UTIL_H
#define UTIL_H

#define SAMPLERATE 31250u
#define PRNG_SEED 0x12345678

#define _rev2(x)  ((((x)&1)<<1) | (((x)>>1)&1))
#define _rev4(x)  ((_rev2(x)<<2) | (_rev2((x)>>2)))
#define _rev8(x)  ((_rev4(x)<<4) | (_rev4((x)>>4)))
#define _rev16(x) ((_rev8(x)<<8) | (_rev8((x)>>8)))
#define _rev(x) (uint16_t)(_rev16(x))

#define LCD_BLACK  _rev(TFT_BLACK)
#define LCD_WHITE  _rev(TFT_WHITE)
#define LCD_RED    _rev(TFT_RED)
#define LCD_GREEN  _rev(TFT_GREEN)
#define LCD_BLUE   _rev(TFT_BLUE)
#define LCD_YELLOW _rev(TFT_YELLOW)
#define LCD_PINK   _rev(TFT_PINK)
#define LCD_PURPLE _rev(TFT_PURPLE)
#define LCD_MODE   _rev(0xf000)

namespace UTIL
{
  static const uint32_t __not_in_flash_func(map)(const uint32_t x,const uint32_t in_min, const uint32_t in_max,const uint32_t out_min, const float out_max)
  {
    // unsigned map
    if (x<in_min)
    {
      return out_min;
    }
    if (x>in_max)
    {
      return out_max;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static const uint32_t quadrature_divisor(const uint32_t f)
  {
    if (f < 6850000ul)
    {
      return 126;
    }
    if (f < 9500000ul)
    {
      return 88;
    }
    if (f < 13600000ul)
    {
      return 64;
    }
    if (f < 17500000ul)
    {
      return 44;
    }
    if (f < 25000000ul)
    {
      return 34;
    }
    if (f < 36000000ul)
    {
      return 24;
    }
    if (f < 45000000ul)
    {
      return 18;
    }
    if (f < 60000000ul)
    {
      return 14;
    }
    if (f < 80000000ul)
    {
      return 10;
    }
    if (f < 100000000ul)
    {
      return 8;
    }
    if (f < 146600000ul)
    {
      return 6;
    }
    return 4;
  }

  static const uint32_t prng32(void)
  {
    volatile static uint32_t state = PRNG_SEED;
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (state = x);
  }

  static const int16_t __not_in_flash_func(ALaw2Linear)(const uint8_t aLaw)
  {
    const static int16_t ALawTab[256] =
    {
      5504, 5248, 6016, 5760, 4480, 4224, 4992, 4736,
      7552, 7296, 8064, 7808, 6528, 6272, 7040, 6784,
      2752, 2624, 3008, 2880, 2240, 2112, 2496, 2368,
      3776, 3648, 4032, 3904, 3264, 3136, 3520, 3392,
      22016, 20992, 24064, 23040, 17920, 16896, 19968, 18944,
      30208, 29184, 32256, 31232, 26112, 25088, 28160, 27136,
      11008, 10496, 12032, 11520, 8960, 8448, 9984, 9472,
      15104, 14592, 16128, 15616, 13056, 12544, 14080, 13568,
      344, 328, 376, 360, 280, 264, 312, 296,
      472, 456, 504, 488, 408, 392, 440, 424,
      88, 72, 120, 104, 24, 8, 56, 40,
      216, 200, 248, 232, 152, 136, 184, 168,
      1376, 1312, 1504, 1440, 1120, 1056, 1248, 1184,
      1888, 1824, 2016, 1952, 1632, 1568, 1760, 1696,
      688, 656, 752, 720, 560, 528, 624, 592,
      944, 912, 1008, 976, 816, 784, 880, 848,
      -5504, -5248, -6016, -5760, -4480, -4224, -4992, -4736,
      -7552, -7296, -8064, -7808, -6528, -6272, -7040, -6784,
      -2752, -2624, -3008, -2880, -2240, -2112, -2496, -2368,
      -3776, -3648, -4032, -3904, -3264, -3136, -3520, -3392,
      -22016, -20992, -24064, -23040, -17920, -16896, -19968, -18944,
      -30208, -29184, -32256, -31232, -26112, -25088, -28160, -27136,
      -11008, -10496, -12032, -11520, -8960, -8448, -9984, -9472,
      -15104, -14592, -16128, -15616, -13056, -12544, -14080, -13568,
      -344, -328, -376, -360, -280, -264, -312, -296,
      -472, -456, -504, -488, -408, -392, -440, -424,
      -88, -72, -120, -104, -24, -8, -56, -40,
      -216, -200, -248, -232, -152, -136, -184, -168,
      -1376, -1312, -1504, -1440, -1120, -1056, -1248, -1184,
      -1888, -1824, -2016, -1952, -1632, -1568, -1760, -1696,
      -688, -656, -752, -720, -560, -528, -624, -592,
      -944, -912, -1008, -976, -816, -784, -880, -848
    };
    return ALawTab[aLaw];
  }
}

#endif