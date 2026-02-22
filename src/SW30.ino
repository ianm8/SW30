/*
 * Classic SW30 Receiver Version 1.4.240
 *
 * Copyright 2026 Ian Mitchell VK7IAN
 * Licenced under the GNU GPL Version 3
 *
 * Libraries
 *
 *  Libraries included!
 *
 * Filter Design
 *
 *  https://www.arc.id.au/FilterDesign.html
 *
 *
 * Build (Silicon Chip Dev Board):
 *  Generic RP2350
 *  CPU Speed: 240Mhz
 *  Optimize: -O3
 *  USB Stack: No USB
 *  Flash Size: 16MB (no FS)
 *  Chip Variant: "RP2350B"
 *
 * Some history
 *  0.1.240 start with MBPTRX code
 *  1.0.240 feature complete
 *  1.1.240 modify si5351 library to remove glitches above 100MHz
 *  1.2.240 set band defaults to ham frequencies
 *  1.3.240 add bandwidth setting to AM mode
 *  1.4.240 update CW processing to FS/4
 *
 * https://github.com/deftio/companders/tree/master
 *
 */

#if 0
#define DEBUGGING_SKIP
#endif

#include <I2S.h>
#include "util.h"
#include "si5351.h"
#include "Rotary.h"
#include "dsp.h"
#include "hardware/pwm.h"

#define TCXO_FREQ 27000000ul
#define MAIN_TUNE_STEP 1000ul
#define FINE_TUNE_STEP 50
#define FAST_TUNE_STEP 100000ul
#define FAST_FINE_STEP 10000ul
#define NUM_BANDS 6
#define CW_TONE 700

#define PIN_0        0 // not used
#define PIN_1        1 // not used
#define PIN_AUPWML   2 // audio PWM low
#define PIN_AUPWMH   3 // audio PWM high
#define PIN_DOUT     4 // I2S
#define PIN_BCLK     5 // I2S
#define PIN_LRCL     6 // I2S
#define PIN_MCLK     7 // I2S
#define PIN_8        8 // not used
#define PIN_9        9 // not used
#define PIN_10      10 // not used
#define PIN_11      11 // not used
#define PIN_BPF0    12 // BPF select (output)
#define PIN_BPF1    13 // BPF select (output)
#define PIN_BPF2    14 // BPF select (output)
#define PIN_BPF3    15 // BPF select (output)
#define PIN_SDA     16 // I2C
#define PIN_SCL     17 // I2C
#define PIN_18      18 // not used
#define PIN_19      19 // not used
#define PIN_BAND2   20 // band switch (input)
#define PIN_21      21 // not used
#define PIN_TUNE    22 // main tuning meter (analog output)
#define PIN_SMETER  23 // S meter (analog output)
#define PIN_BAND1   24 // band switch (input)
#define PIN_25      25 // onboard LED
#define PIN_BAND3   26 // band switch (input)
#define PIN_BAND4   27 // band switch (input)
#define PIN_BAND5   28 // band switch (input)
#define PIN_BAND6   29 // band switch (input)
#define PIN_FILTER1 30 // filter switch (input)
#define PIN_FILTER2 31 // filter switch (input)
#define PIN_FILTER3 32 // filter switch (input)
#define PIN_FILTER4 33 // filter switch (input)
#define PIN_MODE1   34 // mode switch (input)
#define PIN_MODE2   35 // mode switch (input)
#define PIN_MODE3   36 // mode switch (input)
#define PIN_MODE4   37 // mode switch (input)
#define PIN_MODE5   38 // mode switch (input)
#define PIN_MODE6   39 // mode switch (input)
#define PIN_JNR1    40 // JNR switch (input)
#define PIN_JNR2    41 // JNR switch (input)
#define PIN_JNR3    42 // JNR switch (input)
#define PIN_43      43 // not used
#define PIN_FINEA   44 // fine tune rotary (input)
#define PIN_FINEB   45 // fine tune rotary (input)
#define PIN_TUNEA   46 // main tune rotary (input)
#define PIN_TUNEB   47 // main tune rotary (input)

#define ERROR_SYSCLOCK 3u
#define ERROR_SI5351   4u

enum radio_mode_t
{
  MODE_LSB,
  MODE_USB,
  MODE_CWL,
  MODE_CWU,
  MODE_AM,
  MODE_SWL
};

volatile static uint32_t saved_frequency[NUM_BANDS] = 
{
   3600000ul,  // 0MHz - 5MHz
   7100000ul,  // 5MHz - 10MHz
  14200000ul, // 10MHz - 15MHz
  18100000ul, // 15MHz - 20MHz
  24100000ul, // 20MHz - 25MHz
  28400000ul  // 25MHz - 30MHz
};

volatile static struct
{
  uint32_t frequency;
  int32_t tune;
  int32_t fine;
  uint8_t band;
  uint8_t filter;
  uint8_t jnr;
  radio_mode_t mode;
}
radio =
{
  7100000ul,
  0,
  0,
  0,
  0,
  0,
  MODE_LSB
};

static const struct
{
  const uint32_t lo;
  const uint32_t hi;
}
bands[NUM_BANDS] =
{
  {  100000UL, 5000000UL},
  { 5000000UL, 10000000UL},
  {10000000UL, 15000000UL},
  {15000000UL, 20000000UL},
  {20000000UL, 25000000UL},
  {25000000UL, 30000000UL}
};

Si5351 SI5351;
Rotary t = Rotary(PIN_TUNEA,PIN_TUNEB);
Rotary f = Rotary(PIN_FINEA,PIN_FINEB);
I2S i2s(INPUT);

auto_init_mutex(rotary_mutex);
volatile static absolute_time_t dsp_process_next = 0ull;
volatile static uint32_t audio_pwm = 0;
volatile static bool setup_complete = false;

static void error_stop(const uint32_t err_code)
{
  // self-test error code
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  for (;;)
  {
    for (uint32_t i=0;i<err_code;i++)
    {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN,LOW);
      delay(250);
    }
    delay(1000);
  }
}

static void set_filter(void)
{
  // 4 control lines select one of 5 filters
  static uint8_t old_bpf = 0;
  uint8_t new_bpf = 0;
  if (radio.frequency<2000000UL)
  {
    // BPF 0.1MHz-2MHz
    new_bpf = 1;
  }
  else if (radio.frequency<4000000UL)
  {
    // BPF 2MHz-4MHz
    new_bpf = 2;
  }
  else if (radio.frequency<8000000UL)
  {
    // BPF 4MHz-8MHz
    new_bpf = 3;
  }
  else if (radio.frequency<16000000UL)
  {
    // BPF 8MHz-16MHz
    new_bpf = 4;
  }
  else
  {
    // BPF 16MHz-30MHz
    new_bpf = 5;
  }
  if (old_bpf != new_bpf)
  {
    old_bpf = new_bpf;
    switch (new_bpf)
    {
      case 1:
      {
        // BPF 0.1MHz-2MHz
        digitalWrite(PIN_BPF0,LOW);
        digitalWrite(PIN_BPF1,HIGH);
        digitalWrite(PIN_BPF2,HIGH);
        digitalWrite(PIN_BPF3,HIGH);
        break;
      }
      case 2:
      {
        // BPF 2MHz-4MHz
        digitalWrite(PIN_BPF0,HIGH);
        digitalWrite(PIN_BPF1,LOW);
        digitalWrite(PIN_BPF2,HIGH);
        digitalWrite(PIN_BPF3,HIGH);
        break;
      }
      case 3:
      {
        // BPF 4MHz-8MHz
        digitalWrite(PIN_BPF0,LOW);
        digitalWrite(PIN_BPF1,LOW);
        digitalWrite(PIN_BPF2,LOW);
        digitalWrite(PIN_BPF3,HIGH);
        break;
      }
      case 4:
      {
        // BPF 8MHz-16MHz
        digitalWrite(PIN_BPF0,LOW);
        digitalWrite(PIN_BPF1,LOW);
        digitalWrite(PIN_BPF2,HIGH);
        digitalWrite(PIN_BPF3,LOW);
        break;
      }
      case 5:
      {
        // BPF 16MHz-30MHz
        digitalWrite(PIN_BPF0,LOW);
        digitalWrite(PIN_BPF1,LOW);
        digitalWrite(PIN_BPF2,LOW);
        digitalWrite(PIN_BPF3,LOW);
        break;
      }
    }
  }
}

static void init_i2s(void)
{
  i2s.setDATA(PIN_DOUT);
  i2s.setBCLK(PIN_BCLK); // Note: LRCLK = BCLK + 1
  i2s.setMCLK(PIN_MCLK);
  i2s.setBitsPerSample(32);
  i2s.setFrequency(SAMPLERATE);
  i2s.setMCLKmult(256);
  i2s.setBuffers(4, 256, 0);
  i2s.begin();
}

static void set_frequency(void)
{
  static const int32_t ssboffset = SAMPLERATE/4;
  static const int32_t cwoffset = ssboffset + CW_TONE;
  int32_t offset = 0;
  switch (radio.mode)
  {
    case MODE_LSB: offset = +ssboffset; break;
    case MODE_USB: offset = -ssboffset; break;
    case MODE_AM:  offset = +ssboffset; break;
    case MODE_CWL: offset = +cwoffset;  break;
    case MODE_CWU: offset = -cwoffset;  break;
  }
  const uint64_t f = SI5351_FREQ_MULT * ((radio.frequency+offset) * 4ull + 2ull);
#ifndef DEBUGGING_SKIP
  SI5351.set_freq(f,SI5351_CLK0);
#endif
}

static radio_mode_t get_mode(void)
{
  if (digitalRead(PIN_MODE1)==LOW) return MODE_LSB;
  if (digitalRead(PIN_MODE2)==LOW) return MODE_USB;
  if (digitalRead(PIN_MODE3)==LOW) return MODE_AM;
  if (digitalRead(PIN_MODE4)==LOW) return MODE_CWL;
  if (digitalRead(PIN_MODE5)==LOW) return MODE_CWU;
  if (digitalRead(PIN_MODE6)==LOW) return MODE_SWL;
  return MODE_AM;
}

static uint8_t get_band(void)
{
  if (digitalRead(PIN_BAND1)==LOW) return 0u;
  if (digitalRead(PIN_BAND2)==LOW) return 1u;
  if (digitalRead(PIN_BAND3)==LOW) return 2u;
  if (digitalRead(PIN_BAND4)==LOW) return 3u;
  if (digitalRead(PIN_BAND5)==LOW) return 4u;
  if (digitalRead(PIN_BAND6)==LOW) return 5u;
  return 0;
}

static uint8_t get_filter(void)
{
  if (digitalRead(PIN_FILTER1)==LOW) return 0u;
  if (digitalRead(PIN_FILTER2)==LOW) return 1u;
  if (digitalRead(PIN_FILTER3)==LOW) return 2u;
  if (digitalRead(PIN_FILTER4)==LOW) return 3u;
  return 0;
}

static uint8_t get_jnr(void)
{
  if (digitalRead(PIN_JNR1)==LOW) return 1u;
  if (digitalRead(PIN_JNR2)==LOW) return 2u;
  if (digitalRead(PIN_JNR3)==LOW) return 3u;
  return 0;
}


void setup(void)
{
  // run DSP on core 0
  pinMode(PIN_0,INPUT_PULLUP);
  pinMode(PIN_1,INPUT_PULLUP);
  pinMode(PIN_AUPWML,OUTPUT);
  pinMode(PIN_AUPWMH,OUTPUT);
  pinMode(PIN_DOUT,INPUT);
  pinMode(PIN_BCLK,OUTPUT);
  pinMode(PIN_LRCL,OUTPUT);
  pinMode(PIN_MCLK,OUTPUT);
  pinMode(PIN_8,INPUT_PULLUP);
  pinMode(PIN_9,INPUT_PULLUP);
  pinMode(PIN_10,INPUT_PULLUP);
  pinMode(PIN_11,INPUT_PULLUP);
  pinMode(PIN_BPF0,OUTPUT);
  pinMode(PIN_BPF1,OUTPUT);
  pinMode(PIN_BPF2,OUTPUT);
  pinMode(PIN_BPF3,OUTPUT);
  pinMode(PIN_SDA,INPUT_PULLUP);
  pinMode(PIN_SCL,INPUT_PULLUP);
  pinMode(PIN_18,INPUT_PULLUP);
  pinMode(PIN_19,INPUT_PULLUP);
  pinMode(PIN_BAND2,INPUT_PULLUP);
  pinMode(PIN_21,INPUT_PULLUP);
  pinMode(PIN_TUNE,OUTPUT);
  pinMode(PIN_SMETER,OUTPUT);
  pinMode(PIN_BAND1,INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_BAND3,INPUT_PULLUP);
  pinMode(PIN_BAND4,INPUT_PULLUP);
  pinMode(PIN_BAND5,INPUT_PULLUP);
  pinMode(PIN_BAND6,INPUT_PULLUP);
  pinMode(PIN_FILTER1,INPUT_PULLUP);
  pinMode(PIN_FILTER2,INPUT_PULLUP);
  pinMode(PIN_FILTER3,INPUT_PULLUP);
  pinMode(PIN_FILTER4,INPUT_PULLUP);
  pinMode(PIN_MODE1,INPUT_PULLUP);
  pinMode(PIN_MODE2,INPUT_PULLUP);
  pinMode(PIN_MODE3,INPUT_PULLUP);
  pinMode(PIN_MODE4,INPUT_PULLUP);
  pinMode(PIN_MODE5,INPUT_PULLUP);
  pinMode(PIN_MODE6,INPUT_PULLUP);
  pinMode(PIN_JNR1,INPUT_PULLUP);
  pinMode(PIN_JNR2,INPUT_PULLUP);
  pinMode(PIN_JNR3,INPUT_PULLUP);
  pinMode(PIN_43,INPUT_PULLUP);
  pinMode(PIN_FINEA,INPUT_PULLUP);
  pinMode(PIN_FINEB,INPUT_PULLUP);
  pinMode(PIN_TUNEA,INPUT_PULLUP);
  pinMode(PIN_TUNEB,INPUT_PULLUP);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  const uint32_t clksys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  // frequency_count_khz() isn't accurate!
  if (clksys < (240000ul - 5ul) || clksys > (240000ul + 5ul))
  {
    // trap the wrong system clock
    error_stop(ERROR_SYSCLOCK);
  }

  // meters
  analogWriteFreq(200000ul);
  analogWriteResolution(10);
  analogWrite(PIN_TUNE,0);
  analogWrite(PIN_SMETER,0);

  // high speed I2C
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.setClock(400000ul);
  Wire.begin();

  // set up audio out PWM
  gpio_set_function(PIN_AUPWMH,GPIO_FUNC_PWM);
  gpio_set_function(PIN_AUPWML,GPIO_FUNC_PWM);
  audio_pwm = pwm_gpio_to_slice_num(PIN_AUPWML);
  pwm_set_wrap(audio_pwm,63); // 240,000,000 / 64 = 3,750,000
  pwm_set_both_levels(audio_pwm,0,31);
  pwm_set_enabled(audio_pwm,true);

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  // get current settings
  radio.band = get_band();
  radio.mode = get_mode();
  radio.filter = get_filter();
  radio.jnr = get_jnr();
  radio.frequency = saved_frequency[radio.band];
  radio.frequency = constrain(radio.frequency,bands[radio.band].lo,bands[radio.band].hi);

  // init PLL and set default frequency
#ifndef DEBUGGING_SKIP
  const bool si5351_found = SI5351.init(SI5351_CRYSTAL_LOAD_0PF,TCXO_FREQ,0);
  if (!si5351_found)
  {
    error_stop(ERROR_SI5351);
  }
  SI5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
#endif
  set_frequency();
  set_filter();

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  // init rotary
  t.begin();
  f.begin();

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  // set up audio ADC (IQ input)
  init_i2s();

  // setup time for busy_wait_until()
  dsp_process_next = make_timeout_time_us(32ull);
  setup_complete = true;
}

void setup1(void)
{
  // run UI on core 1
  // only go to loop1 when setup() has completed
  while (!setup_complete)
  {
    tight_loop_contents();
  }
#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<5;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif
}

void __not_in_flash_func(loop)(void)
{
  // run DSP on core 0
  volatile static int32_t dac_h = 0;
  volatile static int32_t dac_l = 0;
  busy_wait_until(dsp_process_next);
  dsp_process_next = delayed_by_us(dsp_process_next,32ull);
  if (!i2s.available())
  {
    return;
  }
  pwm_set_both_levels(audio_pwm,dac_l,dac_h);
  int32_t ii = 0;
  int32_t qq = 0;
  i2s.read32(&ii, &qq);
  ii >>= 16;
  qq >>= 16;
  int32_t dac_audio = 0;
  const uint8_t jnr = radio.jnr;
  const uint8_t bw = radio.filter;
  switch (radio.mode)
  {
    case MODE_LSB: dac_audio = DSP::process_ssb(ii,qq,jnr,bw); break;
    case MODE_USB: dac_audio = DSP::process_ssb(qq,ii,jnr,bw); break;
    case MODE_CWL: dac_audio = DSP::process_cw(ii,qq);         break;
    case MODE_CWU: dac_audio = DSP::process_cw(qq,ii);         break;
    case MODE_AM:  dac_audio = DSP::process_am(ii,qq,jnr,bw);  break;
    case MODE_SWL: dac_audio = DSP::process_swl(jnr,bw);       break;
  }
  dac_audio = constrain(dac_audio,-2048l,+2047l);
  dac_audio += 2048l;
  dac_h = dac_audio >> 6;
  dac_l = dac_audio & 0x3f;

  // process tune and fine tune
  static int32_t rotary_tune = 0l;
  static int32_t rotary_fine = 0l;
  switch (t.process())
  {
    case DIR_CW:  rotary_tune++; break;
    case DIR_CCW: rotary_tune--; break;
  }
  switch (f.process())
  {
    case DIR_CW:  rotary_fine++; break;
    case DIR_CCW: rotary_fine--; break;
  }
  if (rotary_tune!=0 || rotary_fine!=0)
  {
    // don't hang around if we can't own the mutex immediately
    // rotary_* will record any rotations
    if (mutex_try_enter(&rotary_mutex,0ul))
    {
      radio.tune += rotary_tune;
      radio.fine += rotary_fine;
      rotary_tune = 0;
      rotary_fine = 0;
      mutex_exit(&rotary_mutex);
    }
  }
}

void loop1(void)
{
  // run UI on core 1
  //
  // band 1 - 6
  // JNR (off 1 - 3)
  // mode: (USB, LSB, AM, CWU, CWL)
  // filter (..., 1500, 2000, 3000, 4000)
  // attenuation (0, 6, 12, 18)
  volatile static uint32_t old_frequency= radio.frequency;
  volatile static radio_mode_t old_mode_select = radio.mode;
  volatile static uint8_t old_filter_select = radio.filter;
  volatile static uint8_t old_band_select = radio.band;
  volatile static uint8_t old_jnr_select = radio.jnr;
  const radio_mode_t mode_select = get_mode();
  const uint8_t filter_select = get_filter();
  const uint8_t band_select = get_band();
  const uint8_t jnr_select = get_jnr();
  if (old_band_select != band_select)
  {
    saved_frequency[old_band_select] = radio.frequency;
    radio.frequency = saved_frequency[band_select];
    radio.band = band_select;
    old_band_select = band_select;
    // mute during change
    DSP::mute();
    delay(250);
    set_filter();
    set_frequency();
    delay(250);
  }
  if (old_filter_select != filter_select)
  {
    old_filter_select = filter_select;
    radio.filter = filter_select;
  }
  if (old_mode_select != mode_select)
  {
    old_mode_select = mode_select;
    radio.mode = mode_select;
    set_frequency();
  }
  if (old_jnr_select != jnr_select)
  {
    old_jnr_select = jnr_select;
    radio.jnr = jnr_select;
  }

  // process main tuning
  mutex_enter_blocking(&rotary_mutex);
  volatile const int32_t tune_delta = radio.tune;
  volatile const int32_t fine_delta = radio.fine;
  radio.tune = 0;
  radio.fine = 0;
  mutex_exit(&rotary_mutex);
  uint32_t new_frequency = radio.frequency;
  if (tune_delta != 0)
  {
    // fast tune when in SWL mode
    const uint32_t tune_step = radio.mode==MODE_SWL?FAST_TUNE_STEP:MAIN_TUNE_STEP;
    new_frequency = new_frequency + (tune_delta * tune_step);
    new_frequency = new_frequency / tune_step;
    new_frequency = new_frequency * tune_step;
  }
  if (fine_delta != 0)
  {
    // fast fine tune when in SWL mode
    const uint32_t tune_step = radio.mode==MODE_SWL?FAST_FINE_STEP:FINE_TUNE_STEP;
    new_frequency = new_frequency + (fine_delta * tune_step);
    new_frequency = new_frequency / tune_step;
    new_frequency = new_frequency * tune_step;
  }
  new_frequency = constrain(new_frequency,bands[radio.band].lo,bands[radio.band].hi);
  if (new_frequency!=old_frequency)
  {
    radio.frequency = new_frequency;
    old_frequency = new_frequency;
    set_filter();
    set_frequency();
  }

  // adjust meters every 50ms
  static uint32_t next_update = millis();
  const uint32_t now = millis();
  if (now>next_update)
  {
    next_update += 50ul;

    // s-meter
    analogWrite(PIN_SMETER,DSP::smeter(radio.frequency));

    // tune meter
    uint64_t tune_value = 1023ull;
    if (radio.frequency!=bands[radio.band].hi)
    {
      tune_value = (uint64_t)(radio.frequency%5000000) * 1023ull / 5000000ull;
    }
    analogWrite(PIN_TUNE,tune_value);
  }
  static uint32_t heartbeat = millis();
  static bool toggle_led = false;
  if (now>heartbeat)
  {
    heartbeat += 500;
    digitalWrite(LED_BUILTIN,toggle_led?HIGH:LOW);
    toggle_led = !toggle_led;
  }
}