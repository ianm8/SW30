#ifndef DSP_H
#define DSP_H

#include "filter.h"
#include "util.h"
#include "BBCWS-20250621-ALaw.h"

namespace DSP
{
  volatile static float agc_peak = 0.0f;

  static void __not_in_flash_func(mute)(void)
  {
    // set AGC to high value so that audio is temporarily muted
    static const float mute_value = 8192.0f;
    agc_peak = mute_value;
  }

  static const int16_t __not_in_flash_func(agc)(const float in)
  {
    // limit gain to max of 40 (32db)
    static const float max_gain = 40.0f;
    // decay about 10dB per second
    static const float k = 0.99996f;

    const float magnitude = fabsf(in);
    if (magnitude>agc_peak)
    {
      agc_peak = magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // trap issues with low values
    if (agc_peak<1.0f) return (int16_t)(in * max_gain);

    // set maximum gain possible for 12 bit DAC
    const float m = 2047.0f/agc_peak;
    return (int16_t)(in*fminf(m,max_gain));
  }

  static const uint16_t __not_in_flash_func(smeter)(const uint32_t f)
  {
    // S9 = -73dBm = 141uV PP
    // need to return 512 for S9
    const bool hiband = (f > 15000000ul);
    if (hiband)
    {
      static const float hiadjust = 4.0f;
      static const float S0_sig = 100.0f / hiadjust;
      static const float S9_sig = 300.0f / hiadjust;
      static const float S9p_sig = 8192.0f/ hiadjust;
      static const uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
      static const uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
      static const uint32_t S9_min = 0ul;
      static const uint32_t S9_max = 639ul;
      static const uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
      static const uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
      static const uint32_t S9p_min = 640ul;
      static const uint32_t S9p_max = 1023ul;
      if (agc_peak<1.0f)
      {
        return 0u;
      }
      const uint32_t log_peak = (uint32_t)(log10f(agc_peak) * 1024.0f);
      if (agc_peak>S9_sig)
      {
        return (uint8_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
      }
      return (uint8_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
    }
    static const float S0_sig = 100.0f;
    static const float S9_sig = 200.0f;
    static const float S9p_sig = 8192.0f;
    static const uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
    static const uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static const uint32_t S9_min = 0ul;
    static const uint32_t S9_max = 639ul;
    static const uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static const uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
    static const uint32_t S9p_min = 640ul;
    static const uint32_t S9p_max = 1023ul;
    if (agc_peak<1.0f)
    {
      return 0u;
    }
    const uint32_t log_peak = (uint32_t)(log10f(agc_peak) * 1024.0f);
    if (agc_peak>S9_sig)
    {
      return (uint16_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
    }
    return (uint16_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
  }

  static const int16_t __not_in_flash_func(process_ssb)(const int16_t in_i,const int16_t in_q,const uint32_t jnr_level,const uint8_t bw)
  {
    // 2 bit quadrature local oscillator
    volatile static struct { uint32_t lo : 2; } quad = { 0 };

    // remove DC and half of image
    const float ii = FILTER::hpf_fs4f_i((float)in_i / 32768.0f);
    const float qq = FILTER::hpf_fs4f_q((float)in_q / 32768.0f);

    // quadrature down-convert from FS/4
    const float iq[] = { qq, ii, -qq, -ii };
    const float ssb = iq[quad.lo++];

    // LPF
    const float audio_lpf = FILTER::bwf[bw](ssb);

    // HPF
    const float audio_raw = FILTER::hpf_200f(audio_lpf);

    // JNR
    const float audio_out = FILTER::jnr(audio_raw,jnr_level);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_cw)(const int16_t in_i,const int16_t in_q)
  {
    // 2 bit quadrature local oscillator
    volatile static struct { uint32_t lo : 2; } quad = { 0 };

    // remove DC and half of image
    const float ii = FILTER::hpf_fs4f_i((float)in_i / 32768.0f);
    const float qq = FILTER::hpf_fs4f_q((float)in_q / 32768.0f);

    // quadrature down-convert from FS/4
    const float iq[] = { qq, ii, -qq, -ii };
    const float ssb = iq[quad.lo++];

    // BPF for CW
    const float audio_out = FILTER::bpf_700f(ssb);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_am)(const int16_t in_i,const int16_t in_q,const uint32_t jnr_level,const uint8_t bw)
  {
    // BPF I with +45 phase shift
    // BPF Q with -45 phase shift
    // SUM (USB)
    // ABS
    // AGC
    // LPF
    // Remove DC

    // AGC settings
    static const float max_gain = 40.0f;
    static const float k = 0.99996f;

    // extract AM signal from USB @ FS/4
    const float ii = FILTER::bpf_45p((float)in_i / 32768.0f);
    const float qq = FILTER::bpf_45n((float)in_q / 32768.0f);
    const float ssb = ii + qq;
    const float magnitude = fabsf(ssb);
    const float rectified = ssb * ssb;

    // AGC on the carrier (not the audio)
    const float agc_magnitude = magnitude * 32768.0f;
    if (agc_magnitude > agc_peak)
    {
      agc_peak = agc_magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // set maximum gain possible for 12 bit DAC
    float gain = max_gain;
    if (agc_peak > 1.0f)
    {
      gain = 2047.0f / agc_peak;
    }
    gain = fminf(gain, max_gain);

    // extract audio from rectified AM signal
    const float audio_raw = FILTER::dcf(sqrtf(fabsf(FILTER::bwf[bw](rectified))));
    const float audio_out = FILTER::jnr(audio_raw,jnr_level);
    return (int16_t)(audio_out * 32768.0f * gain);
  }

  static const int16_t __not_in_flash_func(process_swl)(const uint32_t jnr_level,const uint8_t bw)
  {
    // grab the SWL file data
    // process A-law to linear
    // up-sample from 7812 to 31250
    // filter (interpolate)
    // noise reduction
    static const uint32_t data_length = sizeof(SWL_DATA);
    volatile static struct {uint32_t c : 2; } upsample = { 0 };
    volatile static uint32_t p = 0;
    float audio_raw = 0.0f;
    if (upsample.c==0)
    {
      audio_raw = (float)UTIL::ALaw2Linear(SWL_DATA[p++]) / 32768.0f;
      if (p>=data_length) p = 0;
    }
    upsample.c++;
    const float audio_lpf = FILTER::bwf[bw](audio_raw);
    const float audio_out = FILTER::jnr(audio_lpf,jnr_level);
    return agc(audio_out * 32768.0f);
  }
}
#endif