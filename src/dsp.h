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
    // need to return 10 for S9
    static constexpr float GAIN_3dB = 1.5f;
    static constexpr float GAIN_6dB = 2.0f;
    static constexpr float GAIN_9dB = 3.0f;
    static constexpr float GAIN_12dB = 4.0f;
    static constexpr float GAIN_15dB = 6.0f;
    static constexpr float GAIN_18dB = 8.0f;
    float sensitivity = 1.0f;
    if (f > 22000000ul)
    {
      sensitivity = GAIN_18dB;
    }
    else if (f > 18000000ul)
    {
      sensitivity = GAIN_12dB;
    }
    else if (f > 10000000ul)
    {
      sensitivity = GAIN_6dB;
    }
    // add compensation gain to agc_peak
    const float comp_agc_peak = agc_peak * sensitivity;
    if (comp_agc_peak<1.0f)
    {
      return 0u;
    }
    static constexpr const float S0_sig = 100.0f;
    static constexpr const float S9_sig = 200.0f;
    static constexpr const float S9p_sig = 8192.0f;
    static constexpr const uint32_t S9_from_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
    static constexpr const uint32_t S9_from_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static constexpr const uint32_t S9_min = 0ul;
    static constexpr const uint32_t S9_max = 639ul;
    static constexpr const uint32_t S9p_from_min = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static constexpr const uint32_t S9p_from_max = (uint32_t)(log10f(S9p_sig) * 1024.0f);
    static constexpr const uint32_t S9p_min = 640ul;
    static constexpr const uint32_t S9p_max = 1023ul;
    const uint32_t log_peak = (uint32_t)(log10f(comp_agc_peak) * 1024.0f);
    if (comp_agc_peak>S9_sig)
    {
      return (uint16_t)UTIL::map(log_peak,S9p_from_min,S9p_from_max,S9p_min,S9p_max);
    }
    return (uint16_t)UTIL::map(log_peak,S9_from_min,S9_from_max,S9_min,S9_max);
  }

  static void __not_in_flash_func(noise_blanker)(float &I, float &Q, const uint8_t level)
  {
    static const uint32_t MAX_BLANK_RUN = 32; // ~1ms at 31250 Hz
    static const float alpha = 0.002f;        // ~16ms at 31250 Hz

    // threasholds, number of times greater than
    // average to trigger a blanking event
    static const float thresholds[] =
    {
      0.0f,  // level 0 — unused (returns early)
      9.0f,  // level 1 — only very large spikes
      7.0f,  // level 2 — conservative
      5.0f,  // level 3 — moderate
      3.0f,  // level 4 — fairly aggressive
      2.0f   // level 5 — maximum blanking
    };
    if (level == 0) return;
    if (level > 5) return;
    const float threshold = thresholds[level];

    static struct
    {
      float lastI;
      float lastQ;
      float avg_amp;
      uint32_t blank_run;
      uint32_t initialized;
    } s = {0};

    if (s.initialized==0)
    {
      s.lastI = I;
      s.lastQ = Q;
      s.avg_amp = 0.01f;
      s.blank_run = 0;
      s.initialized = 1;
    }

    // instantaneous amplitude (fast estimate)
    const float amp = fabsf(I) + fabsf(Q);
    const bool blanked = (amp > threshold * s.avg_amp) && (s.blank_run < MAX_BLANK_RUN);
    if (blanked)
    {
      s.blank_run++;

      // hold last good sample
      I = s.lastI;
      Q = s.lastQ;
    }
    else
    {
      // track signal envelope
      s.avg_amp += alpha * (amp - s.avg_amp);
      s.blank_run = 0;
      
      // save last good sample
      s.lastI = I;
      s.lastQ = Q;
    }
  } 

  static const int16_t __not_in_flash_func(process_ssb)(const int16_t in_i,const int16_t in_q,const uint32_t jnr_level,const uint8_t bw)
  {
    // 2 bit quadrature local oscillator
    volatile static struct { uint32_t lo : 2; } quad = { 0 };

    // remove DC and half of image
    float ii = FILTER::hpf_fs4f_i((float)in_i / 32768.0f);
    float qq = FILTER::hpf_fs4f_q((float)in_q / 32768.0f);

    // set up JNR and noise blanker
    uint32_t jnr_filter = 0;
    switch (jnr_level)
    {
      case 1: jnr_filter = 1; break;
      case 3: jnr_filter = 2; break;
    }
    switch (jnr_level)
    {
      case 2: noise_blanker(ii,qq,3); break;
      case 3: noise_blanker(ii,qq,4); break;
    }

    // quadrature down-convert from FS/4
    const float iq[] = { qq, ii, -qq, -ii };
    const float ssb = iq[quad.lo++];

    // LPF
    const float audio_lpf = FILTER::bwf[bw](ssb);

    // HPF
    const float audio_raw = FILTER::hpf_200f(audio_lpf);

    // JNR
    const float audio_out = FILTER::jnr(audio_raw,jnr_filter);

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
    float ii = FILTER::bpf_45p((float)in_i / 32768.0f);
    float qq = FILTER::bpf_45n((float)in_q / 32768.0f);

    // set up JNR and noise blanker
    uint32_t jnr_filter = 0;
    switch (jnr_level)
    {
      case 1: jnr_filter = 1; break;
      case 3: jnr_filter = 2; break;
    }
    switch (jnr_level)
    {
      case 2: noise_blanker(ii,qq,3); break;
      case 3: noise_blanker(ii,qq,4); break;
    }

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
    const float audio_out = FILTER::jnr(audio_raw,jnr_filter);
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