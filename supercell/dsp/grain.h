// Copyright 2014 Olivier Gillet.
//
// Author: Olivier Gillet (ol.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Single grain synthesis.

#ifndef CLOUDS_DSP_GRAIN_H_
#define CLOUDS_DSP_GRAIN_H_

#include "stmlib/stmlib.h"

#include "stmlib/dsp/dsp.h"

#include "supercell/dsp/audio_buffer.h"

#include "supercell/resources.h"

namespace clouds {

// Shortest fade-in/fade-out applied to any grain, in samples. Anything
// shorter than ~2ms is heard as a click.
const float kMinGrainFadeSamples = 64.0f;

enum GrainQuality {
  GRAIN_QUALITY_LOW,
  GRAIN_QUALITY_MEDIUM,
  GRAIN_QUALITY_HIGH
};

class Grain {
 public:
  Grain() { }
  ~Grain() { }

  void Init() {
    active_ = false;
    envelope_phase_ = 1.0f;
  }

  void Start(
      int32_t pre_delay,
      int32_t buffer_size,
      int32_t start,
      int32_t width,
      bool reverse,
      int32_t phase_increment,
      float window_shape,
      float gain_l,
      float gain_r,
      GrainQuality recommended_quality) {
    pre_delay_ = pre_delay;
    reverse_ = reverse;

    first_sample_ = (start + buffer_size) % buffer_size;
    if (reverse) {
      phase_increment_ = -phase_increment;
      phase_ = width * phase_increment;
    } else {
      phase_increment_ = phase_increment;
      phase_ = 0;
    }
    envelope_phase_ = 0.0f;
    envelope_phase_increment_ = 1.0f / static_cast<float>(width);

    // Window shape morphs through:
    //   0.0 flat top with short smooth fades
    //   0.5 smooth bell (Hann-like)
    //   1.0 percussive: fast smooth attack, curved decay
    // Fades never get shorter than kMinGrainFadeSamples.
    float attack, release;
    if (window_shape < 0.5f) {
      attack = release = 0.05f + 0.9f * window_shape;
      envelope_curve_ = 0.0f;
    } else {
      float k = 2.0f * window_shape - 1.0f;
      attack = 0.5f * (1.0f - k);
      release = 1.0f - attack;
      envelope_curve_ = k;
    }
    float min_fade = kMinGrainFadeSamples * envelope_phase_increment_;
    if (min_fade > 0.5f) min_fade = 0.5f;
    if (attack < min_fade) attack = min_fade;
    if (release < min_fade) release = min_fade;
    if (attack + release > 1.0f) release = 1.0f - attack;
    envelope_attack_end_ = attack;
    envelope_attack_scale_ = 1.0f / attack;
    envelope_release_start_ = 1.0f - release;
    envelope_release_scale_ = 1.0f / release;

    active_ = true;
    gain_l_ = gain_l;
    gain_r_ = gain_r;
    recommended_quality_ = recommended_quality;
  }
  
  inline void RenderEnvelope(float* destination, size_t size) {
    const float increment = envelope_phase_increment_;
    const float attack_end = envelope_attack_end_;
    const float attack_scale = envelope_attack_scale_;
    const float release_start = envelope_release_start_;
    const float release_scale = envelope_release_scale_;
    const float curve = envelope_curve_;

    float phase = envelope_phase_;
    while (size--) {
      // Smoothstep fades: zero slope at both ends, so no clicks.
      float gain = 1.0f;
      if (phase < attack_end) {
        float u = phase * attack_scale;
        gain = u * u * (3.0f - 2.0f * u);
      } else if (phase > release_start) {
        float t = (1.0f - phase) * release_scale;
        float smooth = t * t * (3.0f - 2.0f * t);
        gain = smooth + curve * (t * t * t - smooth);
      }
      phase += increment;
      if (phase >= 1.0f) {
        *destination = -1.0f;
        break;
      }
      *destination++ = gain;
    }
    envelope_phase_ = phase;
  }
  
  template<int32_t num_channels, GrainQuality quality, Resolution resolution>
  inline void OverlapAdd(
      const AudioBuffer<resolution>* buffer,
      float* destination,
      float* envelope,
      size_t size) {
    if (!active_) {
      return;
    }
    // Rendering is done on 32-sample long blocks. The pre-delay allows grains
    // to start at arbitrary samples within a block, rather than at block
    // boundaries.
    while (pre_delay_ && size) {
      destination += 2;
      --size;
      --pre_delay_;
    }
    
    // Pre-render the envelope in one pass.
    RenderEnvelope(envelope, size);

    const int32_t phase_increment = phase_increment_;
    const int32_t first_sample = first_sample_;
    const float gain_l = gain_l_;
    const float gain_r = gain_r_;
    int32_t phase = phase_;
    while (size--) {
      int32_t sample_index = first_sample + (phase >> 16);

      float gain = *envelope++;
      if (gain == -1.0f) {
        active_ = false;
        break;
      }

      float l = buffer[0].template Read<InterpolationMethod(quality)>(
          sample_index, phase & 65535) * gain;
      if (num_channels == 1) {
        *destination++ += l * gain_l;
        *destination++ += l * gain_r;
      } else if (num_channels == 2) {
        float r = buffer[1].template Read<InterpolationMethod(quality)>(
            sample_index, phase & 65535) * gain;
        *destination++ += l * gain_l + r * (1.0f - gain_r);
        *destination++ += r * gain_r + l * (1.0f - gain_l);
      }
      phase += phase_increment;
    }
    phase_ = phase;
  }
  
  inline bool active() { return active_; }
  
  inline GrainQuality recommended_quality() const {
    return recommended_quality_;
  }

 private:
  int32_t first_sample_;
  int32_t phase_;
  int32_t phase_increment_;
  int32_t pre_delay_;

  float envelope_attack_end_;
  float envelope_attack_scale_;
  float envelope_release_start_;
  float envelope_release_scale_;
  float envelope_curve_;
  float envelope_phase_;
  float envelope_phase_increment_;

  float gain_l_;
  float gain_r_;

  bool active_;
  bool reverse_;
  
  GrainQuality recommended_quality_;

  DISALLOW_COPY_AND_ASSIGN(Grain);
};

}  // namespace clouds

#endif  // CLOUDS_DSP_GRAIN_H_
