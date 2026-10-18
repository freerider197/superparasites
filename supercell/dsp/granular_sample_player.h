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
// Granular playback of audio stored in a buffer.

#ifndef CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_
#define CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_

#include "stmlib/stmlib.h"

#include <algorithm>
#include <cmath>

#include "stmlib/dsp/atan.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "supercell/dsp/audio_buffer.h"
#include "supercell/dsp/frame.h"
#include "supercell/dsp/grain.h"
#include "supercell/dsp/parameters.h"

#include "supercell/resources.h"

namespace clouds {

const int32_t kMaxNumGrains = 40;

// Clock periods accepted on the TRIG input, in samples. Slower than the max
// (or no clock at all) falls back to free-running grains.
const int32_t kMinClockPeriod = 64;
const int32_t kMaxClockPeriod = 65536;

// With a clock, SIZE steps through these fractions of the clock period and
// POSITION steps through slices aligned to the clock ticks in the recording.
const float kSyncedSizes[] = {
  1.0f / 32.0f, 1.0f / 16.0f, 1.0f / 8.0f, 1.0f / 4.0f, 1.0f / 2.0f, 1.0f, 2.0f
};
const int32_t kNumSyncedSizes = sizeof(kSyncedSizes) / sizeof(kSyncedSizes[0]);
const float kMinSyncedGrainSize = 128.0f;

using namespace stmlib;

class GranularSamplePlayer {
 public:
  GranularSamplePlayer() { }
  ~GranularSamplePlayer() { }
  
  void Init(int32_t num_channels, int32_t max_num_grains) {
    max_num_grains_ = max_num_grains;
    num_midfi_grains_ = 3 * max_num_grains / 4;
    gain_normalization_ = 1.0f;
    for (int32_t i = 0; i < kMaxNumGrains; ++i) {
      grains_[i].Init();
    }
    num_grains_ = 0.0f;
    num_channels_ = num_channels;
    grain_size_hint_ = 1024.0f;
    grain_rate_phasor_ = 0.0f;
    samples_since_clock_ = kMaxClockPeriod + 1;
    clock_period_ = 0;
    ratchets_left_ = 0;
    recorded_since_clock_ = 0;
  }
  
  template<Resolution resolution>
  void Play(
      const AudioBuffer<resolution>* buffer,
      const Parameters& parameters,
      float* out, size_t size) {
    float overlap = parameters.granular.overlap;
    overlap = (overlap * overlap) * (overlap * overlap);
    float target_num_grains = max_num_grains_ * overlap;
    float space_between_grains = grain_size_hint_ / target_num_grains;

    // Every TRIG measures the clock period and restarts the grain rhythm.
    bool seed_trigger = parameters.capture;
    int32_t clock_ratio = parameters.granular.clock_ratio;
    if (seed_trigger) {
      clock_period_ = samples_since_clock_ >= kMinClockPeriod &&
          samples_since_clock_ <= kMaxClockPeriod ? samples_since_clock_ : 0;
      samples_since_clock_ = 0;
      grain_rate_phasor_ = 0.0f;
      // While frozen, keep the grid of the frozen recording.
      if (!parameters.freeze) {
        recorded_since_clock_ = 0;
      }
      ratchets_left_ = clock_ratio - 1;
    }

    // Clock-locked mode: clock_ratio grains evenly spaced within each clock
    // period. Without a clock, grains run free.
    bool clock_locked = clock_ratio > 0 && clock_period_ > 0;
    float ratchet_spacing = clock_locked
        ? static_cast<float>(clock_period_) / static_cast<float>(clock_ratio)
        : 0.0f;
    
    // Build a list of available grains.
    int32_t num_available_grains = FillAvailableGrainsList();
    
    // Try to schedule new grains.
    for (size_t t = 0; t < size; ++t) {
      grain_rate_phasor_ += 1.0f;
      bool seed;
      if (clock_locked) {
        seed = ratchets_left_ > 0 && grain_rate_phasor_ >= ratchet_spacing;
        if (seed) {
          // A ratchet with no free grain is dropped to stay on the grid.
          grain_rate_phasor_ -= ratchet_spacing;
          --ratchets_left_;
        }
      } else {
        seed = grain_rate_phasor_ >= space_between_grains;
      }
      seed = seed || seed_trigger;
      if (num_available_grains && seed) {
        --num_available_grains;
        int32_t index = available_grains_[num_available_grains];
        GrainQuality quality;
        if (num_available_grains < num_midfi_grains_) {
          quality = GRAIN_QUALITY_MEDIUM;
        } else {
          quality = GRAIN_QUALITY_HIGH;
        }
        
        Grain* g = &grains_[index];
        ScheduleGrain(
            g,
            parameters,
            t,
            buffer->size(),
            buffer->head() - size + t,
            quality);
        if (!clock_locked) {
          grain_rate_phasor_ = 0.0f;
        }
        seed_trigger = false;
      }
    }

    samples_since_clock_ += size;
    if (!parameters.freeze && recorded_since_clock_ < (1 << 24)) {
      recorded_since_clock_ += size;
    }
    if (samples_since_clock_ > kMaxClockPeriod) {
      samples_since_clock_ = kMaxClockPeriod + 1;
      clock_period_ = 0;
    }
    
    // Overlap grains.
    std::fill(&out[0], &out[size * 2], 0.0f);
    float* e = envelope_buffer_;
    for (int32_t i = 0; i < max_num_grains_; ++i) {
      Grain* g = &grains_[i];
      if (g->recommended_quality() == GRAIN_QUALITY_HIGH) {
        if (num_channels_ == 1) {
          g->OverlapAdd<1, GRAIN_QUALITY_HIGH>(buffer, out, e, size);
        } else {
          g->OverlapAdd<2, GRAIN_QUALITY_HIGH>(buffer, out, e, size);
        }
      } else if (g->recommended_quality() == GRAIN_QUALITY_MEDIUM) {
        if (num_channels_ == 1) {
          g->OverlapAdd<1, GRAIN_QUALITY_MEDIUM>(buffer, out, e, size);
        } else {
          g->OverlapAdd<2, GRAIN_QUALITY_MEDIUM>(buffer, out, e, size);
        }
      } else {
        if (num_channels_ == 1) {
          g->OverlapAdd<1, GRAIN_QUALITY_LOW>(buffer, out, e, size);
        } else {
          g->OverlapAdd<2, GRAIN_QUALITY_LOW>(buffer, out, e, size);
        }
      }
    }
    
    // Compute normalization factor.
    int32_t active_grains = max_num_grains_ - num_available_grains;
    SLOPE(num_grains_, static_cast<float>(active_grains), 0.9f, 0.2f);

    float gain_normalization = num_grains_ > 2.0f
        ? fast_rsqrt_carmack(num_grains_ - 1.0f)
        : 1.0f;  
    float window_gain = 1.0f + 2.0f * parameters.granular.window_shape;
    CONSTRAIN(window_gain, 1.0f, 2.0f);
    gain_normalization *= Crossfade(
        1.0f, window_gain, parameters.granular.overlap);

    // Apply gain normalization.
    for (size_t t = 0; t < size; ++t) {
      ONE_POLE(gain_normalization_, gain_normalization, 0.01f)
      *out++ *= gain_normalization_;
      *out++ *= gain_normalization_;
    }
  }
  
 private:
  int32_t FillAvailableGrainsList() {
    int32_t num_available_grains = 0;
    for (int32_t i = 0; i < max_num_grains_; ++i) {
      if (!grains_[i].active()) {
        available_grains_[num_available_grains] = i;
        ++num_available_grains;
      }
    }
    return num_available_grains;
  }
  
  void ScheduleGrain(
      Grain* grain,
      const Parameters& parameters,
      int32_t pre_delay,
      int32_t buffer_size,
      int32_t buffer_head,
      GrainQuality quality) {
    float position = parameters.position;
    float pitch = parameters.pitch;
    float window_shape = parameters.granular.window_shape;
    float pitch_ratio = SemitonesToRatio(pitch);
    float inv_pitch_ratio = SemitonesToRatio(-pitch);
    bool synced = clock_period_ > 0;
    float grain_size;
    if (synced) {
      int32_t step = static_cast<int32_t>(parameters.size * kNumSyncedSizes);
      CONSTRAIN(step, 0, kNumSyncedSizes - 1);
      grain_size = clock_period_ * kSyncedSizes[step];
      // Halve until it fits in the buffer (with room for the play head).
      float max_size = buffer_size / (1.0f + pitch_ratio);
      while (grain_size > max_size && grain_size > kMinSyncedGrainSize) {
        grain_size *= 0.5f;
      }
      if (grain_size < kMinSyncedGrainSize) {
        grain_size = kMinSyncedGrainSize;
      }
    } else {
      grain_size = Interpolate(lut_grain_size, parameters.size, 256.0f);
    }
    float pan = 0.5f + parameters.stereo_spread * (Random::GetFloat() - 0.5f);
    float gain_l, gain_r;
    if (num_channels_ == 1) {
      gain_l = Interpolate(lut_sin, pan, 256.0f);
      gain_r = Interpolate(lut_sin + 256, pan, 256.0f);
    } else {
      if (pan < 0.5f) {
        gain_l = 1.0f;
        gain_r = 2.0f * pan;
      } else {
        gain_r = 1.0f;
        gain_l = 2.0f * (1.0f - pan);
      }
    }
    
    if (pitch_ratio > 1.0f && !synced) {
      // The grain's play-head moves faster than the buffer record-head.
      // we must make sure that the grain will not consume too much data.
      // In some situations, it might be necessary to reduce the size of the
      // grain.
      grain_size = std::min(grain_size, buffer_size * 0.25f * inv_pitch_ratio);
    }

    float eaten_by_play_head = grain_size * pitch_ratio;
    float eaten_by_recording_head = grain_size;

    float available = 0.0;
    available += static_cast<float>(buffer_size);
    available -= eaten_by_play_head;
    available -= eaten_by_recording_head;

    bool reverse = parameters.granular.reverse;
    int32_t size = static_cast<int32_t>(grain_size) & ~1;
    float delay = position * available + eaten_by_play_head;
    if (synced) {
      // Candidate start points sit at whole slices before/after the last
      // clock tick: delay = phase + j * slice. POSITION picks j.
      float slice = grain_size;
      float since_tick = static_cast<float>(recorded_since_clock_) +
          (parameters.freeze ? 0.0f : static_cast<float>(pre_delay));
      float phase = since_tick - floorf(since_tick / slice) * slice;
      float j_min = ceilf((eaten_by_play_head - phase) / slice);
      float j_max = floorf((buffer_size - grain_size - phase) / slice);
      if (j_min < 0.0f) j_min = 0.0f;
      if (j_max >= j_min) {
        float j = j_min + floorf(position * (j_max - j_min) + 0.5f);
        delay = phase + j * slice;
      }
    }
    int32_t start = buffer_head - static_cast<int32_t>(delay);
    grain->Start(
        pre_delay,
        buffer_size,
        start,
        size,
        reverse,
        static_cast<uint32_t>(pitch_ratio * 65536.0f),
        window_shape,
        gain_l,
        gain_r,
        quality);
    grain_size_hint_ = grain_size;
  }
  
  int32_t max_num_grains_;
  int32_t num_midfi_grains_;
  int32_t num_channels_;

  float num_grains_;
  float gain_normalization_;
  float grain_size_hint_;
  float grain_rate_phasor_;

  int32_t samples_since_clock_;
  int32_t clock_period_;
  int32_t ratchets_left_;
  int32_t recorded_since_clock_;
  
  Grain grains_[kMaxNumGrains];
  int32_t available_grains_[kMaxNumGrains];
  float envelope_buffer_[kMaxBlockSize];
  
  DISALLOW_COPY_AND_ASSIGN(GranularSamplePlayer);
};

}  // namespace clouds

#endif  // CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_
