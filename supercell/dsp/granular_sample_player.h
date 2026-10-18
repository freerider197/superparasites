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

// TRIG reaches the player kAdcLatency - 1 blocks after the edge (see
// CvScaler). The clock grid in the recording is shifted back by that much.
const int32_t kTriggerLatencyBlocks = 4;

// With a clock, grains only fire on clock ticks, never in between. DENSITY
// picks which ticks fire, going outwards from noon (noon: every tick):
//   Left of noon: straight 16-tick patterns (bit i = tick i), sparser
//   towards the end, down to one hit every 8 ticks.
//   Right of noon: Euclidean patterns, k hits spread evenly over 16 ticks.
const uint32_t kPatternLength = 16;
const int32_t kNumStraightPatterns = 7;
const uint16_t kStraightPatterns[kNumStraightPatterns] = {
  0x5555,  // x.x.x.x.x.x.x.x.
  0x4949,  // x..x..x.x..x..x.
  0x1249,  // x..x..x..x..x...
  0x1111,  // x...x...x...x...
  0x0909,  // x..x....x..x....
  0x1041,  // x.....x.....x...
  0x0101,  // x.......x.......
};
const int32_t kNumEuclideanPatterns = 6;
const int32_t kEuclideanHits[kNumEuclideanPatterns] = { 13, 11, 9, 7, 5, 3 };

// With a clock, SIZE steps through these fractions of the clock period and
// POSITION steps through slices aligned to the clock ticks in the recording.
const float kSyncedSizes[] = {
  1.0f / 4.0f, 1.0f / 2.0f, 3.0f / 4.0f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f
};
const int32_t kNumSyncedSizes = sizeof(kSyncedSizes) / sizeof(kSyncedSizes[0]);
const float kMinSyncedGrainSize = 128.0f;

// Maps value (0..1) to one of num_steps steps. Stays on the current step
// until value moves well past its edges, so pot/CV noise can't flip it.
inline int32_t QuantizeWithHysteresis(
    float value, int32_t num_steps, int32_t* state) {
  float scaled = value * static_cast<float>(num_steps);
  int32_t step = *state;
  if (step < 0 || step >= num_steps ||
      scaled < static_cast<float>(step) - 0.2f ||
      scaled > static_cast<float>(step) + 1.2f) {
    step = static_cast<int32_t>(scaled);
    CONSTRAIN(step, 0, num_steps - 1);
    *state = step;
  }
  return step;
}

// Repeatable pseudo-random value in 0..1 for hit index and lane.
inline float HitHash(uint32_t hit, uint32_t lane) {
  uint32_t x = hit * 0x9e3779b1U + lane * 0x85ebca77U + 0x165667b1U;
  x ^= x >> 15;
  x *= 0x2c1b3c6dU;
  x ^= x >> 12;
  x *= 0x297a2d39U;
  x ^= x >> 15;
  return static_cast<float>(x >> 8) / 16777216.0f;
}

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
    recorded_since_clock_ = 0;
    clock_count_ = 0;
    hit_index_ = 0;
    pattern_side_ = 0;
    pattern_step_ = -1;
    size_step_ = -1;
    position_slot_ = -1;
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
    if (seed_trigger) {
      clock_period_ = samples_since_clock_ >= kMinClockPeriod &&
          samples_since_clock_ <= kMaxClockPeriod ? samples_since_clock_ : 0;
      samples_since_clock_ = 0;
      grain_rate_phasor_ = 0.0f;
      // While frozen, keep the grid of the frozen recording.
      if (!parameters.freeze) {
        recorded_since_clock_ = kTriggerLatencyBlocks * size;
      }
      uint32_t tick = clock_count_++;
      // With a clock, the DENSITY pattern decides whether this tick fires.
      // SPREAD variations follow the tick's place in the pattern.
      if (clock_period_ > 0) {
        seed_trigger = PatternHit(parameters.granular.rhythm_side,
            parameters.granular.overlap, tick);
        hit_index_ = tick % kPatternLength;
      }
    }

    // With a clock, grains only fire on clock ticks. Without one, grains run
    // free.
    bool clock_locked = clock_period_ > 0;
    
    // Build a list of available grains.
    int32_t num_available_grains = FillAvailableGrainsList();
    
    // Try to schedule new grains.
    for (size_t t = 0; t < size; ++t) {
      grain_rate_phasor_ += 1.0f;
      bool seed = !clock_locked && grain_rate_phasor_ >= space_between_grains;
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
        grain_rate_phasor_ = 0.0f;
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
  // Whether clock tick number tick fires a grain. side: -1 left of noon,
  // 0 noon, 1 right of noon; amount: 0 at noon to 1 at the end of travel.
  bool PatternHit(int32_t side, float amount, uint32_t tick) {
    if (side != pattern_side_) {
      pattern_side_ = side;
      pattern_step_ = -1;
    }
    if (side < 0) {
      uint16_t pattern = kStraightPatterns[QuantizeWithHysteresis(
          amount, kNumStraightPatterns, &pattern_step_)];
      return (pattern >> (tick % kPatternLength)) & 1;
    } else if (side > 0) {
      uint32_t k = kEuclideanHits[QuantizeWithHysteresis(
          amount, kNumEuclideanPatterns, &pattern_step_)];
      return ((tick % kPatternLength) * k) % kPatternLength < k;
    }
    return true;
  }

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

    // SPREAD is variation: each hit of the repeating pattern may pan, get
    // quieter, play reversed, jump to a nearby slice or an octave up.
    float variation = parameters.stereo_spread;
    uint32_t hit = hit_index_++;
    float accent = 1.0f - 0.7f * variation * HitHash(hit, 1);
    bool flip = HitHash(hit, 2) < 0.35f * variation;
    float scatter = (2.0f * HitHash(hit, 3) - 1.0f) * 3.0f * variation;
    if (HitHash(hit, 4) < 0.2f * variation) {
      pitch += 12.0f;
    }

    float pitch_ratio = SemitonesToRatio(pitch);
    float inv_pitch_ratio = SemitonesToRatio(-pitch);
    bool synced = clock_period_ > 0;
    float grain_size;
    if (synced) {
      int32_t step = QuantizeWithHysteresis(
          parameters.size, kNumSyncedSizes, &size_step_);
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
    float pan = 0.5f + variation * (HitHash(hit, 0) - 0.5f);
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
    gain_l *= accent;
    gain_r *= accent;
    
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

    bool reverse = parameters.granular.reverse != flip;
    int32_t size = static_cast<int32_t>(grain_size) & ~1;
    float delay = position * available + eaten_by_play_head;
    if (synced) {
      // Candidate start points sit at whole slices before/after the last
      // clock tick: delay = phase + j * slice. POSITION picks j.
      // The range of j doesn't depend on phase, so a given POSITION keeps
      // picking the same slice as the record head moves.
      float slice = grain_size;
      float since_tick = static_cast<float>(recorded_since_clock_) +
          (parameters.freeze ? 0.0f : static_cast<float>(pre_delay));
      float phase = since_tick - floorf(since_tick / slice) * slice;
      int32_t j_min = static_cast<int32_t>(ceilf(eaten_by_play_head / slice));
      int32_t j_max = static_cast<int32_t>(
          floorf((buffer_size - grain_size - slice) / slice));
      if (j_max >= j_min) {
        int32_t j = j_min + QuantizeWithHysteresis(
            position, j_max - j_min + 1, &position_slot_);
        j += static_cast<int32_t>(floorf(scatter + 0.5f));
        CONSTRAIN(j, j_min, j_max);
        delay = phase + static_cast<float>(j) * slice;
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
  int32_t recorded_since_clock_;
  uint32_t clock_count_;
  uint32_t hit_index_;
  int32_t pattern_side_;
  int32_t pattern_step_;
  int32_t size_step_;
  int32_t position_slot_;
  
  Grain grains_[kMaxNumGrains];
  int32_t available_grains_[kMaxNumGrains];
  float envelope_buffer_[kMaxBlockSize];
  
  DISALLOW_COPY_AND_ASSIGN(GranularSamplePlayer);
};

}  // namespace clouds

#endif  // CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_
