# Typhoon granular firmware (personal project)

Custom granular mode (mode 1) for an After Later Audio Typhoon running
SuperParasites. Repo: freerider197/typhoon-granular (not a fork; never open
PRs against patrickdowling/superparasites).

## Baseline
`v6` (tag) is the version the owner keeps and likes. Build on it.
- With a TRIG clock, grains fire only on clock ticks, never faster.
- DENSITY: noon every tick; left = straight 16-tick patterns down to every
  8th tick; right = Euclidean 13/11/9/7/5/3 of 16.
- SIZE (clocked): 1/4, 1/2, 3/4, 1, 1.5, 2, 3, 4 ticks.
- POSITION: clock-aligned slices, hysteresis, TRIG latency compensated.
- SPREAD: deterministic 16-tick groove variation (pan, accent, reverse,
  slice scatter, octave).
- TEXTURE: click-free windows (0..0.6), diffuser 0.6..1.
Code: supercell/dsp/granular_sample_player.h, grain.h, granular_processor.cc,
fx/diffuser.h.

## Owner preferences
- Microcell build only (`VARIANT=MICROCELL`). One WAV per release.
- Musical, rhythmic results (techno 130-145 bpm, clocked); no audio-rate
  grain repeats, no random skips.
- Short, direct answers.

## Building
Release toolchain is gcc-arm-none-eabi 5.4-2016q3 + python2 encoder. In
cloud sessions a newer arm-none-eabi-gcc works with: C++ `-std=gnu++03`,
`-Wno-error` appended, and the linker script discarding `.ARM.exidx*`.
Firmware WAV: stm_audio_bootloader/qpsk/encoder.py
`-t stm32f4 -s 48000 -b 12000 -c 6000 -p 256` (needs a python3 port).
Flash: hold FREEZE at power-up, play WAV into IN L, re-patch audio after.
