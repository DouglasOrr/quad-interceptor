# Binary Size Notes

> Reverse chronological order. Brief.

## 2026-08-24 (8c12eb9) - PID refactor grows release exe ~4k

Moving drone PID state out of `Drone`/`DroneType` into a `Physics::State`
(State pattern, synced via `sync_parallel`) added ~4k to the release exe.
Growth is confined to `physics.o` (+3.5k); `game_manager.o`/`renderer.o` are
unchanged. Drivers:

- `Physics::tick` now instantiates `sync_parallel` over `Physics::State::Drone`
  (sorted-merge loop + init lambda + vector reserve/push/move), +231 in `tick`.
- New `Physics` ctor/dtor ABI functions and a `pid_apply` helper (~114B).
- New `std::vector<Physics::State::Drone>` realloc/growth helper (~198B).

Lifting the PID state off the AI shrank `AI::update` (-73B), so the PID-state
split itself is cheap; the State-pattern wiring (extra vector + `sync_parallel`)
is the dominant cost.

---

## 2026-08-20 (10aa28b) - High-level profiling & report: growth is mostly padding

Added profiling plumbing to `src/platform.cpp` (+8.2k: 113616 -> 121848). All
real code is in `platform.o` (+4.6k object); renderer/physics/audio/game_manager
are byte-identical. Drivers:

- `std::string`/`std::atomic`/`std::vector`/`std::ofstream` + extra includes,
  a `profile_path` arg through `Window`'s ctor, and
  `SDL_GetPerformanceCounter()` timing around swap/audio.

Caveat on the raw size: only ~1.8k of it is actual `.text` (+1780); the rest is
page-alignment padding. The added code pushed `.rodata` across a 4 KiB page
boundary (third `LOAD` base moves 0x13000 -> 0x14000), cascading the layout.
So the real functional cost is ~1.8k, not the headline 8.2k.

---

## 2026-08-06 (e806305) - Motors audio effects grows release exe ~8k

Added the procedural motor-sound subsystem (+8328B: 47760 -> 56088). Growth is
almost entirely new code (`.text` +3.7k); there is no meaningful data/bss growth.
Drivers:

- New `audio.o` (+3.2k object text): per-sample, per-drone, per-motor synthesis
  loop (`fill_buffer`) plus `std::mutex`-guarded `State`, dragging in
  `<vector>`/`<array>`/`<mutex>`/`<functional>` template instantiations.
- `renderer.o` (+661): `camera_pos()`/`camera_right()` accessors and moving the
  FPS timer into `Renderer::draw`.
- `platform.o` (+573): SDL audio device open/close and the `sdl_audio_callback`.
- `physics.o` (+124): computing the new `motor_speed` field.

The whole jump is the new audio subsystem + SDL audio plumbing (unwind/PLT
overhead from the added C++ runtime use), not a single bloated feature.

---
