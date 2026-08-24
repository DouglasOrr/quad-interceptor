# Binary Size Notes

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
