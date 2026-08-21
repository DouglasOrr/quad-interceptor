# Performance & Build Report

- **Date:** 2026-08-21T01:02:50+01:00
- **Commit:** `ae1338343133c12e68617adb0807fa391858f84b`
- **Power:** plugged_in
- **Tests:** passed

## Binary Size

```
  121,856 bytes  build/release/game
```

## Benchmark

```
# vsync=false -v=release

        FPS: 84 (p95 54)
Frame time: 0.4 ms (p95 0.8 ms)
  GPU busy: 75% (p95 83%)
 GPU power: 13.7 W (p95 14.8 W)
Audio busy: 1% (p95 2%)
```

```
# vsync=true -v=profile

        FPS: 59 (p95 53)
Frame time: 11.3 ms (p95 17.2 ms)
  GPU busy: 59% (p95 81%)
 GPU power: 10.6 W (p95 14.4 W)
Audio busy: 1% (p95 2%)
```

## Profile (vsync=true)

```
6.19% SDLAudioP2 game [.] AudioEngine::fill_buffer(short*, int)
1.18% game game [.] void std::vector<float, std::allocator<float> >::_M_range_insert<float const*>(__gnu_cxx::__normal_iterator<float*, std::vector<float, std::allocator<float> > >, float const*, float const*, std::forward_iterator_tag)
1.10% game game [.] Physics::tick(GameState&)
1.08% game game [.] AI::update(GameState&, bool)
0.99% game game [.] City::operator()(std::array<long long, 2ul>) const
0.97% game libc.so.6 [.] __memmove_evex_unaligned_erms
0.79% PulseHotplug [kernel.kallsyms] [k] rt_mutex_slowlock_block.constprop.0
0.71% game game [.] Renderer::draw(GameState const&, float)
0.65% game ld-linux-x86-64.so.2 [.] do_lookup_x
0.63% game libc.so.6 [.] malloc
0.56% game libc.so.6 [.] __memcmp_evex_movbe
0.52% game game [.] Vec<3u> mul<3u, 3u>(Mat<3u, 3u> const&, Vec<3u> const&)
0.52% game game [.] unsigned long long hash<unsigned long long, long long, long long>(unsigned long long, unsigned long long, long long, long long)
0.48% game libc.so.6 [.] _int_malloc
0.46% game libgallium-25.2.8-0ubuntu0.24.04.2.so [.] 0x00000000003de3b1
0.40% game [kernel.kallsyms] [k] clear_page_erms
0.40% game libc.so.6 [.] __printf_buffer
0.39% PulseMainloop [kernel.kallsyms] [k] __raw_spin_lock_irqsave
0.39% SDLAudioP2 [kernel.kallsyms] [k] native_queued_spin_lock_slowpath
0.38% SDLAudioP2 game [.] floorf@plt
```
