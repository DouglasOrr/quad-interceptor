# Performance & Build Report

- **Date:** 2026-08-21T00:05:22+01:00
- **Commit:** `a79dff6d4263455d63d8dcca8d4b26a6e38941d8`
- **Power:** plugged_in
- **Tests:** passed

## Binary Size

```
  121,856 bytes  build/release/game
```

## Benchmark

```
# vsync=false -v=release

       FPS: 103 (p95 88)
Frame time: 0.4 ms (p95 0.9 ms)
  GPU busy: 72% (p95 75%)
 GPU power: 13.1 W (p95 13.8 W)
Audio busy: 7% (p95 24%)
```

```
# vsync=true -v=profile

       FPS: 60 (p95 56)
Frame time: 16.4 ms (p95 17.5 ms)
  GPU busy: 42% (p95 62%)
 GPU power: 6.3 W (p95 9.6 W)
Audio busy: 6% (p95 18%)
```

## Profile (vsync=true)

```
38.30% SDLAudioP2 game [.] AudioEngine::fill_buffer(short*, int)
0.66% game game [.] Physics::tick(GameState&)
0.58% game ld-linux-x86-64.so.2 [.] do_lookup_x
0.57% game game [.] void std::vector<float, std::allocator<float> >::_M_range_insert<float const*>(__gnu_cxx::__normal_iterator<float*, std::vector<float, std::allocator<float> > >, float const*, float const*, std::forward_iterator_tag)
0.55% game libc.so.6 [.] __memmove_evex_unaligned_erms
0.46% game game [.] City::operator()(std::array<long long, 2ul>) const
0.42% game [kernel.kallsyms] [k] clear_page_erms
0.40% game game [.] Renderer::draw(GameState const&, float)
0.39% game libc.so.6 [.] _int_malloc
0.35% game game [.] Vec<3u> mul<3u, 3u>(Mat<3u, 3u> const&, Vec<3u> const&)
0.34% game libc.so.6 [.] __memcmp_evex_movbe
0.33% PulseHotplug [kernel.kallsyms] [k] rt_mutex_slowlock_block.constprop.0
0.29% game [kernel.kallsyms] [k] get_mem_cgroup_from_mm
0.28% game libgallium-25.2.8-0ubuntu0.24.04.2.so [.] 0x00000000003de3b1
0.27% game libc.so.6 [.] malloc
0.26% game [kernel.kallsyms] [k] __mod_memcg_lruvec_state
0.26% PulseMainloop [kernel.kallsyms] [k] __raw_spin_lock_irqsave
0.25% game game [.] unsigned long long hash<unsigned long long, long long, long long>(unsigned long long, unsigned long long, long long, long long)
0.24% PulseHotplug [kernel.kallsyms] [k] native_queued_spin_lock_slowpath
0.24% game libc.so.6 [.] __printf_buffer
```
