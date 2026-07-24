# 01-Decompression Experiment Report

This experiment evaluates different decompression algorithms and approaches for resource-constrained environments (such as embedded systems or small games fitting within 1.44 MB).

## Benchmark Results (Dictionary: `/usr/share/dict/words`, Original Size: 985,084 bytes)

| Method | Decompressor Binary Size (bytes) | Compressed Size (bytes) | Compression Ratio (%) | Verification |
| :--- | :--- | :--- | :--- | :--- |
| **Baseline** (No compression) | 14,608 | 985,084 | 100.0% | PASS |
| **Heatshrink** (LZW-like, w=11, l=4) | 14,728 | 373,001 | 37.9% | PASS |
| **aPLib / apultra** (aPLib format) | 14,712 | 317,777 | 32.3% | PASS |
| **Puff** (Mark Adler's DEFLATE) | 18,824 | 264,762 | 26.9% | PASS |
| **Miniz** (zlib-subset DEFLATE) | 22,928 | 269,636 | 27.4% | PASS |

## Findings

1. **Baseline**: Minimal binary footprint (14.6 KB) but zero compression.
2. **Heatshrink**: Extremely small binary footprint (14.7 KB) with moderate compression (37.9%), well-suited for extremely memory-constrained streaming decompression.
3. **aPLib / apultra**: Excellent balance of tiny decompressor binary size (14.7 KB, virtually identical to baseline/heatshrink) and strong compression ratio (32.3%), making it exceptionally attractive for tight storage constraints.
4. **Puff**: Superior DEFLATE compression ratio (26.9%) while maintaining a very compact decompressor binary size (18.8 KB), significantly smaller than full miniz/zlib implementations.
5. **Miniz**: Full zlib-compatible DEFLATE compression and decompression. Provides top-tier compression (27.4%), with a slightly larger decompressor binary size (22.9 KB).
