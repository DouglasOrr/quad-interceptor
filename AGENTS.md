# Small game that fits in 1.44 MB

## Structure

```
examples/<DD>-<name> -- standalone examples/demos
src/                 -- the game; all *.cpp are globbed by `dev` into one `game` binary
```

## Commands

- `./dev -v <debug|release> build` (also `run`, `clean`, `ci`)
- `./dev run test`

## Instructions

- Leave git commands to me, except e.g. when reviewing diffs.

## Style

- C++20, -Wall -Wextra -Werror, clang-format.
- Prefer concise code. Inline implementations where possible. Outline for reuse or clarity, where it reduces complexity.
- Simple and direct over abstract and general.
