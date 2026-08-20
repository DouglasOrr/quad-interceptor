# Small game that fits in 1.44 MB

## Structure

```
dev                  -- automation (standard build & run commands)
src/                 -- the game; all *.cpp are globbed by `dev` into one `game` binary
  game.hpp           -- common header for most things (except platform)
  platform.hpp       -- target platform wrapper
  prog/              -- entry points
  tests/             -- unit tests
  *.cpp              -- core implementation
examples/<DD>-<name> -- standalone examples/demos
```

## Commands

- `./dev -v <debug|release|profile> build` (also `run`, `clean`, `ci`, `benchmark`)
- `./dev run test`

## Instructions

- Leave git edit commands to the user, do not stage/unstage, commit or push, except where explicitly prompted.
- If asked to use a worktree, put it in `./agents/<name>`
- In general, don't generate separate headers for new components - prefer game.hpp.
- Keep constants, state and helper function declarations hidden in cpp files where possible.
- Don't define the same constants in multiple places - if needed by multiple cpp files, it goes into the header.

## Style

- C++20, -Wall -Wextra -Werror, clang-format.
- Simple and direct over abstract and general.
- Prefer concise code. Inline implementations where possible. Outline for reuse or clarity, where it reduces complexity.
- Avoid default arguments/initialisers - prefer being explicit at call site.
- Use comments sparingly, only when intent is otherwise unclear, and keep them brief.

## Writing a report.md

Match the existing format (keep brief & focussed).

- Record ISO datetime, commit hash, "battery" or "plugged_in"
  - Run `./dev -v debug run test` and `./dev -v release run test`, report "passed" or "failed (tests...)"
- Run `./dev -v release build game` and report the size (printed)
- Run `./dev -v release benchmark` and copy in the output.
- Run `./dev -v profile benchmark --vsync` and copy in the output. Also copy the top 20 lines from `perf.report`.
