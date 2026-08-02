# Small game that fits in 1.44 MB

## Structure

```
dev                  -- automation (standard build & run commands)
src/                 -- the game; all *.cpp are globbed by `dev` into one `game` binary
examples/<DD>-<name> -- standalone examples/demos
```

## Commands

- `./dev -v <debug|release> build` (also `run`, `clean`, `ci`)
- `./dev run test`

## Instructions

- Leave git edit commands to the user, except where explicitly prompted.

## Style

- C++20, -Wall -Wextra -Werror, clang-format.
- Prefer concise code. Inline implementations where possible. Outline for reuse or clarity, where it reduces complexity.
- Simple and direct over abstract and general.
- Use comments sparingly, only when intent is otherwise unclear, and keep them brief.
