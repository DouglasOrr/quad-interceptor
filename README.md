# Small game

Creating a small game that consumes < 1.44 MB on disk, so as to [fit on a floppy](https://2pgarcade.com/contest-144mb.html).

## Requirements

```sh
apt install libsdl2-dev clang clang-format
```

## Style

- C++20, -Wall -Wextra -Werror, clang-format
- Prefer concise code, inline implementations where possible. Outline for reuse or clarity, where it reduces complexity.
- Simple and direct over abstract and general.
