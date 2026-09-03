# Quad Interceptor

An ultra-small quadcopter simulation game that consumes < 1.44 MB on disk, so as to [fit on a floppy](https://2pgarcade.com/contest-144mb.html).

## Running

We support Linux (Ubuntu 24.04), with:

```sh
apt install libsdl2-2.0-0 libgl1 libegl1 libgl1-mesa-dri
./quad-interceptor
```

## Development

```sh
apt install clang clang-format libsdl2-dev ninja-build
./dev setup

./dev --help
```

Consider installing [bloaty](https://github.com/google/bloaty) for binary size analysis.
