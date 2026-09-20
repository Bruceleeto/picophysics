# picophysics

A small 3D rigid body physics engine.


## Use
Copy `picophysics.h` into your project. In exactly one `.c` file:

```c
#define PICOPHYSICS_IMPLEMENTATION
#include "picophysics.h"
```

Everywhere else, just include it. Then:

```c
PPVec3 g = {.xyz = {0, -9.8f, 0}}, p = {.xyz = {0, 5, 0}};
pp_physics_set_gravity(&g);
PPBody *ball = pp_physics_create_sphere(0.5f, &p, 1.0f, 0);

pp_physics_step(1.0f / 60.0f, 8, 4);   // once per frame
```

Pools are small by default (32 bodies, 128 triangles). Raise them by defining
`PICOPHYSICS_MAX_OBJECTS`, `PICOPHYSICS_MAX_TRIANGLES` and friends before the
implementation include. The API is documented in the header.

## SH4ZAM

Define `PICOPHYSICS_USE_SH4ZAM`

## Tests

```sh
git submodule update --init      # pulls SH4ZAM into extern/, for tests_sh4zam
mkdir build && cd build
cmake ..
make
./tests/tests
./tests/tests_sh4zam             # same tests, maths through SH4ZAM
```

NOTE: For KOS users, use `kos-cmake` instead of `cmake`. Both suites then come
out as Dreamcast ELFs, and `make run-tests` / `make run-tests-sh4zam`

On a PC `tests_sh4zam` uses SH4ZAM's software back-end, so SH4ZAM quirks show
up as a failing test rather than as boxes falling through the floor on
hardware. It cannot reproduce the SH4's own arithmetic (reduced-precision
`1/sqrt`, table-driven sin/cos), so run the Dreamcast build too.


## License

MIT or public domain, your choice. See the end of `picophysics.h`.
