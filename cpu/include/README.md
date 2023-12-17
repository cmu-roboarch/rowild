This directory includes the implementation of miscellaneous functions RoWild
uses. Excluding `rowild_utils.h` and `vec_utils.h`, all files are third-party.

[`nlohmann`](https://github.com/nlohmann/json) is used to operate on JSON
files.

[`parallel_hashmap`](https://greg7mdp.github.io/parallel-hashmap/) is used to
implement fast, (parallel) hashmaps.

[`vcl`](https://github.com/vectorclass/version2) is used for vectorization.

[`args.h`](https://github.com/ElliotLockerman/cpp_args) is used to parse
command-line arguments.

[`glob.hpp`](https://github.com/p-ranav/glob) is used for finding files.

[`log.h`](https://github.com/s5z/zsim/blob/master/src/log.h) includes general
logging, warning, assertion, etc. routines.

[`pad.h`](https://github.com/s5z/zsim/blob/master/src/pad.h) includes general
macros for padding data.

`rowild_utils.h` implements some operations that are common among different
kernels of RoWild (e.g., matrix multiplication, etc).

[`stb_image.h`](https://github.com/nothings/stb/blob/master/stb_image.h) is
used to read PNG images.

`vec_utils.h` implements some generic vectorization operations.

[`zsim_hooks.h`](https://github.com/s5z/zsim/blob/master/misc/hooks/zsim_hooks.h)
provides utility to communicate with the [zsim](https://github.com/s5z/zsim)
simulator.
