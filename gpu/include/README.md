This directory includes the implementation of miscellaneous functions RoWild
uses. Excluding `rowild_utils.h` and `vec_utils.h`, all files are third-party.

[`moderngpu`](https://github.com/moderngpu/moderngpu) is a general-purpose,
productivity library for GPUs.

[`nlohmann`](https://github.com/nlohmann/json) is used to operate on JSON
files.

[`vcl`](https://github.com/vectorclass/version2) is used for vectorization.

[`args.h`](https://github.com/ElliotLockerman/cpp_args) is used to parse
command-line arguments.

[`glob.hpp`](https://github.com/p-ranav/glob) is used for finding files.

[`log.h`](https://github.com/s5z/zsim/blob/master/src/log.h) includes general
logging, warning, assertion, etc. routines.

`rowild_utils.h` implements some operations that are common among different
kernels of RoWild (e.g., matrix multiplication, etc).

[`stb_image.h`](https://github.com/nothings/stb/blob/master/stb_image.h) is
used to read PNG images.

`timer.h` implements a simple timer class.

`vec_utils.h` implements some generic vectorization operations.
