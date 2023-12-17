This directory includes some scripts, mostly for checking and enforcing coding
conventions.

`add_copyright.py` is used to add the copyright message to codes.

`check_all_func_bound.py` and `func_bound.py` are used to check spaces among
consecutive functions in a file.

`clangformat_all.sh` formats the C++/CUDA codes according to LLVM guidelines.

`cpplint_all.py` checks whether C++ files observe [Google's C++ style
guide](https://google.github.io/styleguide/cppguide.html).

`find_todos.py` finds TODO, FIXME, etc. in the codes.

`fix_whitespace.sh` is used to remove trailing whitespaces.

`last_line_blank.py` is used to find files with blank last lines.

`lint_includes.py` is used to check `.h` files.

`shfmt_all.sh` formats shell scripts in the repository.

`verible_all.sh` formats Verilog codes in the repository.

The other files are not stand-alone scripts; instead, they serve as components
utilized by the other scripts.
