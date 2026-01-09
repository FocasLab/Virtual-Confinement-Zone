# Installing CUDD Library

This repository includes the CUDD (Colorado University Decision Diagram) library version 3.0.0. Follow these steps to compile and install it for use with the examples.

## Prerequisites

Ensure you have the following installed on your system:
- `make`
- A C/C++ compiler (`gcc`/`g++` or `clang`/`clang++`)

On Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install build-essential
```

## Compilation Steps

1.  Navigate to the CUDD directory:
    ```bash
    cd cudd-3.0.0
    ```

2.  Run `make` to build the library:
    ```bash
    make
    ```
    
    **Note:** If you prefer to use `clang`, you can run:
    ```bash
    make CXX=clang++ GCC=clang
    ```

3.  **Verification**:
    After the build completes, check that the library file `libcudd.a` has been created in the `lib` directory:
    ```bash
    ls lib/libcudd.a
    ```

## Usage

The `one_arm` and `two_arm` examples in this repository are configured to link against this locally built CUDD library. You do *not* need to install it to your system directories (like `/usr/local/lib`).
