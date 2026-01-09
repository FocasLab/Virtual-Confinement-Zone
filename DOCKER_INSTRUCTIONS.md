# Docker Instructions

This repository includes a `Dockerfile` to build the project and its examples in a containerized environment.

## Prerequisites

- Docker installed on your system.

## Building the Docker Image

Run the following command in the root of the repository:

```bash
docker build -t vcz-scots .
```

This will:
1.  Install necessary dependencies (Clang, libc++, Python, etc.).
2.  Build the CUDD library.
3.  Build the `one_arm` and `two_arm` examples.

## Running the Examples

You can run the container iteratively:

```bash
docker run -it vcz-scots
```

Inside the container, navigate to the example directories to run them.

### One Arm Example

```bash
cd examples/one_arm
./one_arm_scots
```

### Two Arm Example

```bash
cd examples/two_arm
./two_arm_scots
python3 scs_to_csv.py
```

## Note

The `MultiAgent` example requires MATLAB, which is not included in this Docker image.
