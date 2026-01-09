# Use Ubuntu 22.04 LTS as base image
FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
# build-essential: for make, gcc, etc.
# clang, libc++-dev, libc++abi-dev: for compiling with -stdlib=libc++ as used in examples
# python3, python3-numpy: for helper scripts
RUN apt-get update && apt-get install -y \
    build-essential \
    clang \
    libc++-dev \
    libc++abi-dev \
    git \
    python3 \
    python3-pip \
    python3-numpy \
    make \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy the repository
COPY . /app

# Build CUDD library
# We perform this step to ensure the CUDD library included in the repo is built and ready.
# We use clang to be consistent with the examples' compiler choice.
WORKDIR /app/cudd-3.0.0
RUN make clean && make CXX=clang++ GCC=clang

# Build one_arm example
WORKDIR /app/examples/one_arm
RUN make clean && make

# Build two_arm example
WORKDIR /app/examples/two_arm
RUN make clean && make

# Return to root
WORKDIR /app

# Default command
CMD ["/bin/bash"]
