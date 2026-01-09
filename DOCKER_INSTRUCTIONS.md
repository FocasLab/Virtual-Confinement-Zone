# Docker Instructions

This repository includes a `Dockerfile` to build the project and its examples in a containerized environment.

## Prerequisites

- Docker installed and **running** on your system (Docker Desktop for Windows/Mac, or Docker Engine for Linux).
- Verify with: `docker --version`

## Building the Docker Image

Run the following command in the root of the repository:

```bash
docker build -t vcz-scots .
```

This will:
1. Install necessary dependencies (Clang, libc++, Python, etc.).
2. Build the CUDD library.
3. Build the `one_arm` and `two_arm` examples.

---

## Running the Examples

Run Docker with volume mounting so output files are saved to your local folder (required for MATLAB):

**Linux/Mac:**
```bash
docker run -it -v "$(pwd)":/app vcz-scots
```

**Windows PowerShell:**
```powershell
docker run -it -v ${PWD}:/app vcz-scots
```

Inside the container, rebuild and run the examples:

### One Arm Example

```bash
cd /app/examples/one_arm
make clean && make
./one_arm_scots
```

### Two Arm Example

```bash
cd /app/examples/two_arm
make clean && make
./two_arm_scots
```

After running, type `exit` to leave the container. All output files will be available in your local `examples/` folder for use in MATLAB.

> **Note:** For post-processing instructions or details on editing each example, refer to the `readme.md` file in the respective example folder.

---

## Making Code Changes

1. **Edit** the `.cc` files on your local machine using any editor.
2. **Run** Docker with volume mount (same command as above).
3. **Rebuild** inside the container:
   ```bash
   cd /app/examples/one_arm
   make clean && make
   ./one_arm_scots
   ```
4. **Exit** — compiled files and outputs are already in your local folder.

---

## Quick Reference

| Task | Command |
|------|---------|
| Build image | `docker build -t vcz-scots .` |
| Run with volume mount | `docker run -it -v ${PWD}:/app vcz-scots` |
| Rebuild example | `make clean && make` |

---

## Note

The `MultiAgent` example requires MATLAB, which is not included in this Docker image.
