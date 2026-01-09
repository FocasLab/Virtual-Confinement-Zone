# Virtual Confinement Zone (VCZ) for Symbolic Control

This repository implements the **Virtual Confinement Zone (VCZ)** framework, as described in the research paper:
**["Scalable and Approximation-free Symbolic Control for Unknown Euler-Lagrange Systems"](https://arxiv.org/abs/2509.19859)**.

The project builds upon the foundational **SCOTS Toolbox** (Symbolic Control of Transition Systems) to provide robust control strategies for Euler-Lagrange systems with unknown dynamics.

---

## Overview

Traditional symbolic control requires precise models or conservative approximations. The VCZ approach enables:
* **Scalable Synthesis**: Efficiently handles complex systems by confining them within parameterized zones.
* **Model-Free Guarantee**: Provides safety and reachability even when system dynamics are unknown.
* **Integrated Workflow**: Combines C++ for high-performance controller synthesis with MATLAB for simulation and visualization.

## Getting Started

### Prerequisites
*   A C++11 compliant compiler (Clang recommended).
*   [MATLAB](https://www.mathworks.com/products/matlab.html) (for post-processing and simulation).
*   [CUDD Library](https://github.com/ivmai/cudd) (Colorado University Decision Diagram).

### Installation
1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/FocasLab/Virtual-Confinement-Zone.git
    cd Virtual-Confinement-Zone
    ```
2.  **Install CUDD**: Follow the instructions in [INSTALL_CUDD.md](INSTALL_CUDD.md) to build the library locally.

#### Docker-based Execution
For a quick setup without manual C++ dependency management:
1.  **Build/Run**: Refer to [DOCKER_INSTRUCTIONS.md](DOCKER_INSTRUCTIONS.md) for commands to build the image and run containers with volume mounting.
2.  **Inside Container**: Use the same synthesis steps as described in the Manipulator workflows.
3.  **Visualization**: All outputs are synced to your local machine for use in MATLAB.

---

## Repository Structure

*   `./examples/`: Contains benchmark implementations:
    *   `one_arm`: Single-arm manipulator using SCOTS + MATLAB.
    *   `two_arm`: Two-arm manipulator (2R) using SCOTS + MATLAB.
    *   `MultiAgent`: Multi-agent system implemented entirely in MATLAB.
*   `./doc/`: Doxygen-generated C++ documentation.
*   `./mfiles/`: Core MATLAB functions for symbolic control and VCZ visualization.
*   `./src/`: Underlying SCOTS header-only library.

---

## Example Workflows

### 1. One-Arm & Two-Arm Manipulators
These examples use the SCOTS toolbox for controller synthesis.
1.  **Build**: Navigate to `examples/one_arm` (or `two_arm`) and run `make`.
2.  **Synthesize**: Execute `./one_arm_scots` (or `./two_arm_scots`) to generate `controller.scs`.
3.  **Export**: Open MATLAB, `cd` to the example folder, and run `to_csv.m` to convert the controller to csv format.
4.  **Simulate**: Run the main manipulator script (e.g., `One_arm_manipulator.m`) to simulate and  visualize the results.

### 2. Multi-Agent Systems
This example is handled directly within MATLAB.
1.  **Synthesize**: Run `VCZ.m` to compute the VCZ center trajectory and save `poses.csv`.
2.  **Control**: Run `VCZ_control2.m` to track the simulated trajectory.



---

## Citation

If you use this work in your research, please cite:

```bibtex
@misc{das2025scalableapproximationfreesymboliccontrol,
      title={Scalable and Approximation-free Symbolic Control for Unknown Euler-Lagrange Systems},
      author={Ratnangshu Das and Shubham Sawarkar and Pushpak Jagtap},
      year={2025},
      eprint={2509.19859},
      archivePrefix={arXiv},
      primaryClass={eess.SY},
      url={https://arxiv.org/abs/2509.19859},
}

@inproceedings{rungger2016scots,
      author={Rungger, Matthias and Zamani, Majid},
      title={SCOTS: A Tool for the Synthesis of Symbolic Controllers},
      booktitle={Proceedings of the 19th International Conference on Hybrid Systems: Computation and Control (HSCC)},
      year={2016},
      doi={10.1145/2883817.2883834}
}
```

## Acknowledgments
We acknowledge **Matthias Rungger** for the original SCOTS toolbox. This branch extends the toolbox to support VCZ-based control for Euler-Lagrange systems.
