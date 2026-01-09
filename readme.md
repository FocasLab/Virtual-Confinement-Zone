# Welcome to SCOTS VCZ!


This repository is based on the work **"Scalable and Approximation-free Symbolic Control for Unknown Euler-Lagrange Systems"** (https://arxiv.org/abs/2509.19859) BASED ON **"SCOTS Toolbox** ".
This repository hosts the VCZ branch with one-arm, two-arm, and multi-agent examples.

**SCOTS** is an open source software tool to compute discrete abstractions and symbolic controllers.

Please consult the [manual](https://gitlab.lrz.de/matthias/SCOTSv0.2/raw/master/manual/manual.pdf) for installation instructions,
usage description and background information.

For implementation details please have a look in the C++ documentation ./doc/html/index.html

Bug reports and feature requests are happily received at <matthias.rungger@tum.de> 

### How to use:

* The basic implementation of **SCOTS** is inlined and header only. Hence, only a working C++ compiler
  with C++11 support is needed.

* The best way to find out about **SCOTS** is to clone the repository 
  
    `$ git clone https://github.com/FocasLab/Virtual-Confinement-Zone`
  
    and run the VCZ examples in the examples directory:

  * `./examples/one_arm/`    [SCOTS toolbox + MATLAB]
  * `./examples/two_arm/`    [SCOTS toolbox + MATLAB]
  * `./examples/MultiAgent/` [MATLAB]

    Have a look in the readme file for some info and compiler options

### Running with Docker

If you want to run the SCOTS toolbox using Docker (recommended for easy setup), see [DOCKER_INSTRUCTIONS.md](DOCKER_INSTRUCTIONS.md).

### Example workflows:

* **One-arm (VCZ)**
  1) `make` in `examples/one_arm` to build `one_arm_scots`.
  2) Run `./one_arm_scots` to generate `controller.scs`.
  3) In MATLAB, run `to_csv.m` to export `controller.csv`.
  4) Run `One_arm_manipulator.m` to simulate and plot results.
  5) Synthesis is performed using the SCOTS toolbox (multi-agent uses MATLAB-based abstraction/synthesis instead).

* **Two-arm (VCZ 2R)**
  1) `make` in `examples/two_arm` to build `two_arm_scots`.
  2) Run `./two_arm_scots` to generate `controller.scs`.
  3) Run `python3 scs_to_csv.py` to export `controller.csv`.
  4) Run `two_arm_manipulator.m` (or `windom.m`) in MATLAB to visualize.
  5) Synthesis is performed using the SCOTS toolbox (multi-agent uses MATLAB-based abstraction/synthesis instead).

* **Multi-agent (VCZ)**
  1) In MATLAB, run `VCZ.m` to generate the VCZ centre trajectory and save `poses.csv`.
  2) Run `VCZ_control2.m` to track the VCZ centre.
  3) Synthesis is performed using MATLAB-based abstraction and synthesis.
  
### What's new:

* VCZ branch with one-arm and two-arm manipulator examples

* The sparse branch (contributed by Eric Kim) implements the algorithm described
  in [Sparsity-Sensitive Finite Abstraction](https://arxiv.org/abs/1704.03951)

* New data structure to store the transition function of symbolic models
   
* New synthesis algorithms for invariance and reachability specifications 
    (see the manual for details)

* Dynamic variable reordering can now be safely activated throughout all computations in the BDD implementation

* An example demonstrating the usage of validated ODE solvers 
    to compute a priori enclosures and growth bounds

* Complete redesign of the code base to accommodate for modern C++

* Doxygen documentation in ./doc/html/


### Citation

If you use this code in your work, please cite the relevant SCOTS/VCZ publication. BibTeX:

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

@inproceedings{10.1145/2883817.2883834,
      author={Rungger, Matthias and Zamani, Majid},
      title={SCOTS: A Tool for the Synthesis of Symbolic Controllers},
      year={2016},
      isbn={9781450339551},
      publisher={Association for Computing Machinery},
      address={New York, NY, USA},
      url={https://doi.org/10.1145/2883817.2883834},
      doi={10.1145/2883817.2883834},
      booktitle={Proceedings of the 19th International Conference on Hybrid Systems: Computation and Control},
      pages={99--104},
      numpages={6},
      keywords={symbolic models, feedback refinement relations, discrete abstractions, c++/matlab toolbox},
      location={Vienna, Austria},
      series={HSCC '16},
}
```

### Contributions

Contributions are welcome. Please open an issue or submit a pull request with a clear description of the change and any relevant results.

### Acknowledgments

Thanks to the original SCOTS author, Matthias Rungger, for the foundational codebase and tools this VCZ branch builds on. Original SCOTS repo: [https://gitlab.lrz.de/matthias/SCOTSv0.2.git](https://gitlab.lrz.de/matthias/SCOTSv0.2.git)
