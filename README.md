# On the SDP Relaxation of Direct Torque Finite Control Set Model Predictive Control
This repository provides the complete code package for the simulations shown in our paper 
*"On the SDP Relaxation of Direct Torque Finite Control Set Model Predictive Control."* 
It includes a simulation environment for an electric motor, multiple controler configurations, 
and simulation scenarios as discussed in our paper.

## Installation
This code uses the [SCS optimizer, version 3.2.9](https://www.cvxgrp.org/scs/). 
Its installation instructions can be found under the provided link. 
Please make sure that the package is added to the MATLAB path and have the correct version is installed before running this code. 

Run ```Setup.m``` when first opening the repository. 
You can also start by directly running the Simulation files described below.
After running Setup.m, you can optionally run ```savepath``` to keep this repository on your MATLAB path for future runs. 
However, we do not recommend doing so, as the local version of YALMIP might interfere with other YALMIP versions.

**Note:** We included YALMIP-R20210331 in this repo for ease of use. 
Due to compatibility issues with the latest version of SCS, we changed the methods ```extras/sdpsettings``` and ```solvers/callscs```. 
These changed methods can be found under ```src/Tools```, which is added to the MATLAB path at every execution, such that the outdated YALMIP functions are shadowed. 
This allows the usage of the included YALMIP-R20210331, as well as YALMIP-R20210331 in a different repository. You can optionally download your own version of [YALMIP-R20210331 here](https://github.com/yalmip/YALMIP/releases/tag/R20210331). 
*If YALMIP is hosted in a separate repository, make sure you are using the correct version, R20210331.*

## Citation
<!-- You can find a free-access version of our paper on [arXiv](https://arxiv.org/abs/2412.11666). -->
If using this code for academic work, cite us. Here are the Text and BibTex cytations:
```Text Citation
Luca M. Hartmann, Orcun Karaca, Tinus Dorfling, Tobias Geyer, Adam Kurpisz,
"On the SDP Relaxation of Direct Torque Finite Control Set Model Predictive Control,"
2024, arXiv:2412.11666.
```
```BibTex Citation
@misc{hartmann2024sdprelaxationdirecttorque,
      title={On the SDP Relaxation of Direct Torque Finite Control Set Model Predictive Control}, 
      author={Luca M. Hartmann and Orcun Karaca and Tinus Dorfling and Tobias Geyer and Adam Kurpisz},
      year={2024},
      eprint={2412.11666},
      archivePrefix={arXiv},
      primaryClass={math.OC},
      url={https://arxiv.org/abs/2412.11666}, 
}
```

## Simulations
The simulations of interest can be found in the root folder and in Numerical_Case_Studies:

```main.m``` in the root folder provides a flexible, single simulation. It is currently set up to simulate a single torque step to show the impact of the SDP relaxations in finding good solutions.

```Numerical_Case_Studies/main_Initial_Guess.m``` explores the impact of different initial guesses on the number of nodes traversed by the branch-and-bound algorithm.

```Numerical_Case_Studies/main_Simulation_Sweep.m``` runs an extensive simulation sweep over multiple steps with different controller types.






