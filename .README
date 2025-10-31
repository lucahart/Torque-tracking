# On the SDP Relaxation of Direct Torque Finite Control Set Model Predictive Control
This repository provides the complete code package for the simulations shown in our paper 
*"On the SDP Relaxation of Direct Torque Finite Control Set Model Predictive Control."* 
It includes a simulation environment for an electric motor, multiple controler configurations, 
and simulation scenarios as discussed in our paper.

## Installation
This code uses the [SCS optimizer](https://www.cvxgrp.org/scs/) and the [Yalmip library](https://yalmip.github.io/tutorial/installation/). 
Their respective installation instructions can be found under the provided links. 
Please make sure that both backages are added to the path and up-to-date before running this code.

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






