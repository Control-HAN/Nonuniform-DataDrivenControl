# Nonuniform-DataDrivenControl
S. Y. Han, “Nonuniform Data-driven Control for Networked Control Systems with Safe Set,” International Journal of Control, Automation, and Systems, Under Review.



----

MATLAB implementation of **Nonuniform-DataDrivenControl (DDC)** by [Seungyong Han](https://sites.google.com/view/jbnu-dscl)

<!-- <p align="center">
  <img src="Figures/03_Ex1_Case1_Leader_Follower_Trajectory.png" width="250" />
  <img src="Figures/12_Ex2_Case1_NODE_MPC_ILMPC_State.png" width="250" />
  <img src="Figures/19_Ex2_Case2_NODE_MPC_TDMPC_State.png" width="250" />
</p> -->

# 1. Method

<!-- The proposed **LMPC** is a framework for sampled-data control systems with partially unknown dynamics. 
In the addressed system, a plant is dependent on a time-varying parameter whose dynamics is unknown. To learn the unknown dynamics, a neural network (NN) is trained using **a neural ordinary differential equation (NODE)**. The trained NN is integrated into the sampled-data MPC framework to estimate the time-varying parameter and predict future system states. The proposed LMPC method guarantees ultimate boundedness for the sampled data control system that operates with a continuous-time plant and a discrete-time control strategy.-->

# 2. Getting Started
## Required Packages
- MATLAB R2024b+
  - CasADi 3.6.7 (https://web.casadi.org/get/)
  - MPT 3.0 (https://people.ee.ethz.ch/~mpt/2/downloads/)

## Contact
 - If you have questions or suggestions, please reach out via email [hansy@jbnu.ac.kr](mailto:hansy@jbnu.ac.kr).


