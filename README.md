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

The proposed **LMPC** is a framework for sampled-data control systems with partially unknown dynamics. 
The nonuniform data-driven control (DDC) method is proposed for safe networked control systems (NCSs). The proposed DDC approach guarantees closed-loop stability and keeping the system state within a prescribed safe operating region, even when the plant is subject to physical constraints. In this method, nonuniformly observed data is utilized to design a stabilizing controller without an explicit plant model. The controller design condition is derived by using a looped-functional approach. This condition is formulated in terms of linear matrix inequalities (LMIs) that incorporate the constraint of a safe set. Finally, the effectiveness of the proposed method is validated through a numerical example, demonstrating that the closed-loop system can indeed be stabilized.

# 2. Getting Started
## Required Packages
- MATLAB R2024b+

## Contact
 - If you have questions or suggestions, please reach out via email [hansy@jbnu.ac.kr](mailto:hansy@jbnu.ac.kr).


