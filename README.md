# UR5e Center-Pose MoveIt Standard IK

## Scope

The solver requests arm joint solutions from MoveIt through the standard `GetPositionIK` service at `/compute_ik`. The included MoveIt configuration selects `kdl_kinematics_plugin/KDLKinematicsPlugin`.

`moveit_center_ik.py` holds the midpoint pose while the hand servo changes, transforms that target into a flange pose, requests a solution from MoveIt, and executes the resulting arm trajectory.

This repository isolates the selected method from the larger TRiSPAN ROS 2 workspace. It preserves the original ROS package names and the model/configuration files needed to understand the experiment. ROS 2 Humble, MoveIt 2, the Universal Robots description package, and the remaining workspace runtime dependencies are external prerequisites. Use the complete workspace archive for the original launch environment.

## Source layout

- `src/ur5e_hand_integration/scripts/`: MoveIt IK node and required mock/runtime helper nodes.
- `src/ur5e_hand_integration/urdf/` and `meshes/`: the UR5e hand model assets used by the experiment.
- `src/ur5e_hand_moveit_config/config/`: the MoveIt SRDF and solver/controller configuration.
- `src/ur5e_hand_moveit_config/launch/moveit_standard_center_ik_demo.launch.py`: the method-specific RViz mock launch.

## Reproduction context

Launch `ur5e_hand_moveit_config moveit_standard_center_ik_demo.launch.py` in the archived ROS 2 workspace. The archived dataset and run logs are kept separately from this source repository.

The archived RViz mock comparison reports 210/210 samples within 0.5 mm position and 0.1 degree orientation tolerance. The paired 0.05-second trajectory-condition data report a mean center-position error of 0.04186 mm and a maximum of 0.48753 mm for MoveIt. These are simulation-only measurements; trajectory timing is not IK computation time.

The reported values describe the recorded mock experiment and are not a claim of real-robot accuracy. No license was declared in the original source package, so this repository does not add a license grant.
