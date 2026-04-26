cmake_minimum_required(VERSION 3.0.2)
project(mpc_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  f110_msgs
  dynamic_reconfigure
)

# ### HJ : Phase 5 — dynamic_reconfigure server for per-instance MPC tuning.
# MPCCost       : legacy MPCC / Frenet-D backends
# FrenetKinCost : FrenetKin v3+ backend (opti.parameter() hot-swap, JIT-safe)
generate_dynamic_reconfigure_options(
  cfg/MPCCost.cfg
  cfg/FrenetKinCost.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  node/mpc_planner_node.py
  node/mpc_planner_state_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
