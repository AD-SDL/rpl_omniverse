# rpl_omniverse

## Top level files

**start_sim_docker.sh**: Start the Isaac Sim docker container. (Run ./runheadless.native.sh in the container, then use the Streaming Client in Omniverse to attach to the it). Useful for ROS2 since Ubuntu 22.04 is currently not compatible with Isaac Sim's ROS2 setup.

**start_sim_python.sh**: Start the Isaac Sim python in the current terminal.

**ov_layout_rmb.json**: Rearrangement of Omniverse window elements. In Omniverse, use "Window->Layout->Load Layout..." to load this file.
