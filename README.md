# rpl_omniverse

## cad/

This repo only contains converted files (cad/*/*/2_export/*.usdc) from turning the source CAD files into something more polished for Omniverse.
To get the source CAD files, you must first have an official Argonne login with access to the ANL Box, then ask Rory Butler for the link.

## customization/

**ov_layout_rmb.json**: Custom layout of Omniverse window elements mimicking Unity. In Omniverse, use "Window->Layout->Load Layout..." to load this file.

## scripts/

**blend_to_usd.py**: Converts all cad/**/*.blend files to .usdc files.

**start_sim_docker.sh**: Start the Isaac Sim docker container. (Run ./runheadless.native.sh in the container, then use the Streaming Client in Omniverse to attach to the it). Useful for ROS2 since Ubuntu 22.04 is currently not compatible with Isaac Sim's ROS2 setup.

**start_sim_python.sh**: Start the Isaac Sim python in the current terminal.

## usd/

Contains .usda files that reference the .usdc files in cad/. These files are where materials, joints, and any other setup should be added for import to other scenes.
