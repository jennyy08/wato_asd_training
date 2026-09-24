# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

## My Submission

This project is a ROS 2 navigation stack for a simulated differential-drive robot. The robot uses lidar readings from `/lidar` and its provided position from `/odom/filtered` to plan and follow a route to a user-specified goal in a static obstacle course.

I implemented four nodes:

- **Costmap:** Converts lidar scans into a local occupancy grid and inflates detected obstacles. Input: `/lidar`. Output: `/costmap`.
- **Map memory:** Integrates local costmaps over time using odometry to build a persistent global map. Inputs: `/costmap`, `/odom/filtered`. Output: `/map`.
- **Planner:** Uses A* over the global occupancy grid to plan a route to a goal point. Inputs: `/map`, `/odom/filtered`, `/goal_point`. Output: `/path`.
- **Control:** Uses Pure Pursuit to follow the planned path and publish velocity commands. Inputs: `/path`, `/odom/filtered`. Output: `/cmd_vel`.

The implementation also includes obstacle inflation, periodic map updates, replanning, and clearing invalid paths so the controller does not continue following an outdated route. Working through the project helped me understand how ROS 2 nodes communicate, how occupancy grids and coordinate frames represent the environment, and how perception, mapping, planning, and control fit together in a navigation system.

Acknowledgement: This project was completed as part of WATonomous's ASD onboarding assignment, which provided the simulation and assignment framework.
