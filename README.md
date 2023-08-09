# Task Planning with Plansys2

## Table of Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Getting Started](#getting-started)
- [Configuring PDDL Extension](#configuring-pddl-extension)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

Task planning plays a crucial role in various domains, from robotics and automation to project management and artificial intelligence. PDDL 2.1 (Planning Domain Definition Language) offers a formal way to define planning problems, while Plansys2 provides an efficient framework for plan execution and representation using ROS2.

This repository showcases the PlanSys2 implementation for ARIAC, offering insights into utilizing PDDL and Plansys2 for multi-robot agile simulation task planning. It also promises upcoming examples that demonstrate the effective application of these techniques in various scenarios ideally with.

## Dependencies

- [ROS2(Galactic)](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [ARIAC Workspace](https://github.com/usnistgov/ARIAC)
- [PlanSys2](https://plansys2.github.io/index.html)
- [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)


## Getting Started

To get started with this repository, follow these steps:

1. Clone the repository to your local machine using the following command:

    ```sh
    cd <Your ROS2 workspace src folder>
    git clone https://github.com/Sanchitkedia/Task_Planning_PlanSys2.git plansys2_ariac
    cd ..
    rosdep update --include-eol-distros
    rosdep install --from-paths src -y --ignore-src
    colcon build --packages-select plansys2_ariac
    ```

2. Install Plansys2

    ```sh
    sudo apt install ros-<ros2-distro>-plansys2-*
    ```

3. Install the PDDL Extension from VS Code [PDDL](https://marketplace.visualstudio.com/items?itemName=jan-dolejsi.pddl):

    Launch VS Code Quick Open (<kbd>control</kbd> + <kbd>shift</kbd> + <kbd>p</kbd>) paste the following command and press <kbd>enter</kbd>

    ```Vscode
    ext install jan-dolejsi.pddl
    ```

4. Install and Setup [ARIAC Workspace](https://ariac.readthedocs.io/en/latest/getting_started/installation.html)

## Configuring PDDL Extension

Follow these steps to set up the PDDL extension for Durative Actions and configure the planner as popf:

1. Make sure that ROS2 is sourced

2. Perform the following actions to clone and build the popf repository:

    - Execute the command below in the terminal:

        ```sh
        sudo apt-get -qy install git g++ cmake coinor-libcbc-dev coinor-libcgl-dev coinor-libclp-dev coinor-libcoinutils-dev bison flex
        git clone -b vscode https://github.com/DanigarciaLopez/popf.git
        cd popf
        mkdir build
        cd build
        cmake ../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE=TRUE
        make -j
        sudo make install
        ```

3. Launch VS Code Quick Open (<kbd>control</kbd> + <kbd>shift</kbd> + <kbd>p</kbd>) paste the following command `PDDL:Select planner` and press <kbd>enter</kbd>

4. Choose the option `Create New Planner Configuration`

5. Select `POPF` from the available choices

6. In the popup window, navigate to the previously created build folder `/home/$USER/popf/build` From there, select the `popf executable`

## Usage

To generate the plan using the extension right click on the [problem file](pddl/kitting_problem.pddl) and select `PDDL: Run the planner and Display the Plan -> No options (use defaults)`

To use the Simulation with ARIAC follow the following steps:

1. Open 3 Terminals

2. Terminal 1:

    ```sh
    source /opt/ros/galactic/setup.bash
    source <Your workspace>/install/setup.bash
    ros2 launch ariac_gazebo ariac.launch.py
    ```

3. Terminal 2:

    ```sh
    source /opt/ros/galactic/setup.bash
    source <Your workspace>/install/setup.bash
    ros2 launch plansys2_ariac plansys2_ariac.launch.py
    ```

4. Terminal 3:

    ```sh
    source /opt/ros/galactic/setup.bash
    source <Your workspace>/install/setup.bash
    cd <Your workspace>/src/plansys2_ariac/pddl
    ros2 run plansys2_terminal plansys2_terminal --ros-args -p problem_file:=kitting_problem.pddl
    ```

## Contributing

Contributions to this repository are welcome and encouraged! If you have improvements, bug fixes, or new examples to add, please feel free to submit a pull request.

## License

This repository is licensed under the [Apache License 2.0](LICENSE), which means you're free to use, modify, and distribute the codebase under the terms of this license.

---

We hope this repository serves as a valuable resource for diving into task planning using PDDL 2.1 and Plansys2. If you have any questions or need assistance, feel free to open an issue or contact us.

Happy planning!
