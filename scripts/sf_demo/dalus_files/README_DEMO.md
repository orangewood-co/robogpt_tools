# Orangewood ROS 1 Noetic Demo

Live Gaussian Splat rendering using ROS 1

Our Simulation Platform has two major components:
    - *DalusSimCoreRelease* - This library reads gaussian splats, renders them, and applies transformations to move the robot along it's joints
    - *DalusPySim*: This library interfaces with ROS, or a physics simulator like IsaacSim to get the transformation of the robot joints and any objects in the scene

# Virtual Environment
The *DalusSimCoreRelease* requires a virtual environment alongside. It already exists in this docker container, and is called `gsplat`. You can activate it using `micromamba activate gsplat`. 

To create the environment from scratch, refer to `README_environment_setup.md`.

# Running the demo
To run the ROS demo, you will need 4 terminals:
    - Terminal 1: Run the ROS master
        ```bash
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        roscore
        ```

    - Terminal 2:  Run the DalusSimCoreRelease library with
        ```bash
        source setup_env.sh
        micromamba activate gsplat
        python DalusSimCoreRelease/dalus_sim_app.py
        ```

        The expected output is a black screen that says "Waiting for simulation to start..."
    
    - Terminal 3:   Run the DalusPySim ROS 1 example with
        ```bash
        source setup_sim.sh
        python3 DalusPySim/examples/ros_noetic/tf_subscriber.py
        ```
        The expected behavior is to see the message "ROSNoeticAdapter Setup Complete." printed on the terminal
    
    - Terminal 4:   Play the rosbag
        First place the rosbag in some convenient location
        Then play the bag with:
        
        ```bash
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        rosbag play <path_to_bag>
        ```
        
        The expected behavior is that the simulation screen should stay black for a few seconds, and then the simulation should load up.

# Troubleshooting
As you're developing the application, if you would like to inspect the Gaussian Splat, you can turn the debug flag on line 19 of `DalusSimCoreRelease/dalus_sim_app.py`.

This will open a web-based renderer on port 8000 of your browser which will allow you to interactively view the 3D environment.