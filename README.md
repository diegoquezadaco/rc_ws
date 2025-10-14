# SETUP DIGITAL TWIN ROBOTIC CELL
**PLEASE MAKE SURE TO READ ALL THE SECTION CONTENT BEFORE IMPLEMENTING ANYTHING**
# Table of contents
[0. Requirements](#0-requirements).
[1. Physical Setup](#1-physical-setup).
[2. Computer Setup](#2-computer-setup).
  [2.5 Set up the Digital Twin](#25-set-up-the-digital-twin).


## 0. Requirements
- Computer in the Robotic Cell (AMD® Ryzen 9 5900x 12-core processor × 24 with TITAN RTX)
- Robotic Arm UF850 with XArm gripper
- Robotic Arm XArm6 with XArm gripper on a UFactory Linear Motor
- 3 LAN Cables
- Switch or Access point
## 1. Physical Setup
- Verify that both AC Control boxes of the robots are plugged
- Turn on the switches in both AC control boxes so they look like this:
<img width="493" height="254" alt="image" src="https://github.com/user-attachments/assets/92c21ece-12df-430d-88c2-4ad7d605c24d" />
<img width="432" height="453" alt="image" src="https://github.com/user-attachments/assets/b11c1d49-e96f-45ef-95ee-a4a918483abe" />

- Release the Emergency Stop button (Red one) on both AC control Boxes
- Verify that the LAN cables between the AC Control Boxes and the Access Point are connected
- Verify that the LAN cable between the Access Point 
## 2. Computer Setup

### 2.1 Open Isaac Sim:
  - Open a new terminal (CTRL + Alt + T)
  - Copy and paste:
      ```bash
      /home/acis/isaacsim/_build/linux-x86_64/release/isaac-sim.sh
  - Click enter and let it run
  - A Pop up Window like this should appear: 
<img width="443" height="224" alt="image" src="https://github.com/user-attachments/assets/b30262b0-fdf5-4344-88b8-f5197decbfb7" />

  - It is usual that a pop-up window of "Isaac Sim is not responding" appear, if so, Wait, or click "Wait"
<img width="444" height="166" alt="image" src="https://github.com/user-attachments/assets/841c769a-8448-46b9-a9cd-07a2575b23e3" />

  - If you see something like this, Isaac Sim is completely loaded:
<img width="444" height="166" alt="image" src="https://github.com/user-attachments/assets/a2bad767-fd72-4fe6-8492-a589325ce773" />

### 2.2 Open the robotic cell scene:
  - At the left top of the Isaac Sim window, Click on File
  - Go to Open Recent
  - Click on file:/home/acis/Downloads/diego/dt-isaac-sim/new-robotic-cell-dq.usd
<img width="532" height="216" alt="image" src="https://github.com/user-attachments/assets/1bf5b162-51ed-48c3-acfa-8c3d183ac1ad" />

  - In the pop-up window, Click on "Don't save"
<img width="457" height="246" alt="image" src="https://github.com/user-attachments/assets/fe4110ee-6af1-4d2d-a6a2-ff231ffb20e4" />

  - You have now successfully opened the DT of the Robotic Cell!
<img width="533" height="298" alt="image" src="https://github.com/user-attachments/assets/46af71c9-2cb2-43d1-a870-f1b3da0c026d" />

### 2.3 Enable ROS Topics for simulated robots
  - Press space bar
<img width="173" height="399" alt="image" src="https://github.com/user-attachments/assets/01a02439-42bd-4c29-b903-951806547218" />

- You can verify that the topics are enabled by:
    - Open a new terminal (CTRL + Alt + T)
      ```bash
      ros2 topic list
- The names of the topics that you have to look for are:
  - /uf850_sim/joint_command
  - /uf850_sim/joint_states
  - /xarm6_sim/joint_command
  - /xarm6_sim/joint_states
- The joint_command topic is the topic that receives the commands, and the joint_states is the topic that publishes the states
### 2.4 Enable ROS Topics for real robots
  - Open a new terminal (CTRL + Alt + T)
  - Copy and paste the following commands:
    ```bash
    cd scan_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch xarm_moveit_config dual_xarm6_moveit_realmove.launch.py robot_ip_1:=192.168.0.211 robot_ip_2:=192.168.0.239 [add_gripper:=true]
  - You will hear the robots activate, and see this pop-up window of RVIZ
<img width="1523" height="855" alt="image" src="https://github.com/user-attachments/assets/7917cb85-e16d-4248-89ee-6260420a341f" />

- You can verify that the topics are enabled by:
  -  Open a new terminal (CTRL + Alt + T)
        ```bash
            ros2 topic list
  - The names of the topics that you have to look for are:
    - /L_uf850_traj_controller/controller_state
    - /L_uf850_traj_controller/joint_trajectory
    - /L_uf850_traj_controller/state
    - /L_uf850_traj_controller/transition_event
    - /L_ufactory/joint_states
    - /L_ufactory/robot_states
    - /L_ufactory/uf_ftsensor_ext_states
    - /L_ufactory/uf_ftsensor_raw_states
    - /L_ufactory/vc_set_cartesian_velocity
    - /L_ufactory/vc_set_joint_velocity
    - /L_ufactory/xarm_cgpio_states
    - /R_ufactory/joint_states
    - /R_ufactory/robot_states
    - /R_ufactory/uf_ftsensor_ext_states
    - /R_ufactory/uf_ftsensor_raw_states
    - /R_ufactory/vc_set_cartesian_velocity
    - /R_ufactory/vc_set_joint_velocity
    - /R_ufactory/xarm_cgpio_states
    - /R_xarm6_traj_controller/controller_state
    - /R_xarm6_traj_controller/joint_trajectory
    - /R_xarm6_traj_controller/state
    - /R_xarm6_traj_controller/transition_event
### 2.5 Set up the Digital Twin
 - Open a new terminal (CTRL + Alt + T)
  - Copy and paste the following commands:
    ```bash
    cd robotic_cell_files/rc_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ros2 run robotic_cell setup_digital_twin
  - In that terminal, you should see something like this that updates really fast:
<img width="1204" height="378" alt="image" src="https://github.com/user-attachments/assets/978113c5-3e08-4ac7-9f0b-d3458e748877" />

  - You should see the Simulated robots in Isaac Sim, mimic the pose of the real ones

## 3. Interacting with the Digital Twin in Isaac Sim

### 3.1 Interacting with the Digital Twin in Isaac Sim

### 3.2 Sending Commands to the Real Robot Using RVIZ interface

### 3.3 Sending commands to the Real Robot using command line 




    


