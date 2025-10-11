# SETUP DIGITAL TWIN ROBOTIC CELL
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
- Turn On the PC
- Select Ubuntu as OS
- Select acis user and eneter the password
- Open Isaac Sim:
  - Open a new terminal (CTRL + Alt + T)
  - Copy and paste: /home/acis/isaacsim/_build/linux-x86_64/release/isaac-sim.sh
  - Click enter and let it run
  - A Pop up Window like this should appear: 
<img width="443" height="224" alt="image" src="https://github.com/user-attachments/assets/b30262b0-fdf5-4344-88b8-f5197decbfb7" />

  - It is usual that a pop-up window of "Isaac Sim is not responding" appear, if so, Wait, or click "Wait"
<img width="444" height="166" alt="image" src="https://github.com/user-attachments/assets/841c769a-8448-46b9-a9cd-07a2575b23e3" />

  - If you see something like this, Isaac Sim is completely loaded:
<img width="444" height="166" alt="image" src="https://github.com/user-attachments/assets/a2bad767-fd72-4fe6-8492-a589325ce773" />

- Open the robotic cell scene:
  - At the left top of the Isaac Sim window, Click on File
  - Go to Open Recent
  - Click on file:/home/acis/Downloads/diego/dt-isaac-sim/new-robotic-cell-dq.usd
<img width="532" height="216" alt="image" src="https://github.com/user-attachments/assets/1bf5b162-51ed-48c3-acfa-8c3d183ac1ad" />

  - In the pop-up window, Click on "Don't save"
  - You have now successfully opened the DT of the Robotic Cell!
- Enable ROS Topics for simulated robots
  - Press space bar
  - You can verify that the topics are enabled by opening a cmd and running "ros2 topic list"
    - The names of the topics that you have to look for are:
      - /uf850_sim/joint_command
      - /uf850_sim/joint_states
      - /xarm6_sim/joint_command
      - /xarm6_sim/joint_states
    - The joint_command topic is the topic that receives the commands, and the joint_states is the topic that publishes the states
- Enable ROS Topics for real robots

