# Furuta Pendulum ROS2 Network

This repository provides a ROS2 Humble implementation for controlling the Furuta pendulum using Xenomai. It supports both command-line operations and a Qt-based graphical interface.

## Installation

To set up the system, follow these steps:

### Clone the Repository

Make sure you have a ROS2 workspace (e.g., `ros2_ws`), then clone the necessary repositories:

```
cd ~/ros2_ws/src
git clone https://github.com/VanvitelliRobotics/uclv_xenomai_furuta_pendulum.git
git clone https://github.com/Vanvitelli-Robotics/serial.git
```

### Build the Package
After cloning, navigate to the workspace root and build the package using colcon:
```
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### Arduino Setup
Upload letturaposizione.ino to the Arduino MKR Zero using the Arduino IDE. The file is located in xenopkg/src/arduinofiles.

### Wiring Configuration

| Cable Color | Arduino MKR Zero Pin |
| ----------- | -------------------- |
| Gray        | Pin 4                |
| Black       | Pin 5                |
| White       | 3.3V (Vcc)           |
| Purple      | GND                  |

On the encoder side, make sure that the gray cable is on A, the black cable is on B, the white cable is on +, and the purple cable is on -.
## Running the Pendulum (Command-Line Mode)
### Start the EtherCAT Driver Node
To start the EtherCAT driver node, use root privileges to access the network interface. The -E flag in sudo ensures that the user's environment variables are preserved (workspace source required in the .bashrc file):

```
sudo -E bash
ros2 run xenopkg xenoethercat
```
### Launch the ROS2 Network
After starting the driver node, launch the ROS2 network with the following command:
```
ros2 launch xenopkg kalmancontroller.launch.py
```
### Start the Main Pendulum Node
Once the ROS2 network is running, start the main pendulum node by executing:

```
ros2 run xenopkg mainpendulum
```

## Running the Pendulum (Graphical Interface Mode)
For a more user-friendly experience, a Qt-based graphical interface is available.

### Launch the GUI
To start the graphical interface, run:
```
ros2 run xenopkg penduluminterface
```
### **GUI Controls**

The graphical interface provides buttons to manage the system:

- **Start MECA Driver:** Initializes the EtherCAT driver.
- **Start ROS2 Network:** Launches the serial, controller, and energy nodes.
- **Start Pendulum:** Activates the pendulum operation.
- **Stop:** Safely halts the pendulum.
- **Swing Up ON/OFF:** Enables or disables the swing-up control dynamically.

## **ROS2 Nodes Overview**

### **Driver Node**
The `driver_node` interfaces with the Meca500 robotic arm via EtherCAT. It uses Xenomai real-time threads to manage communication and:
- Publishes joint states on `/joint_states`.
- Provides services such as `/joint_position_srv` and `/set_mode_srv`.

### **Serial Node**
The `serial_node` acquires pendulum encoder data (position and velocity) from an Arduino MKR Zero and:
- Publishes the data on `/encoder_data`.
- Uses the `/set_serial_srv` service to manage real-time thread activation.

### **Controller Node**
The `controller_node` computes the control law required to stabilize the pendulum. It:
- Subscribes to `/encoder_data` and `/joint_states`.
- Publishes control commands on `/cmd_vel`.
- Provides services like `/activate_lqr_srv` and `/set_swingup_srv` for dynamic control configuration.

### **Energy Node**
The `energy_node` calculates the mechanical energy of the system and:
- Publishes the energy data on `/energy_topic` for monitoring and analysis.


