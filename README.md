Here’s the revised README that incorporates both your existing content and the details about configuring the joystick and launching the `joy` node:

---

# README: Using Xbox Controller with ROS (Noetic)

## Overview
This guide explains how to set up and use an Xbox controller with ROS Noetic to control a robot using the `joy` package. It includes steps to:

1. Install and configure the `joy` package.
2. Connect and test the Xbox controller.
3. Launch the `joy` node (via the provided launch file).

---

## Prerequisites

### Requirements
- **Operating System**: Ubuntu 20.04 (64-bit)
- **ROS Distribution**: ROS Noetic
- **Controller**: Xbox Controller (USB/Bluetooth connection)

---

## Step 1: Install the Joy Package

The `joy` package provides a ROS interface for joystick devices. Install it using the following command:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-joy
```

---

## Step 2: Connect Your Xbox Controller

### USB Connection
- Plug in the Xbox controller using a USB cable.

### Bluetooth Connection
1. Turn on Bluetooth on your computer.
2. Put the Xbox controller in pairing mode (press and hold the **Xbox button** until it blinks).
3. Pair the controller via your system’s Bluetooth settings.

Verify the connection by listing the available joystick devices:

```bash
ls /dev/input/js*
```

You should see something like `/dev/input/js0`.

---

## Step 3: Test the Joystick

Install the `jstest` utility to verify joystick functionality:

```bash
sudo apt-get install joystick
```

Run the `jstest` tool to test the controller:

```bash
sudo jstest /dev/input/js0
```

Move the controller sticks and press buttons to ensure proper functionality. You should see live data updates.

If you encounter permission issues, update the device permissions:

```bash
sudo chmod a+rw /dev/input/js0
```

---

## Step 4: Configure and Launch the Joy Node

The `joy` node is responsible for publishing joystick inputs to ROS topics. The node is already included in the launch file provided with this package.

1. **Verify Joystick Recognition:**
   List all input devices:
   ```bash
   ls /dev/input/
   ```
   Look for a device like `js0` in the output.

2. **Check Joystick Permissions:**
   Run the following command to verify permissions:
   ```bash
   ls -l /dev/input/js0
   ```
   You should see an output like:
   ```
   crw-rw-rw- 1 root dialout 188, 0 <timestamp> /dev/input/js0
   ```
   If permissions are not configured (`rw` is missing), run:
   ```bash
   sudo chmod a+rw /dev/input/js0
   ```

3. **Launch the Joy Node:**
   The `joy` node is launched via the package's provided launch file:
   ```bash
   roslaunch jaco_xbox_controller xbox_controller.launch
   ```

---

## Step 5: Troubleshooting

### Joy Node Not Publishing Data
- Verify the connection:
  ```bash
  ls /dev/input/js*
  ```
- Check the ROS topic:
  ```bash
  rostopic echo /joy
  ```
- Restart the joy node if necessary.

### Xbox Controller Not Recognized
- Ensure the controller is properly paired (for Bluetooth).
- Install Xbox driver support:
  ```bash
  sudo apt-get install xboxdrv
  ```
- Restart your computer and try again.

---

## Additional Notes
- The `joy` node must be running before launching your application.
- For advanced configurations, refer to the [ROS Joy Documentation](http://wiki.ros.org/joy).

---

Happy coding!
