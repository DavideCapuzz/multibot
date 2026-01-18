# Multibot Simulation

Top-level folder for multi-robot simulation using ROS2, Docker, and Gazebo.

---

## Quick Start

### Initial Setup

1. **Clone the repository**
   ```bash
   git clone git@github.com:DavideCapuzz/multibot.git
   cd multibot
   ```

2. **Initialize submodules**
   ```bash
   git submodule update --init --recursive
   ```

3. **Open in development container**
   - Open the folder in your IDE (VSCode/CLion)
   - Use the provided `devcontainer.json` to start the development container

---

## Running Your First Simulation

Once inside the container:

1. **Navigate to the workspace**
   ```bash
   cd ~/ros2_ws/
   ```

2. **Build the workspace**
   ```bash
   colcon build
   ```

3. **Source the environment**
   ```bash
   source install/setup.bash
   ```

4. **Launch the simulation**
   ```bash
   ros2 launch launch/gz_base.py
   ```

---

## Accessing Container UIs

Each container runs a VNC server accessible through your web browser. The ports are automatically assigned based on the number of containers:

| Container | URL |
|-----------|-----|
| Container 1 | http://localhost:6080/ |
| Container 2 | http://localhost:6081/ |
| Container 3 | http://localhost:6082/ |
| Container 4 | http://localhost:6083/ |
| ... | ... |

The pattern continues incrementally for each additional container you add.

---

## Submodules

This project uses the following submodules:

| Submodule | Description | Repository |
|-----------|-------------|------------|
| **ros2docker** | ROS2-Docker integration and container management | [github.com/DavideCapuzz/ros2docker](https://github.com/DavideCapuzz/ros2docker) |
| **gazebo_common** | Common Gazebo setups and configurations | [github.com/DavideCapuzz/gazebo_common](https://github.com/DavideCapuzz/gazebo_common) |


---

## Configuration

Configure your multi-robot setup by editing `.devcontainer/setup.ini`. See the [ros2docker documentation](https://github.com/DavideCapuzz/ros2docker) for detailed configuration options.

---

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

---

## Related Projects

- [ros2docker](https://github.com/DavideCapuzz/ros2docker) - Docker container management for ROS2
- [gazebo_common](https://github.com/DavideCapuzz/gazebo_common) - Gazebo simulation configurations

---

## Troubleshooting

### Container won't start
- Ensure Docker is running
- Check that all submodules are initialized: `git submodule status`

### Can't access VNC viewer
- Verify the container is running: `docker ps`
- Check that the port isn't already in use
- Try accessing with the correct port number (6080, 6081, etc.)

### Build errors
- Make sure you've sourced the ROS2 environment: `source install/setup.bash`
- Try cleaning the build: `rm -rf build/ install/ log/` then rebuild
