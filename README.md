# General description
You will need to create a system that reads data from Ethernet sensors and logs the data into a CSV file.

Some of the work is made for you, your tasks is to finish the implementation.

# Setup
1. Install ROS2 Foxy (https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
2. Setup ROS2 environment in terminal: `source /opt/ros/foxy/setup.bash`
3. Build: `./build.sh -c`

# Run
1. Setup environment: `source install/setup.bash`
2. Run sensor simulator: `./sensor_sender.py`
3. Run nodes: `ros2 launch app_launch.py`

# Tasks
* General
  * Solution must work on Ubuntu 20.04 and ROS2 Foxy
  * Fix all issues in the code
  * Fix TODOs
  * Describe the application in `DOCUMENTATION.md`
  * You are not required to implement tests
  * You must use git throughout the development and share the solution as a link to git repository
  * The expected solution should not take more than 6h to implement. Add comments for functionality that could be added, but you deem out of scope
* UDP node
  * Python script emulates set of sensors with ethernet connection, each sensor has ID and reading
  * Read and parse sensor data from Python script `sensor_sender.py`
  * Sensors must be pinged at least once every 5 seconds with string `ping`, otherwise the data will stop
* Logger node
  * Logger must save both sensor ID and reading using optimal data type
  * Logger must save all data that sensors publish
* `sensor_sender.py`
  * You can use this to test your code
  * `sensor_sender.py` sends on port `12345` and listens on `12346` on `localhost`
  * Data rate from sensors varies: 1 - 100 Hz
  * Data from up to 100 sensors is sent
* __NB!: You are the owner and maintainer of the whole project. If you find something else to fix besides TODO-s, do it__