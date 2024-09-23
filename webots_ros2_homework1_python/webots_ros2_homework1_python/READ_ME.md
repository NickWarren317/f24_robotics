<h1> How to run the simulation </h1>

<ol>
    <li>Edit the f24_robotics_1.wbt file to select your desired bot start position and orientation</li>
    <li>Navigate to the f24robotics directory and type "colcon build" in the terminal</li>
    <li>From here source the build with "source install/setup.bash" </li>
    <li>Then run the ros2 script with "ros2 run webots_ros2_homework1_python webots_ros2_homework1_python"</li>
    <li>Open another terminal and source the same script from before, then run the simulation with ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py </li>
    <li> Once the simulation is running, you can Ctrl + C in the terminal running the controller to terminate the simulationi</li>
    <li> As the controller runs, it logs its location every 5 seconds. This data is then printed to a file called "coords.txt" in the
    directory </li>
    <li> Finally, once can copy it into a new file with the name format of spawnXtrialX.txt and run the render_path.py with the file in the same directory as it, selecting the number of total trials per the spawn number in the file name. Run the file and the graph and some data will be outputted</li>
</ol>