# Occupancy Grid Mapping

## Install the simulator

Follow the steps in the following link (until line 8 of XTDrone Source Download)
- [Install drone simulator](https://www.yuque.com/xtdrone/manual_en/basic_config_1.11)


## Setup the directory:

Clone the repository:
- ``` git clone https://github.com/NunoVeiga99/OccupancyGridMapping.git ```
- erase the build folder inside sa_project
- ``` cd OccupancyGridMapping/sa_project/ ```
- ``` catkin_make ```
  
Source the setup.bash file

- ``` gedit ~/.bashrc ```
- Add ``` source ~/OccupancyGridMapping/sa_project/devel/setup.bash ``` at the end of the file

## Run the code

In order to run the simulation:
  - Open QGroundControl 
  - ``` roslaunch px4 ogm_mapping.launch ```

Or run a pre recorded rosbag with:
  - ``` roscore ```
  - ``` rosbag play -l rosbag_name.sa ```

Then, run the mapping script with:
  - ``` roslaunch ogm_mapping mapping.launch ```


## How to use github

Go to the directory file with ``` cd OccupancyGrid Mapping ```.
To check changed files in the directory run ``` git status ```.
Whenever you start working with the code, make sure that you pull all changes with ``` git pull ```.

If you have made changes in the code that you want to commit run the following commands:

- ``` git add [path to files you want to add] ```
- ``` git commit -m "Message explaining the changes" ```
- ``` git push ```


