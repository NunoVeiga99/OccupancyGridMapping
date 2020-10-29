# Occupancy Grid Mapping

## Install the simulator

Follow the steps in the following link (until line 8 of XTDrone Source Download)
- [Install drone simulator](https://www.yuque.com/xtdrone/manual_en/basic_config_1.11)


## Setup the directory:

- ``` git clone [github link] ``` 
- erase the build folder inside sa_project
- ``` cd OccupancyGridMapping/sa_project/ ```
- ``` catkin_make ```
  
Source the setup.bash file

- ``` gedit ~/.bashrc ```
- Add ``` source ~/OccupancyGridMapping/sa_project/devel/setup.bash ``` at the end of the file

## Run the code

Open a terminal and run:

- ``` roslaunch px4 ogm_mapping.launch ```
- ``` roslaunch ogm_mapping mapping.launch ```


