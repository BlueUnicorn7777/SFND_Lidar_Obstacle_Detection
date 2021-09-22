# Sensor Fusion Self-Driving Car Course


## Dependencies


* Ubuntu 16.04
* PCL - v1.7.2
* C++ v11
* gcc v5.5
* sudo apt install libpcl-dev


## Clone and Build

1. Clone this github repo:
 
   `git clone https://github.com/BlueUnicorn7777/SFND_Lidar_Obstacle_Detection.git`

2. Execute the following commands in a terminal
 
   `cd ~/SFND_Lidar_Obstacle_Detection`   
   `mkdir build && cd build`    
   `cmake ..`    
   `make`    
   
## To Run  
   
`./environment highway - `      
This command will run the highway functions from classroom tutorial.
      
`./environment cityblock_pcl filename ` 
This command run the builtin in pcl library functions from classroom tutorial for segmentation and clustering. The filename specifies the path to the pcd files.    

eg `./environment cityblock_pcl ../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1`
    
`./environment cityblock filename`    
This command run functions from SFND.cpp and SFND.h files for segmentation and clustering. This file contains the 3D rendering and kdtree student functions required by rubric points.
The filename specifies the path to the pcd files.    
eg `./environment cityblock ../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1`


