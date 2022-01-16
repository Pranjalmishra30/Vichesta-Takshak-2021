# TO DO  
1. Test on legion  DONE
2. Clean up code  DONE
3. Take pictures  
4. Document





# Team: ROSManipal-RM
Submission for Takshak-Vichesta, pan-india robotics competition organised by IIT-Dhanbad. View it [here](https://drive.google.com/drive/u/1/folders/1RB3HhomwGslC3dvN9G3or7PAeE64e1Ou)  
  

ROS-Package can be found in the [takshak](Vichesta-Takshak-2021/takshak) directory



## Executing the code locally  

### Install the package  
```  
cd ~/catkin_ws/src  
git clone https://github.com/Pranjalmishra30/Vichesta-Takshak-2021.git  
cd ..  
catkin_make  
source devel/setup.bash        
```  

### Install external packages  
sudo apt install ros-[version]-[package_name]

```  
sudo apt install ros-melodic-move-base  
sudo apt install ros-melodic-gmapping  
sudo apt install ros-melodic-dwa-local-planner  
sudo apt install ros-melodic-explore-lite  
sudo apt install ros-melodic-ros-numpy  
sudo apt install ros-melodic-map-server  
```

### Run the launch file
```
roslaunch takshak launch_all.launch 
```

## Explanation  












# Future Scope  


# References  
competition

## Resources  

## External packages used  
* [move_base](http://wiki.ros.org/move_base)     
* [gmapping](http://wiki.ros.org/gmapping)  
* [dwa_local_planner](http://wiki.ros.org/dwa_local_planner)   
* [explore_lite](https://wiki.ros.org/explore_lite)   
* [ros_numpy](https://wiki.ros.org/ros_numpy)   
* [map_server](https://wiki.ros.org/map_server)  