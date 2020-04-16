# mechknownet

Dependencies
====
PCL 1.7 or 1.8
OpenCV 2 or 3 (make sure your opencv version is the same with the cv_brige package, otherwise it will go wrong, to build cv_brige under opencv3 you can compile the cv_bridge package from source code)

Compile
====
catkin bt under ros environment


Usage
====
download the trained mrcnn weights from https://drive.google.com/file/d/1R5yu8HNPjJdUErYi6YSLeGepDkDeZhQ-/view?usp=sharing then put it on mechknownet/script/mask_rcnn/

or you can train your mrcnn model using the tool in https://github.com/himlen1990/toolbox 

roslaunch mechknownet mechknownet_server.launch

rosrun mechknownet system

python check_output_without_execute.py -> if nothing goes wrong, you should see the trajectory and 
tf output after the detection process finished

To drive the HSR robot, please see the code in pound.py or support.py or pour_demo.py

Data Flow inside system.cpp
====
![image](https://github.com/himlen1990/mechknownet/blob/master/IMG/1.png)