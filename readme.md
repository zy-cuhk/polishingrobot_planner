1. step 1: 
mobile platform planning using matlab 
(1) execute main2.m; 
(2) execute main3.m 
the output is: 
(1) mobile platform positions list
(2) two points position representing the covered rectangular area


2. step 2: viewpoint planning using c++
(1) obtian pcd file 
(2) read pcd file
(3) transfer into octomap format
(4) sample multiple point and obtain colision-free viewpoints and obtain json file 
(5) the loop is: obtain the viewpoints with maximum coverage area
(6) remove and update operation
(7) exit condition: all are covered


3. publish planned robot state, including mobile platform position and manipulator joints list
the refrence code is: 











