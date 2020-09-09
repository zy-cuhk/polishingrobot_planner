#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
 
int main(int argc,char **argv)
{
    ros::init(argc,argv,"gridCell");
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<nav_msgs::GridCells>("/gridCell1", 1);;
    ros::Publisher pub2 = nh.advertise<nav_msgs::GridCells>("/gridCell2", 1);;

    nav_msgs::GridCells cells, cells1;
 
    cells.header.frame_id="map";
    cells.cell_height=0.5;
    cells.cell_width=0.5;
    cells.cells.resize(3);
    // cells.cells[0].x=0;
    // cells.cells[0].y=0;
    // cells.cells[0].z=0;

    cells1.header.frame_id="map";
    cells1.cell_height=0.5;
    cells1.cell_width=0.5;
    cells1.cells.resize(3);
    cells1.cells[0].x=0;
    cells1.cells[0].y=0;
    cells1.cells[0].z=0;


    geometry_msgs::Point obstacle; 
    obstacle.x = 1;
    obstacle.y = 1;
    obstacle.z = 0;
    cells.cells.push_back(obstacle);

    obstacle.x = 2;
    obstacle.y = 2;
    obstacle.z = 0;
    cells.cells.push_back(obstacle);

    obstacle.x = 2;
    obstacle.y = 3;
    obstacle.z = 0;
    cells1.cells.push_back(obstacle);

    obstacle.x = 2;
    obstacle.y = 4;
    obstacle.z = 0;
    cells1.cells.push_back(obstacle);
 
    // while (ros::ok())
    while (!cells1.cells.empty())
    {
        pub1.publish(cells);
        pub2.publish(cells1);
        sleep(5);
        cells1.cells.pop_back();
    }
}
