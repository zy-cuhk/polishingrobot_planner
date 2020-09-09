// http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
// The LINE_STRIP type uses each point as a vertex in a connected set of lines, where point 0 is connected to point 1, 1 to 2, 2 to 3, etc. The LINE_LIST type creates unconnected lines out of each pair of points, i.e. point 0 to 1, 2 to 3, etc. 
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
 
#include <cmath>
 
int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
 
  ros::Rate r(30);
 
  float f = 0.0;
  while (ros::ok())
  {
 
    visualization_msgs::Marker points, line_strip, line_list_x,line_list_y, line_list_z;
    points.header.frame_id = line_strip.header.frame_id = line_list_x.header.frame_id = line_list_y.header.frame_id =line_list_z.header.frame_id  = "/base_link";
    points.header.stamp = line_strip.header.stamp = line_list_x.header.stamp= line_list_y.header.stamp = line_list_z.header.stamp= ros::Time::now();
    points.ns = line_strip.ns = line_list_x.ns= line_list_y.ns= line_list_z.ns = "points_and_lines";
    points.action = line_strip.action = line_list_x.action = line_list_y.action =line_list_z.action =visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list_x.pose.orientation.w =line_list_y.pose.orientation.w =line_list_z.pose.orientation.w = 1.0;
 
 
 
    points.id = 0;
    line_strip.id = 1;
    line_list_x.id = 2;
    line_list_y.id = 3;
    line_list_z.id = 4;
 
 
 
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list_x.type = visualization_msgs::Marker::LINE_LIST;
    line_list_y.type = visualization_msgs::Marker::LINE_LIST;
    line_list_z.type = visualization_msgs::Marker::LINE_LIST;
 
 
 
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
 
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list_x.scale.x = 0.1;
    line_list_y.scale.x = 0.1;
    line_list_z.scale.x = 0.1;
 
 
 
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
 
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
 
    // Line list is red
    line_list_x.color.r = 1.0;
    line_list_x.color.a = 1.0;
    line_list_y.color.g = 1.0;
    line_list_y.color.a = 1.0;
    line_list_z.color.b = 1.0;
    line_list_z.color.a = 1.0;
 
 
 
    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
 
      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;
 
      points.points.push_back(p);
//      line_strip.points.push_back(p);
 
      // The line list needs two points for each line
 
 
      line_list_z.points.push_back(p);
      p.z += 1.0;
      line_list_z.points.push_back(p);
 
      p.z -= 1.0;
      line_list_y.points.push_back(p);
      p.y += 1.0;
      line_list_y.points.push_back(p);
 
      p.y -= 1.0;
      line_list_x.points.push_back(p);
      p.x += 1.0;
      line_list_x.points.push_back(p);
    }
 
 
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list_x);
    marker_pub.publish(line_list_y);
    marker_pub.publish(line_list_z);
 
    r.sleep();
 
    //f += 0.04;
  }

}