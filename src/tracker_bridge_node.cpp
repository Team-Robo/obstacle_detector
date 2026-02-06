#include <ros/ros.h>
#include <vector>

#include <obstacle_detector/Obstacles.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Point32.h>

class TrackerBridge
{
public:
  TrackerBridge() : nh_("")
  {
    sub_ = nh_.subscribe("obstacles", 1, &TrackerBridge::callback, this);
    pub_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/MpcLocalPlannerROS/obstacles", 1);
    ROS_INFO("Tracker Bridge Initialized: Bridging /obstacles -> /move_base/MpcLocalPlannerROS/obstacles");
  }

  void callback(const obstacle_detector::Obstacles::ConstPtr& msg)
  {
    costmap_converter::ObstacleArrayMsg out_msg;
    out_msg.header = msg->header;

    // --- Convert Circles ---
    for (const auto& circle : msg->circles)
    {
      costmap_converter::ObstacleMsg obs;
      obs.header = msg->header;

      // Center Point
      geometry_msgs::Point32 p;
      p.x = static_cast<float>(circle.center.x);
      p.y = static_cast<float>(circle.center.y);
      p.z = 0.0;
      obs.polygon.points.push_back(p);

      // Radius & Velocity
      obs.radius = circle.radius;
      obs.velocities.twist.linear = circle.velocity; 
      
      // Identical orientation
      obs.orientation.w = 1.0; 

      out_msg.obstacles.push_back(obs);
    }

    // --- Convert Segments ---
    for (const auto& segment : msg->segments)
    {
      costmap_converter::ObstacleMsg obs;
      obs.header = msg->header;

      // Point 1
      geometry_msgs::Point32 p1;
      p1.x = static_cast<float>(segment.first_point.x);
      p1.y = static_cast<float>(segment.first_point.y);
      p1.z = 0.0;
      obs.polygon.points.push_back(p1);

      // Point 2
      geometry_msgs::Point32 p2;
      p2.x = static_cast<float>(segment.last_point.x);
      p2.y = static_cast<float>(segment.last_point.y);
      p2.z = 0.0;
      obs.polygon.points.push_back(p2);

      // Identical orientation
      obs.orientation.w = 1.0;

      out_msg.obstacles.push_back(obs);
    }

    pub_.publish(out_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracker_bridge_node");
  TrackerBridge bridge;
  ros::spin();
  return 0;
}