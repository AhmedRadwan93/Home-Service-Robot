#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

const int max_wait_goal = 10;
const int DEBUGodom = 350;
const float dist_thres = 0.6;
const bool DEBUG = false;
const bool use_goal_pub = false;

class add_markers
{
public:
  add_markers();

  enum order_of_goal {
    pickup_goal = 1,
    drop_goal = 2,
    undefined_goal= 0
  };
  
  enum marker_status {
    marker_start = 0,
    marker_go_pick= 1,
    marker_pick = 2,
    marker_carry = 3,
    marker_go_drop  = 4,
    marker_drop = 5,
    marker_undefined= 6
 };
 
private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber goal_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber mb_sub;
  marker_status status;

  marker_status newstatus;
  order_of_goal goal_order;
  bool subscriber_exist;
  geometry_msgs::Pose goal;
  geometry_msgs::Pose odom;
  visualization_msgs::Marker marker;

  int logc;
  int wait_goal_pub;
  ros::Time pick_time;

  bool close_enough(const geometry_msgs::Pose &pos, const geometry_msgs::Pose &target);
  void goalCallback(const geometry_msgs::Pose &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void update();
};

add_markers::add_markers() {
  logc = 0;
  status = marker_start;
  goal_order = undefined_goal;
  subscriber_exist = false;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  goal_sub = n.subscribe("/target",1,&add_markers::goalCallback,this);
  odom_sub = n.subscribe("/odom",1,&add_markers::odomCallback,this);


  uint32_t shape = visualization_msgs::Marker::CUBE;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;

  
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;




  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;


  marker.color.r = 0.3f;
  marker.color.g = 0.5f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;
}


bool add_markers::close_enough(const geometry_msgs::Pose &pos, const geometry_msgs::Pose &target) {
  float dx = pos.position.x - target.position.x;
  float dy = pos.position.y - target.position.y;
  float dz = pos.position.z - target.position.z;
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);
  if (DEBUG && (logc % DEBUGodom == 0)) {
    ROS_INFO("d:%f, t:%f, s:%d, n:%d, g:%d, c:%d, x:%f vs %f, y:%f vs %f, z:%f vs %f s0:%d, s1:%d, s2:%d, s3:%d, s4:%d, s5:%d",
      dist, dist_thres,
      status, newstatus, goal_order,
      (dist < dist_thres),
      pos.position.x, target.position.x,
      pos.position.y, target.position.y,
      pos.position.z, target.position.z,
      status == marker_start,
      status == marker_go_pick,
      status == marker_pick,
      status == marker_carry,
      status == marker_go_drop,
      status == marker_drop);
  }
  return (dist < dist_thres);
}

void add_markers::goalCallback(const geometry_msgs::Pose &msg) {
  if (!use_goal_pub) {
    return;
  }
  if (DEBUG) ROS_INFO("Goal received");
  goal.position.x = msg.position.x;
  goal.position.y = msg.position.y;
  goal.position.z = msg.position.z;
  goal.orientation.x = msg.orientation.x;
  goal.orientation.y = msg.orientation.y;
  goal.orientation.z = msg.orientation.z;
  goal.orientation.w = msg.orientation.w;
  if (goal_order == undefined_goal) goal_order = pickup_goal;
  else if (goal_order == pickup_goal) {
    goal_order = drop_goal;
    ROS_INFO("Carry the object to the drop off point");
  }
  if (DEBUG) ROS_INFO("Goal received goal_order:%d, x:%f, y:%f, w:%f",
    goal_order, msg.position.x, msg.position.y, msg.orientation.w);
  if (DEBUG) ROS_INFO("Goal received x:%f vs %f, y:%f vs %f, z:%f vs %f",
    msg.position.x,
    goal.position.x,
    msg.position.y,
    goal.position.y,
    msg.orientation.z,
    goal.position.z
    );
  this->update();
}

void add_markers::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom.position.x = msg->pose.pose.position.x;
  odom.position.y = msg->pose.pose.position.y;
  odom.position.z = msg->pose.pose.position.z;
  odom.orientation.x = msg->pose.pose.orientation.x;
  odom.orientation.y = msg->pose.pose.orientation.y;
  odom.orientation.z = msg->pose.pose.orientation.z;
  odom.orientation.w = msg->pose.pose.orientation.w;

  bool close = close_enough(odom, goal);
  newstatus = marker_undefined;
 
  if (status == marker_go_pick && goal_order == pickup_goal && close) {
    newstatus = marker_pick;
    if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
  }
  else if (status == marker_carry && goal_order == drop_goal && close) {
    newstatus = marker_go_drop;
    if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
  }
  logc++;
  if (newstatus != marker_undefined) status = newstatus;
  this->update();
}

void add_markers::update() {
    if (!subscriber_exist) {
      if (marker_pub.getNumSubscribers() > 0) {
        subscriber_exist = true;
      }
      else {
        if (!ros::ok())
        {
          return;
        }
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
    }
    bool wait_for_goal_pub = false;
    if (goal_order == undefined_goal && wait_goal_pub <= max_wait_goal) {
      ROS_WARN("Waiting for a goal location %d / %d", wait_goal_pub, max_wait_goal);
      wait_goal_pub++;
      wait_for_goal_pub = true;
      ros::Duration(0.5).sleep();
    }
    if (subscriber_exist && !wait_for_goal_pub) {
      if (status == marker_pick) {
        marker.action = visualization_msgs::Marker::DELETE;
        ROS_INFO("Robot picks up the object");
        newstatus = marker_carry;
        if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
        status = newstatus;
        pick_time = ros::Time::now();
      }
      else if (status == marker_start || status == marker_go_drop) {

        marker.action = visualization_msgs::Marker::ADD;

        if (status == marker_start) {
          if(goal_order != pickup_goal) {
            ROS_INFO("Goal publisher doesn't work, update manually pickup goal_order:%d", goal_order);
            goal_order = pickup_goal;
            goal.position.x = 4.0;
            goal.position.y = 6.0;
            goal.orientation.w = -0.5;
          }


          marker.pose.position.x = goal.position.x;
          marker.pose.position.y = goal.position.y;
          marker.pose.orientation.w = goal.orientation.w;
          newstatus = marker_go_pick;
          if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
          status = newstatus;
         }

         else if (status == marker_go_drop) {
          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x =  goal.position.x;
          marker.pose.position.y = goal.position.y;
          marker.pose.orientation.w =  goal.orientation.w;
          newstatus = marker_drop;
          ROS_INFO("Drop the object at the drop off point");
          if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
          status = newstatus;
         }
         else {
          ROS_WARN("ERROR on status:%d or goal_order%d", status, goal_order);
         }
      }
      else if (status == marker_carry && goal_order != drop_goal){
        ros::Duration t_from_pick = ros::Time::now() - pick_time;
        if(t_from_pick >= ros::Duration(5.0)) {
          ROS_INFO("Goal publisher doesn't work, update manually drop off goal_order:%d", goal_order);
          goal_order = drop_goal;
          goal.position.x = -4.0;
          goal.position.y = 6.0;
          goal.orientation.w = -0.5;
          ROS_INFO("Carry the object to the drop off point");
        }
      }


      marker.lifetime = ros::Duration();

      marker_pub.publish(marker);
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  add_markers add_markers;
  ros::Rate r(1);
  ros::spin();

  return 0;
}
