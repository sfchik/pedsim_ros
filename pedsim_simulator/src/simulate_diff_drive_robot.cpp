#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double g_updateRate, g_simulationFactor;
double v_current, omega_current;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
boost::mutex mutex;
ros::Publisher twistPublisher;
/// Simulates robot motion of a differential-drive robot with translational and rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world --> base_footprint frame.
void updateLoop()
{
    ros::Rate rate(g_updateRate);
    double dt = g_simulationFactor / g_updateRate;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion quat;   

    while (true) {
        // Get current pose
        double x = g_currentPose.getOrigin().x();
        double y = g_currentPose.getOrigin().y();
        double theta = tf::getYaw(g_currentPose.getRotation());

        // Get requested translational and rotational velocity
        double v_cmd, omega_cmd;
        {
            boost::mutex::scoped_lock lock(mutex);
            v_cmd = g_currentTwist.linear.x;
            omega_cmd = g_currentTwist.angular.z;
        }       

//         Simulate robot movement
         if(v_cmd > v_current)
             v_current = std::min(v_cmd, v_current + 1.0*dt);
         else if(v_cmd < v_current)
             v_current = std::max(v_cmd, v_current - 1.0*dt);        
         else
            v_current = v_cmd;

         if(omega_cmd > omega_current)
             omega_current = std::min(omega_cmd, omega_current + 2.0*dt);
         else if(v_cmd < v_current)
             omega_current = std::max(omega_cmd, omega_current - 2.0*dt);        
         else
            omega_current = omega_cmd;

        x += cos(theta) * v_current * dt;
        y += sin(theta) * v_current * dt;
        theta += omega_current * dt;

        // Update pose
        g_currentPose.getOrigin().setX(x);
        g_currentPose.getOrigin().setY(y);
        g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));

        // Broadcast transform
        g_transformBroadcaster->sendTransform(tf::StampedTransform(g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));
        // Publish odom
        // quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta);
        // odom.header.frame_id = g_worldFrame;
        // odom.header.stamp = ros::Time::now();
        // odom.child_frame_id = g_robotFrame;
        // odom.pose.pose.position.x = x;
        // odom.pose.pose.position.y = y; 
        // odom.pose.pose.orientation = quat;
        // odom.twist.twist.linear.x = v_current;
        // odom.twist.twist.angular.z = omega_current;
        // twistPublisher.publish(odom);

        rate.sleep();
    }
}

void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist)
{
    boost::mutex::scoped_lock lock(mutex);
    g_currentTwist = *twist;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulate_diff_drive_robot");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Process parameters
    privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
    privateHandle.param<std::string>("robot_frame", g_robotFrame, "base_footprint");

    privateHandle.param<double>("simulation_factor", g_simulationFactor, 1.0); // set to e.g. 2.0 for 2x speed
    privateHandle.param<double>("update_rate", g_updateRate, 25.0); // in Hz

    double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
    privateHandle.param<double>("pose_initial_x", initialX, 0.0);
    privateHandle.param<double>("pose_initial_y", initialY, 0.0);
    privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);

    g_currentPose.getOrigin().setX(initialX);
    g_currentPose.getOrigin().setY(initialY);
    g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

    // Create ROS subscriber and TF broadcaster
    g_transformBroadcaster.reset(new tf::TransformBroadcaster());
    ros::Subscriber twistSubscriber = nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 3, onTwistReceived);
    twistPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/robot_twist/direct", 10);

    v_current = 0.0;
    omega_current = 0.0;

    // Run
    boost::thread updateThread(updateLoop);
    ros::spin();
}
