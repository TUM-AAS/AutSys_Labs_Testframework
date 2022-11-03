#ifndef TRAJECTORY_TEST
#define TRAJECTORY_TEST

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <vector>

class Trajectory_test  {
public:
    Trajectory_test(ros::NodeHandle nh_);
    void checkRunTime(const ros::TimerEvent& t);
    void checkRotorSpeeds(const mav_msgs::Actuators& rotor_speeds);
    void checkSentTrajectory (const mav_planning_msgs::PolynomialTrajectory4D& msg);
    void checkDronePosition(const nav_msgs::Odometry& cur_state);
    void writeTestResult();

    
private:
    void readReferencePositions();
    void checkGoalPositions(Eigen::Vector3d dronePosition);

    ros::NodeHandle nh;
    ros::Time startup_time;
    ros::Subscriber sub_dronePosition;
    ros::Subscriber sub_trajectory;
    ros::Subscriber sub_rotorSpeedCmds;
    ros::Timer t_checkRunTime;

    std::vector<Eigen::Vector3d> referencePositions;
    std::vector<unsigned int> reachedPositionNTimes;
    std::vector<unsigned int> positionSequence;
    std::vector<bool> droneAtGoalPosition;

    int numberCalls;
    unsigned long numberCallsPropSpeeds;
    Eigen::Vector4d minWrench;
    Eigen::Vector4d maxWrench;

    std::vector<Eigen::Vector4d> stopPositions;
    bool receivedTrajectory;

    double flightDuration;
};

#endif