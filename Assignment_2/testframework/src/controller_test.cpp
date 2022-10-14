#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <fstream>

class Controller_test  {
public:
    Controller_test(ros::NodeHandle nh_) : nh(nh_), maxDeviation(0.0), averageDeviation(0.0),
                numberCalls(0), numberDeviationOutsideThreshold(0), referencePosition(0.0,0.0,0.0) {
        sub_dronePosition = nh.subscribe("current_state", 5, &Controller_test::checkDronePosition, this);
        sub_desPosition = nh.subscribe("desired_state", 5, &Controller_test::saveReferencePosition, this);
        startup_time = ros::Time::now();
        
        t_checkRunTime = nh.createTimer(ros::Duration(0.1), &Controller_test::checkRunTime, this);
        t_checkRunTime.start();
       
    }

    void checkRunTime(const ros::TimerEvent& t) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        if(time > 50.0) {
            writeTestResult();
            ros::shutdown();
        }
    }

    void saveReferencePosition (const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state) {
        geometry_msgs::Vector3 t = des_state.transforms[0].translation;
        referencePosition << t.x, t.y, t.z;
    }

    void checkDronePosition(const nav_msgs::Odometry& cur_state) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        while(time < 10.0) {
            return;
        }
        Eigen::Vector3d dronePosition;
        dronePosition << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
        ROS_INFO_STREAM(dronePosition);
        ROS_WARN_STREAM(referencePosition);
        double deviation = (dronePosition-referencePosition).norm();
        averageDeviation += deviation;
        if(deviation > maxDeviation)
            maxDeviation = deviation;
        if(deviation > 0.5) {
            numberDeviationOutsideThreshold++;
        }
        numberCalls++;

    }

    void writeTestResult() {
        std::ofstream results("../../../results.txt");
        if(!results.is_open()) {
            ROS_ERROR_STREAM("No Results available!");
            return;
        }
        if(numberCalls == 0) {
            maxDeviation = std::nan("");
            numberDeviationOutsideThreshold = INT_MAX;
        }
        results << "##########################\nResult Report:\n"
            << "Tested drone positions: " << numberCalls << std::endl
            << "Average deviation from optimal route: " << averageDeviation / (double)numberCalls << std::endl
            << "Maximum deviation from optimal route: " << maxDeviation << std::endl
            << "Number of drone positions outside flight path threshold: " << numberDeviationOutsideThreshold << std::endl
            << "##########################\n\n";
        results.close();
    }

private:
    ros::NodeHandle nh;
    ros::Time startup_time;
    ros::Subscriber sub_dronePosition;
    ros::Subscriber sub_desPosition;
    ros::Timer t_checkRunTime;

    Eigen::Vector3d referencePosition;

    double maxDeviation;
    double averageDeviation;
    int numberDeviationOutsideThreshold;
    int numberCalls;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Controller_test_node");
    ros::NodeHandle nh;
    Controller_test test(nh);
    ros::spin();
}
