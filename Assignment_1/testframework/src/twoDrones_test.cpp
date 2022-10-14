#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <eigen3/Eigen/Core>

#include <fstream>

class TwoDrones_test  {
public:
    TwoDrones_test(ros::NodeHandle nh_) : nh(nh_), maxDeviation(0.0), averageDeviation(0.0),
                numberCalls(0), numberDeviationOutsideThreshold(0) {
        sub_dronePosition = nh.subscribe("tf", 5, &TwoDrones_test::checkDronePosition, this);
        startup_time = ros::Time::now();
        
        t_checkRunTime = nh.createTimer(ros::Duration(0.1), &TwoDrones_test::checkRunTime, this);
        t_checkRunTime.start();
       
    }

    void checkRunTime(const ros::TimerEvent& t) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        if(time > 10.0) {
            writeTestResult();
            ros::shutdown();
        }
    }

    void checkDronePosition(tf2_msgs::TFMessage msg) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        Eigen::Vector3d dronePosition;
        geometry_msgs::TransformStamped position = msg.transforms[0];
        dronePosition << position.transform.translation.x, position.transform.translation.y, position.transform.translation.z;
        Eigen::Vector3d referencePosition;
        if(position.child_frame_id == "av1") {
            referencePosition << cos(time), sin(time), 0;
        }
        else {
            referencePosition << sin(time), 0, cos(2*time);
        }
        double deviation = (dronePosition-referencePosition).norm();
        averageDeviation += deviation;
        if(deviation > maxDeviation)
            maxDeviation = deviation;
        if(deviation > 0.1) {
            numberDeviationOutsideThreshold++;
        }
        numberCalls++;

    }

    void writeTestResult() {
        std::ofstream results("../../../results.txt");
        if(!results.is_open())
            ROS_ERROR_STREAM("No Results available!");
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
    ros::Timer t_checkRunTime;

    double maxDeviation;
    double averageDeviation;
    int numberDeviationOutsideThreshold;
    int numberCalls;
        
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "twoDrones_test_node");
    ros::NodeHandle nh;
    TwoDrones_test test(nh);
    ros::spin();
}
