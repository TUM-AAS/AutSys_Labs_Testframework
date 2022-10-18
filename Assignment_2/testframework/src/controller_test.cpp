#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <fstream>
#define START_TIME 5.0
#define SHUTDOWN_TIME 30.0

class Controller_test  {
public:
    Controller_test(ros::NodeHandle nh_) : nh(nh_), maxDeviation(0.0), averageDeviation(0.0),
                numberCalls(0), numberDeviationOutsideThreshold(0), referencePosition(0.0,0.0,0.0), minWrench(10000.0, 10.0, 10.0, 10.0), maxWrench(-10000, -10.0, -10.0, -10.0), averageWrench(0.0,0.0,0.0,0.0), numberCallsPropSpeeds(0) {
        sub_dronePosition = nh.subscribe("current_state", 5, &Controller_test::checkDronePosition, this);
        sub_desPosition = nh.subscribe("desired_state", 5, &Controller_test::saveReferencePosition, this);
        sub_rotorSpeedCmds = nh.subscribe("rotor_speed_cmds", 5, &Controller_test::checkRotorSpeeds, this);
        startup_time = ros::Time::now();
        
        t_checkRunTime = nh.createTimer(ros::Duration(0.1), &Controller_test::checkRunTime, this);
        t_checkRunTime.start();
       
    }

    void checkRunTime(const ros::TimerEvent& t) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        if(time > SHUTDOWN_TIME) {
            writeTestResult();
            ros::shutdown();
        }
    }

    static double signed_pow2(double val) {
        return val>0?pow(val,2):-pow(val,2);
    }

    void checkRotorSpeeds(const mav_msgs::Actuators& rotor_speeds) {
    	ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        while(time < START_TIME) {
            return;
        }
        numberCallsPropSpeeds++;
    	Eigen::Vector4d props;
        for(int i = 0; i < 4; i++) {
            props[i] = signed_pow2(rotor_speeds.angular_velocities[i]);
        }      
        
        const double d_hat = 0.3/sqrt(2);
        const double cd = 1e-5;
        const double cf = 1e-3;
	    Eigen::Matrix4d F;

	    F << cf, cf, cf, cf, 
		    cf*d_hat, cf*d_hat, -cf*d_hat, -cf*d_hat, 
		    -cf*d_hat, cf*d_hat, cf*d_hat, -cf*d_hat,
		    cd, -cd, cd, -cd;

    
        Eigen::Vector4d wrench = F*props;
        averageWrench += wrench;
        for(int i = 0; i < 4; i++) {
            if(wrench[i] < minWrench[i])
                minWrench[i] = wrench[i];
            else if(wrench[i] > maxWrench[i])
                maxWrench[i] = wrench[i];
        }
    }

    void saveReferencePosition (const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state) {
        geometry_msgs::Vector3 t = des_state.transforms[0].translation;
        referencePosition << t.x, t.y, t.z;
    }

    void checkDronePosition(const nav_msgs::Odometry& cur_state) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        while(time < START_TIME) {
            return;
        }
        Eigen::Vector3d dronePosition;
        dronePosition << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
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
        if(numberCallsPropSpeeds == 0) {
            maxWrench = {-std::nan(""),-std::nan(""),-std::nan(""),-std::nan("")};
            minWrench = {std::nan(""),std::nan(""),std::nan(""),std::nan("")};
        }
        if(abs(minWrench[0]) < 1e10)
            minWrench[0] = 0;
        averageWrench /= numberCallsPropSpeeds;
        results << "##########################\nResult Report:\n"
            << "Tested drone positions: " << numberCalls << std::endl
            << "Average deviation from optimal route: " << averageDeviation / (double)numberCalls << std::endl
            << "Maximum deviation from optimal route: " << maxDeviation << std::endl
            << "Number of drone positions outside flight path threshold: " << numberDeviationOutsideThreshold << std::endl
            << "Tested rotor speed commands: " << numberCallsPropSpeeds << std::endl
            << "Received (elementwise) average wrench: " << averageWrench[0] << ", " << averageWrench[1] << ", " << averageWrench[2] << ", " << averageWrench[3] << "\n"
            << "Received (elementwise) maximum wrench: " << maxWrench[0] << ", " << maxWrench[1] << ", " << maxWrench[2] << ", " << maxWrench[3] << "\n"
            << "Received (elementwise) minimum wrench: " << minWrench[0] << ", " << minWrench[1] << ", " << minWrench[2] << ", " << minWrench[3] << "\n"
            << "##########################\n\n";
        results.close();
    }

private:
    ros::NodeHandle nh;
    ros::Time startup_time;
    ros::Subscriber sub_dronePosition;
    ros::Subscriber sub_desPosition;
    ros::Subscriber sub_rotorSpeedCmds;
    ros::Timer t_checkRunTime;

    Eigen::Vector3d referencePosition;

    double maxDeviation;
    double averageDeviation;
    int numberDeviationOutsideThreshold;
    int numberCalls;
    unsigned long numberCallsPropSpeeds;
    Eigen::Vector4d minWrench;
    Eigen::Vector4d maxWrench;
    Eigen::Vector4d averageWrench;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Controller_test_node");
    ros::NodeHandle nh;
    Controller_test test(nh);
    ros::spin();
}
