#include <trajectory_test.h>

#include <fstream>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


#define MAX_FLIGHT_TIME 250.0
Trajectory_test::Trajectory_test(ros::NodeHandle nh_) : nh(nh_), numberCalls(0), minWrench(10000.0, 10.0, 10.0, 10.0), maxWrench(-10000, -10.0, -10.0, -10.0),
            numberCallsPropSpeeds(0), flightDuration(MAX_FLIGHT_TIME), referencePositions(), reachedPositionNTimes(), positionSequence(),
            droneAtGoalPosition(), stopPositions(), receivedTrajectory(0) {

    sub_dronePosition = nh.subscribe("current_state", 5, &Trajectory_test::checkDronePosition, this);
    sub_trajectory = nh.subscribe("trajectory", 5, &Trajectory_test::checkSentTrajectory, this);
    sub_rotorSpeedCmds = nh.subscribe("rotor_speed_cmds", 5, &Trajectory_test::checkRotorSpeeds, this);
    startup_time = ros::Time::now();
    
    t_checkRunTime = nh.createTimer(ros::Duration(0.1), &Trajectory_test::checkRunTime, this);
    
    readReferencePositions();
    
}

void Trajectory_test::readReferencePositions() {
    referencePositions.resize(6);
    reachedPositionNTimes.resize(referencePositions.size());
    droneAtGoalPosition.resize(referencePositions.size());
    for(size_t i = 1; i <= referencePositions.size(); i++) {
        std::string vertexName = "Vertex_" + std::to_string(i);
        std::vector<double> pos;
        nh.getParam("/test_node/Vertex/"+vertexName+"/pos", pos);
        referencePositions[i-1] << pos[0], pos[1], pos[2];
    }
}

void Trajectory_test::checkRunTime(const ros::TimerEvent& t) {
    ros::Time actual_time = ros::Time::now();
    double time = (actual_time - startup_time).toSec();
    if(time > flightDuration) {
        writeTestResult();
        ros::shutdown();
    }
}

static double signed_pow2(double val) {
    return val>0?pow(val,2):-pow(val,2);
}

void Trajectory_test::checkRotorSpeeds(const mav_msgs::Actuators& rotor_speeds) {
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

    if(receivedTrajectory == 1) {
        for(int i = 0; i < 4; i++) {
            if(wrench[i] < minWrench[i])
                minWrench[i] = wrench[i];
            else if(wrench[i] > maxWrench[i])
                maxWrench[i] = wrench[i];
        }
    }
}

void Trajectory_test::checkSentTrajectory (const mav_planning_msgs::PolynomialTrajectory4D& msg) {
    startup_time = ros::Time::now();
    flightDuration = 2.0;
    for(auto iSegment : msg.segments) {
        flightDuration += iSegment.segment_time.toSec();
    }
    t_checkRunTime.start();
    receivedTrajectory = 1;
}

void Trajectory_test::checkDronePosition(const nav_msgs::Odometry& cur_state) {
    numberCalls++;
    Eigen::Vector3d dronePosition;
    dronePosition << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
    
    checkGoalPositions(dronePosition);

    Eigen::Vector3d vel;
    vel << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    if(vel.norm() < 0.03 && dronePosition.norm() > 0.1) {
        ros::Time actual_time = ros::Time::now();
        double time = (actual_time - startup_time).toSec();
        Eigen::Vector4d stopPosition(time, dronePosition[0], dronePosition[1], dronePosition[2]);
        stopPositions.push_back(stopPosition);
    }
}

void Trajectory_test::checkGoalPositions(Eigen::Vector3d dronePosition) {
    for(size_t i = 0; i < referencePositions.size(); i++) {
        if(abs((dronePosition-referencePositions[i]).norm()) < 0.2) {
            if(droneAtGoalPosition[i] != 1) {
                positionSequence.push_back(i);
                reachedPositionNTimes[i]++;
                droneAtGoalPosition[i] = 1; 
            }
        }
        else {
            droneAtGoalPosition[i] = 0;
        }
    }
}

void Trajectory_test::writeTestResult() {
    std::ofstream results("../../../results.txt");
    if(!results.is_open()) {
        ROS_ERROR_STREAM("No Results available!");
        return;
    }
    if(numberCallsPropSpeeds == 0) {
        maxWrench = {-std::nan(""),-std::nan(""),-std::nan(""),-std::nan("")};
        minWrench = {std::nan(""),std::nan(""),std::nan(""),std::nan("")};
    }
    results << "##########################\nResult Report:\n"
        << "Recorded flight time: " << flightDuration << std::endl
        << "Tested drone positions: " << numberCalls << std::endl
        << "Tested rotor speed commands: " << numberCallsPropSpeeds << std::endl
        << "Received (elementwise) maximum wrench: " << maxWrench[0] << ", " << maxWrench[1] << ", " << maxWrench[2] << ", " << maxWrench[3] << "\n"
        << "Received (elementwise) minimum wrench: " << minWrench[0] << ", " << minWrench[1] << ", " << minWrench[2] << ", " << minWrench[3] << "\n"
        << "Goal sequence: ";
    //print sequence
    for(int i = 0; i < positionSequence.size(); i++) {
        int pos = positionSequence[i]+1;
        results << pos;
        if(i != positionSequence.size()-1) {
            results << ", ";
        }
    }
    //print reached positions
    results << "\nCounted drone transitions for all " << referencePositions.size() << " goals:\n";
    for(int i = 0; i < referencePositions.size(); i++) {
        results << "    Counted transitions for [" << referencePositions[i][0] << ", " << referencePositions[i][1] << ", " << referencePositions[i][2]
            << "]: " << reachedPositionNTimes[i] << std::endl;
    }
    results << "Reported number of drone positions with 0 velocity: " << stopPositions.size() << std::endl;
    for(auto iPos : stopPositions) {
        results << "    [" << iPos[1] << ", " << iPos[2] << ", " << iPos[3] << "] at time: " << iPos[0] << std::endl;
    }
    results << "##########################\n\n";
    results.close();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_test_node");
    ros::NodeHandle nh;
    Trajectory_test test(nh);
    ros::spin();
}