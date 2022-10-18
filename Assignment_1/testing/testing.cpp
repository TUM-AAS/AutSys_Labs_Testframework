#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <iostream>

class TestSuite : public ::testing::Test {
public:
    std::string getNthLine(std::string filename, int lineNumber) {
        std::ifstream results(filename);
        EXPECT_TRUE(results.is_open());
        std::string line;
        for(int i = 0; i < lineNumber; i++) {
            std::getline(results, line);
        }
        return line;
    }

    double getNumberOfString(std::string line) {
        auto posStart = line.find(": ") + 2;
        auto posStop = line.find("\n");
        EXPECT_NE(posStart, std::string::npos);
        std::string numberStr = line.substr(posStart, posStop);
        double number = std::stod(numberStr);
        return number;
    }
};
TEST_F(TestSuite, checkReportedCalls)  {
	std::cout << "Checks weather enough calls for a comparison were sent\n";
    std::string line = getNthLine("catkin_ws/results.txt", 3);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCalls, 100);
}

TEST_F(TestSuite, checkAverageDronePosition) {
	std::cout << "Compares the average deviation of the drone position to the desired position\n";
    std::string line = getNthLine("catkin_ws/results.txt", 4);
    double averageDeviation = getNumberOfString(line);
    EXPECT_LE(averageDeviation, 0.06);
}
TEST_F(TestSuite, checkMaxDronePosition) {
	std::cout << "Checks if the maximum deviation of the reported drone positions to the desired position is small enough\n";
    std::string line = getNthLine("catkin_ws/results.txt", 5);
    double maxDeviation = getNumberOfString(line);
    EXPECT_LE(maxDeviation, 0.2);
}
TEST_F(TestSuite, checkNumberOfFalseDronePositions) {
	std::cout << "Checks if maximal 1 reported drone position has a deviation greater than 0.1 to the desired position\n";
    std::string line = getNthLine("catkin_ws/results.txt", 6);
    int numberFalsePositions = (int)getNumberOfString(line);
    EXPECT_LE(numberFalsePositions, 1);
}
TEST_F(TestSuite, checkPlotsPublisher) { 
	std::cout << "Checks if the recommended functions were used in the plots\_publisher\_node\n";
    std::ifstream code ("catkin_ws/src/two_drones_pkg/src/plots_publisher_node.cpp");
    EXPECT_TRUE(code.is_open());
    std::stringstream buf;
    buf << code.rdbuf();
    std::string input = buf.str();
    std::string test = "waitForTransform";
    auto pos = input.find(test);
    pos = input.find(test, pos+1);
    EXPECT_NE(pos, std::string::npos);
    test = "lookupTransform";
    pos = input.find(test);
    pos = input.find(test, pos+1);
    EXPECT_NE(pos, std::string::npos);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
