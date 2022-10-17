#include <gtest/gtest.h>
#include <fstream>
#include <string>

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
    std::string line = getNthLine("catkin_ws/results.txt", 3);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCalls, 100);
}

TEST_F(TestSuite, checkAverageDronePosition) {
    std::string line = getNthLine("catkin_ws/results.txt", 4);
    double averageDeviation = getNumberOfString(line);
    EXPECT_LE(averageDeviation, 0.3);
}
TEST_F(TestSuite, checkMaxDronePosition) {
    std::string line = getNthLine("catkin_ws/results.txt", 5);
    double maxDeviation = getNumberOfString(line);
    EXPECT_LE(maxDeviation, 0.5);
}
TEST_F(TestSuite, checkNumberOfFalseDronePositions) {
    std::string line = getNthLine("catkin_ws/results.txt", 6);
    int numberFalsePositions = (int)getNumberOfString(line);
    EXPECT_EQ(numberFalsePositions, 0);
}
TEST_F(TestSuite, checkReportedRotorSpeedCalls) {
    std::string line = getNthLine("catkin_ws/results.txt", 7);
    unsigned long numberCallsRotorSpeeds = (unsigned long)getNumberOfString(line);
    line = getNthLine("catkin_ws/results.txt", 3);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCallsRotorSpeeds, numberCalls*15.0);
}
TEST_F(TestSuite, checkMinimumWrench) {
    std::string line = getNthLine("catkin_ws/results.txt", 9);
    double minForce = getNumberOfString(line);
    EXPECT_GT(minForce, -1e-10);
}
TEST_F(TestSuite, checkMaximumWrench) {
    std::string line = getNthLine("catkin_ws/results.txt", 8);
    double maxForce = getNumberOfString(line);
    EXPECT_LE(maxForce, 3500.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
