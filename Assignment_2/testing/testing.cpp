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
    EXPECT_LE(averageDeviation, 0.25);
}
TEST_F(TestSuite, checkMaxDronePosition) {
    std::string line = getNthLine("catkin_ws/results.txt", 5);
    double maxDeviation = getNumberOfString(line);
    EXPECT_LE(maxDeviation, 0.5);
}
TEST_F(TestSuite, checkNumberOfFalseDronePositions) {
    std::string line = getNthLine("catkin_ws/results.txt", 6);
    int numberFalsePositions = (int)getNumberOfString(line);
    EXPECT_EQ(numberFalsePositions, 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
