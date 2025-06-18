#include <gtest/gtest.h>
#include "ukf.h"

TEST(UKFTest, Initialization) {
    UKF ukf;
    ukf.initialize();
    // 添加初始化测试
    EXPECT_TRUE(true);
}

TEST(UKFTest, Prediction) {
    UKF ukf;
    ukf.initialize();
    ukf.predict(0.1);
    // 添加预测测试
    EXPECT_TRUE(true);
}

TEST(UKFTest, Update) {
    UKF ukf;
    ukf.initialize();
    Eigen::VectorXd measurement(2);
    measurement << 1.0, 2.0;
    ukf.update(measurement);
    // 添加更新测试
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
