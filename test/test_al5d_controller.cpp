#include <gtest/gtest.h>
#include "arminator/al5d_controller.hpp"
#include "arminator/kinematics.hpp"

namespace arminator {

class AL5DControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Note: These tests use a mock serial port that won't actually connect
        controller_ = std::make_unique<AL5DController>("/dev/null", 9600);
    }
    
    void TearDown() override {
        if (controller_) {
            controller_->shutdown();
        }
    }
    
    std::unique_ptr<AL5DController> controller_;
};

TEST_F(AL5DControllerTest, ConstructorTest) {
    EXPECT_NE(controller_, nullptr);
}

TEST_F(AL5DControllerTest, JointLimitsTest) {
    JointLimits limits(6);
    limits.min_position = {-90, -90, -90, -90, -180, 0};
    limits.max_position = {90, 90, 90, 90, 180, 180};
    
    controller_->setJointLimits(limits);
    
    auto retrieved_limits = controller_->getJointLimits();
    
    EXPECT_EQ(retrieved_limits.min_position.size(), 6);
    EXPECT_EQ(retrieved_limits.max_position.size(), 6);
    EXPECT_EQ(retrieved_limits.min_position[0], -90.0);
    EXPECT_EQ(retrieved_limits.max_position[0], 90.0);
}

TEST_F(AL5DControllerTest, JointStateTest) {
    auto joint_state = controller_->getCurrentJointState();
    
    EXPECT_EQ(joint_state.positions.size(), 6);
    EXPECT_EQ(joint_state.velocities.size(), 6);
    EXPECT_EQ(joint_state.efforts.size(), 6);
}

class AL5DKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        kinematics_ = std::make_unique<AL5DKinematics>();
    }
    
    std::unique_ptr<AL5DKinematics> kinematics_;
};

TEST_F(AL5DKinematicsTest, ForwardKinematicsTest) {
    std::vector<double> joint_angles = {0, 0, 0, 0, 0}; // Home position
    
    auto pose = kinematics_->forwardKinematics(joint_angles);
    
    // At home position, end effector should be in front of the robot
    EXPECT_GT(pose.x, 0);
    EXPECT_NEAR(pose.y, 0, 1.0); // Should be close to Y=0
}

TEST_F(AL5DKinematicsTest, InverseKinematicsTest) {
    // Test with a reachable pose
    CartesianPose target_pose(200, 0, 200, 0, 0, 0); // 200mm forward and up
    
    auto joint_angles = kinematics_->inverseKinematics(target_pose);
    
    // Should return a valid solution (non-empty vector)
    EXPECT_EQ(joint_angles.size(), 5);
    
    // Verify forward kinematics gives back similar pose
    auto forward_pose = kinematics_->forwardKinematics(joint_angles);
    
    EXPECT_NEAR(forward_pose.x, target_pose.x, 10.0); // Within 10mm tolerance
    EXPECT_NEAR(forward_pose.y, target_pose.y, 10.0);
    EXPECT_NEAR(forward_pose.z, target_pose.z, 10.0);
}

TEST_F(AL5DKinematicsTest, WorkspaceLimitsTest) {
    auto limits = kinematics_->getWorkspaceLimits();
    
    EXPECT_EQ(limits.size(), 6);
    EXPECT_LT(limits[0], limits[1]); // min_x < max_x
    EXPECT_LT(limits[2], limits[3]); // min_y < max_y
    EXPECT_LT(limits[4], limits[5]); // min_z < max_z
}

TEST_F(AL5DKinematicsTest, ReachabilityTest) {
    // Test reachable pose
    CartesianPose reachable_pose(200, 0, 200, 0, 0, 0);
    EXPECT_TRUE(kinematics_->isReachable(reachable_pose));
    
    // Test unreachable pose (too far)
    CartesianPose unreachable_pose(1000, 0, 1000, 0, 0, 0);
    EXPECT_FALSE(kinematics_->isReachable(unreachable_pose));
}

class UtilityTest : public ::testing::Test {};

TEST_F(UtilityTest, TrajectoryPointTest) {
    JointState state(6);
    state.positions = {0, 0, 0, 0, 0, 90};
    
    TrajectoryPoint point(state, 1.5);
    
    EXPECT_EQ(point.joint_state.positions.size(), 6);
    EXPECT_EQ(point.time_from_start, 1.5);
}

TEST_F(UtilityTest, CartesianPoseTest) {
    CartesianPose pose(100, 200, 300, 10, 20, 30);
    
    EXPECT_EQ(pose.x, 100);
    EXPECT_EQ(pose.y, 200);
    EXPECT_EQ(pose.z, 300);
    EXPECT_EQ(pose.roll, 10);
    EXPECT_EQ(pose.pitch, 20);
    EXPECT_EQ(pose.yaw, 30);
}

} // namespace arminator

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}