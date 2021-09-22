#include <gtest/gtest.h>

#include "planning_params.h"
#include "structs.h"
#include "motion_planner.h"


class MotionPlannerTest : public ::testing::Test {
 protected:
  std::unique_ptr<MotionPlanner> motion_planner;
  
  void SetUp() override {
     motion_planner = std::make_unique<MotionPlanner>(
      P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);
  }

  // void TearDown() override {}
};

TEST_F(MotionPlannerTest, OffsetGoalsStraight) {
  State goal;
  goal.location.x = 10;
  
  std::vector<State> offset_goals = motion_planner->generate_offset_goals(goal);
  
  for (int i = 0; i < P_NUM_PATHS; ++i) {
    float offset = (i - (int)(P_NUM_PATHS / 2)) * P_GOAL_OFFSET;
    EXPECT_FLOAT_EQ(goal.location.x, offset_goals[i].location.x);
    EXPECT_FLOAT_EQ(goal.location.y + offset, offset_goals[i].location.y);
  }
}

TEST(VelocityProfileGeneratorTest, CalcDistanceZeroAcceleration) {
  auto distance = VelocityProfileGenerator::calc_distance(10.0, 10.0, 10.0);
  EXPECT_FLOAT_EQ(0.0, distance);
}

TEST(VelocityProfileGeneratorTest, CalcDistance) {
  auto distance = VelocityProfileGenerator::calc_distance(10.0, 0.0, -10.0);
  EXPECT_FLOAT_EQ(5.0, distance);
}

TEST(VelocityProfileGeneratorTest, CalcFinalSpeedFullStop) {
  auto speed = VelocityProfileGenerator::calc_final_speed(10.0, -10.0, 5.0);
  EXPECT_FLOAT_EQ(0.0, speed);
}
