// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// When a failed mission is worth planning again.
//
// The loop around it needs a planner, an executor and performers; whether to go
// round it again does not, and that decision is what these tests pin down.

#include <sstream>

#include "gtest/gtest.h"

#include "plansys2_executor/MissionSupervisor.hpp"

using plansys2::MissionRetryPolicy;

namespace
{

MissionRetryPolicy policy(int max_attempts, int max_without_progress)
{
  MissionRetryPolicy::Limits limits;
  limits.max_attempts = max_attempts;
  limits.max_attempts_without_progress = max_without_progress;
  return MissionRetryPolicy(limits);
}

}  // namespace

TEST(MissionRetryPolicy, OneAttemptIsTheClassicalBehaviour)
{
  auto p = policy(1, 0);
  ASSERT_TRUE(p.may_attempt());
  p.attempt_started();
  EXPECT_FALSE(p.failed(5)) << "a budget of one plan must not replan, however far it got";
  EXPECT_FALSE(p.reason().empty());
}

TEST(MissionRetryPolicy, AnAttemptThatGotFurtherIsWorthAnother)
{
  auto p = policy(0, 2);
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(p.may_attempt());
    p.attempt_started();
    // Each attempt completes something before failing: the mission is moving,
    // and no number of such attempts is a loop.
    EXPECT_TRUE(p.failed(1)) << "attempt " << p.attempts();
    EXPECT_EQ(p.attempts_without_progress(), 0);
  }
}

TEST(MissionRetryPolicy, RepeatingTheSameFailureIsGivenUpOn)
{
  auto p = policy(0, 3);
  p.attempt_started();
  EXPECT_TRUE(p.failed(0));
  p.attempt_started();
  EXPECT_TRUE(p.failed(0));
  p.attempt_started();
  EXPECT_FALSE(p.failed(0)) << "three failures in a row with nothing completing is a loop";
  EXPECT_NE(p.reason().find("same question"), std::string::npos) << p.reason();
}

TEST(MissionRetryPolicy, ProgressResetsTheCount)
{
  auto p = policy(0, 2);
  p.attempt_started();
  EXPECT_TRUE(p.failed(0));
  EXPECT_EQ(p.attempts_without_progress(), 1);
  p.attempt_started();
  EXPECT_TRUE(p.failed(3));
  EXPECT_EQ(p.attempts_without_progress(), 0);
  p.attempt_started();
  EXPECT_TRUE(p.failed(0));
  p.attempt_started();
  EXPECT_FALSE(p.failed(0));
}

TEST(MissionRetryPolicy, TheBudgetBoundsEvenAMissionThatIsMoving)
{
  auto p = policy(3, 0);
  for (int i = 0; i < 2; ++i) {
    p.attempt_started();
    EXPECT_TRUE(p.failed(1));
  }
  p.attempt_started();
  EXPECT_FALSE(p.failed(1));
  EXPECT_FALSE(p.may_attempt());
  EXPECT_NE(p.reason().find("all 3"), std::string::npos) << p.reason();
}

TEST(MissionRetryPolicy, ZeroMeansNoBound)
{
  auto p = policy(0, 0);
  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(p.may_attempt());
    p.attempt_started();
    ASSERT_TRUE(p.failed(0));
  }
}

TEST(MissionRetryPolicy, TheDefaultsGiveUpAfterThreeFruitlessAttempts)
{
  MissionRetryPolicy p;
  EXPECT_EQ(p.limits().max_attempts, 0);
  EXPECT_EQ(p.limits().max_attempts_without_progress, 3);
}

TEST(MissionSupervisor, AnOutcomePrintsByName)
{
  // GoogleTest prints both sides of a failed EXPECT_EQ; an Outcome with no
  // printer of its own did not compile against recent GoogleTest.
  std::ostringstream os;
  os << plansys2::MissionSupervisor::Outcome::GaveUp;
  EXPECT_EQ(os.str(), "gave up");
  EXPECT_EQ(
    plansys2::MissionSupervisor::Outcome::Succeeded,
    plansys2::MissionSupervisor::Outcome::Succeeded);
}
