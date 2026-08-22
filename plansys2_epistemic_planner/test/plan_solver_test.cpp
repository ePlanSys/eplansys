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

// The front end through the plugin: EPDDL sources named by parameter, and a
// plan back. The pieces either side of this — grounding, and the search — have
// their own tests; what is checked here is that the plugin joins them, since
// that is the path a launch file takes and the one no unit test covers.

#include <unistd.h>

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_epddl_grounder/epddl_grounder.hpp"
#include "plansys2_epistemic_planner/epistemic_plan_solver.hpp"
#include "plansys2_pddl_parser/AmentIndexCompat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

std::string example(const std::string & name)
{
  return plansys2::get_package_share_dir("plansys2_epddl_grounder") + "/examples/" + name;
}

/// The muddy-children sources, as the grounder's own example ships them.
plansys2::EpddlSpec muddy_children()
{
  plansys2::EpddlSpec spec;
  spec.domain = example("muddy-children-domain.epddl");
  spec.problem = example("muddy-children-problem.epddl");
  return spec;
}

/// plank is a run-time dependency built from source, so a workspace can
/// legitimately be without it. Ask the grounder rather than guessing at PATH.
bool plank_is_available()
{
  plansys2::EpddlGrounder grounder;
  return grounder.ground(muddy_children()).ok;
}

}  // namespace

class SolverFromEpddl : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!plank_is_available()) {
      GTEST_SKIP() << "plank is not built in this workspace";
    }

    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("epistemic_solver_test");
    solver_.configure(node_, "EPISTEMIC");

    const auto spec = muddy_children();
    node_->set_parameter(rclcpp::Parameter("EPISTEMIC.epddl_domain", spec.domain));
    node_->set_parameter(rclcpp::Parameter("EPISTEMIC.epddl_problem", spec.problem));
    node_->set_parameter(
      rclcpp::Parameter(
        "EPISTEMIC.action_mapping",
        std::string(EPISTEMIC_EXAMPLE_MAPPING_DIR) + "/muddy-children-2.json"));
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  plansys2::EpistemicPlanSolver solver_;
};

TEST_F(SolverFromEpddl, PlansWithoutEverSeeingAGroundedTask)
{
  // The PDDL strings are what the planner node passes through from the domain
  // and problem experts. Neither says anything about knowledge, and the solver
  // is expected to ignore both and read its own sources.
  const auto plan = solver_.getPlan("(define (domain d))", "(define (problem p))");

  ASSERT_TRUE(plan.has_value());
  ASSERT_FALSE(plan->items.empty());

  // Solving muddy children means asking; the mapping turns the grounded
  // `ask_c1` into the expression the executor dispatches.
  for (const auto & item : plan->items) {
    EXPECT_NE(item.action.find("(ask "), std::string::npos) << item.action;
  }
}

TEST_F(SolverFromEpddl, ASourceThatDoesNotExistIsReportedRatherThanPlannedAround)
{
  node_->set_parameter(
    rclcpp::Parameter("EPISTEMIC.epddl_problem", example("no-such-problem.epddl")));

  EXPECT_FALSE(solver_.getPlan("", "").has_value());
}

// Leave through _exit, so the DDS threads never outlive the process.
//
// rclcpp::shutdown() does not finalise the global context; that happens in
// static destruction, inside _dl_fini, while Fast DDS listener threads are
// still running in libraries the loader is unmapping. Under --coverage, which
// is how the rolling job builds, that same exit path writes a .gcda for every
// object file, stretching the window until about one run in ten segfaults with
// every test already passed. Dumping the counters and leaving through _exit
// keeps the coverage data and skips static destruction. __gcov_dump is weak:
// it is null in an uninstrumented build.
extern "C" void __gcov_dump(void) __attribute__((weak));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  if (__gcov_dump) {
    __gcov_dump();
  }
  _exit(result);
}
