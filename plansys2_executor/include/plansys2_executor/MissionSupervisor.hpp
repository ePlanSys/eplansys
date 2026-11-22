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

#ifndef PLANSYS2_EXECUTOR__MISSIONSUPERVISOR_HPP_
#define PLANSYS2_EXECUTOR__MISSIONSUPERVISOR_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @brief Whether a failed mission is worth planning again.
 *
 * Kept apart from the loop that talks to the planner and the executor so the
 * decision can be tested without a running system, the way the executor's
 * own replan guard is.
 *
 * Two bounds, and they answer different questions. `max_attempts` is a
 * budget: how many plans the mission may ask for in all. The progress bound is
 * what tells a recoverable failure from a permanent one: an attempt in which
 * some action completed has moved the mission, and replanning from where it
 * got to is a different question from the last one; an attempt in which
 * nothing completed asks the planner the same question again, and a planner
 * that answered it the same way last time will do so again.
 */
class MissionRetryPolicy
{
public:
  struct Limits
  {
    /// Plans the mission may ask for in all, the first included. 0 is no bound.
    int max_attempts{0};
    /// Failed attempts in a row with no action completing before giving up.
    /// 0 is no bound.
    int max_attempts_without_progress{3};
  };

  MissionRetryPolicy();
  explicit MissionRetryPolicy(const Limits & limits);

  /// A plan is about to be asked for. Returns false when the budget is spent.
  bool may_attempt() const;

  /// Record that an attempt started.
  void attempt_started();

  /// Record that an attempt failed, having completed `completed_actions`
  /// actions. Returns whether the mission should plan again.
  bool failed(std::size_t completed_actions);

  int attempts() const {return attempts_;}
  int attempts_without_progress() const {return without_progress_;}
  const Limits & limits() const {return limits_;}

  /// Why the last call to failed() returned false, or empty.
  const std::string & reason() const {return reason_;}

private:
  Limits limits_;
  int attempts_{0};
  int without_progress_{0};
  std::string reason_;
};

/**
 * @class plansys2::MissionSupervisor
 * @brief Plans, executes, and plans again when execution fails.
 *
 * The loop every mission on hardware needs and every demo used to write for
 * itself: ask for a plan, run it, and when an action fails ask again rather
 * than stopping. The planner is asked against the problem as it now stands,
 * so the next plan starts from where the mission got to. With the epistemic
 * solver this includes what the robots have learned: the epistemic state
 * publishes the belief the mission reached, the solver plans from it, and a
 * scan that already happened is not planned for twice.
 *
 * It needs nothing epistemic and works the same over a classical solver; what
 * the epistemic stack adds is a belief worth replanning from.
 */
class MissionSupervisor
{
public:
  using Ptr = std::shared_ptr<MissionSupervisor>;
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;

  struct Options
  {
    MissionRetryPolicy::Limits limits;
    /// How long to let the system settle between a failure and the next plan,
    /// so the state the replan reads includes the failed attempt's last
    /// updates.
    std::chrono::milliseconds settle{std::chrono::seconds(2)};
    /// How often execution is checked.
    double check_rate_hz{4.0};
    /// How long wait_for_system() waits for the domain expert by default.
    std::chrono::seconds system_timeout{std::chrono::seconds(60)};

    /**
     * @brief Read the options from `node`'s parameters, declaring any not yet
     *        declared, with the values above as defaults.
     *
     * mission.max_attempts, mission.max_attempts_without_progress,
     * mission.settle_seconds, mission.check_rate_hz.
     */
    static Options from_parameters(const rclcpp::Node::SharedPtr & node);
  };

  enum class Outcome
  {
    Succeeded,       ///< a plan ran to completion
    AlreadyHolds,    ///< the planner had nothing left to do
    NoPlan,          ///< the planner found no plan
    Refused,         ///< the executor would not start the plan
    GaveUp,          ///< execution kept failing and the retry policy said stop
    Interrupted,     ///< rclcpp shut down, or cancel() was called
  };

  /// One failed action, as the executor reported it.
  struct Failure
  {
    std::string action;
    std::string message;
  };

  struct Report
  {
    Outcome outcome{Outcome::NoPlan};
    int attempts{0};
    /// The actions that failed in the last failed attempt.
    std::vector<Failure> failures;
    /// Human readable, for a log line.
    std::string message;
    double elapsed_seconds{0.0};

    bool succeeded() const
    {
      return outcome == Outcome::Succeeded || outcome == Outcome::AlreadyHolds;
    }
  };

  /// What happened during run(), for a caller that wants to log or react.
  struct Event
  {
    enum class Kind {Planning, Planned, Executing, AttemptFailed, Replanning, Finished};
    Kind kind;
    int attempt{0};
    /// Planned: the plan. Otherwise null.
    const plansys2_msgs::msg::Plan * plan{nullptr};
    /// AttemptFailed: what failed.
    std::vector<Failure> failures;
  };

  using EventCallback = std::function<void (const Event &)>;

  /**
   * @param node Used for logging, parameters and spinning while execution is
   *        checked. The clients make their own nodes, as everywhere else.
   */
  MissionSupervisor(rclcpp::Node::SharedPtr node, const Options & options);
  /// With Options::from_parameters(node).
  explicit MissionSupervisor(rclcpp::Node::SharedPtr node);

  /// Wait until the domain expert answers, as a sign the bringup is up.
  bool wait_for_system(std::optional<std::chrono::seconds> timeout = std::nullopt);

  /// Plan and execute until the mission succeeds or the retry policy gives up.
  /// The problem (instances, facts, goal) must already be set.
  Report run(const EventCallback & on_event = {});

  /// Stop run() at its next check. Safe from another thread.
  void cancel() {cancelled_ = true;}

  const Options & options() const {return options_;}

  std::shared_ptr<DomainExpertClient> domain() {return domain_;}
  std::shared_ptr<ProblemExpertClient> problem() {return problem_;}
  std::shared_ptr<PlannerClient> planner() {return planner_;}
  std::shared_ptr<ExecutorClient> executor() {return executor_;}

  static const char * to_string(Outcome outcome);

private:
  rclcpp::Node::SharedPtr node_;
  Options options_;
  std::shared_ptr<DomainExpertClient> domain_;
  std::shared_ptr<ProblemExpertClient> problem_;
  std::shared_ptr<PlannerClient> planner_;
  std::shared_ptr<ExecutorClient> executor_;
  std::atomic<bool> cancelled_{false};

  bool keep_going() const;
  void settle();
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__MISSIONSUPERVISOR_HPP_
