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

#include "plansys2_executor/MissionSupervisor.hpp"

#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "plansys2_msgs/msg/action_execution_info.hpp"

namespace plansys2
{

// ── MissionRetryPolicy ──────────────────────────────────────────────────────

MissionRetryPolicy::MissionRetryPolicy()
: MissionRetryPolicy(Limits())
{
}

MissionRetryPolicy::MissionRetryPolicy(const Limits & limits)
: limits_(limits)
{
}

bool MissionRetryPolicy::may_attempt() const
{
  return limits_.max_attempts <= 0 || attempts_ < limits_.max_attempts;
}

void MissionRetryPolicy::attempt_started()
{
  ++attempts_;
  reason_.clear();
}

bool MissionRetryPolicy::failed(std::size_t completed_actions)
{
  if (completed_actions > 0) {
    // The attempt moved the mission. The next plan starts somewhere new, so it
    // is not the same question asked again.
    without_progress_ = 0;
  } else {
    ++without_progress_;
  }

  if (limits_.max_attempts_without_progress > 0 &&
    without_progress_ >= limits_.max_attempts_without_progress)
  {
    reason_ = std::to_string(without_progress_) +
      " attempt(s) in a row failed with no action completing; the planner is being "
      "asked the same question and answering it the same way";
    return false;
  }

  if (!may_attempt()) {
    reason_ = "the mission has used all " + std::to_string(limits_.max_attempts) +
      " attempt(s) it was allowed";
    return false;
  }

  return true;
}

// ── MissionSupervisor ───────────────────────────────────────────────────────

MissionSupervisor::Options
MissionSupervisor::Options::from_parameters(const rclcpp::Node::SharedPtr & node)
{
  Options options;

  const auto declare = [&](const std::string & name, const auto & value) {
      using T = std::decay_t<decltype(value)>;
      if (!node->has_parameter(name)) {
        return node->declare_parameter<T>(name, value);
      }
      return node->get_parameter(name).get_value<T>();
    };

  options.limits.max_attempts = static_cast<int>(
    declare("mission.max_attempts", static_cast<int64_t>(options.limits.max_attempts)));
  options.limits.max_attempts_without_progress = static_cast<int>(
    declare(
      "mission.max_attempts_without_progress",
      static_cast<int64_t>(options.limits.max_attempts_without_progress)));
  options.settle = std::chrono::milliseconds(
    static_cast<int64_t>(
      1000.0 * declare(
        "mission.settle_seconds",
        std::chrono::duration<double>(options.settle).count())));
  options.check_rate_hz = declare("mission.check_rate_hz", options.check_rate_hz);

  if (options.check_rate_hz <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "mission.check_rate_hz must be positive; using 4.0");
    options.check_rate_hz = 4.0;
  }
  return options;
}

MissionSupervisor::MissionSupervisor(rclcpp::Node::SharedPtr node, const Options & options)
: node_(std::move(node)),
  options_(options),
  domain_(std::make_shared<DomainExpertClient>()),
  problem_(std::make_shared<ProblemExpertClient>()),
  planner_(std::make_shared<PlannerClient>()),
  executor_(std::make_shared<ExecutorClient>())
{
}

MissionSupervisor::MissionSupervisor(rclcpp::Node::SharedPtr node)
: MissionSupervisor(node, Options::from_parameters(node))
{
}

const char * MissionSupervisor::to_string(Outcome outcome)
{
  switch (outcome) {
    case Outcome::Succeeded: return "succeeded";
    case Outcome::AlreadyHolds: return "goal already holds";
    case Outcome::NoPlan: return "no plan";
    case Outcome::Refused: return "refused by the executor";
    case Outcome::GaveUp: return "gave up";
    case Outcome::Interrupted: return "interrupted";
  }
  return "unknown";
}

bool MissionSupervisor::keep_going() const
{
  return rclcpp::ok() && !cancelled_;
}

void MissionSupervisor::settle()
{
  const auto until = std::chrono::steady_clock::now() + options_.settle;
  while (keep_going() && std::chrono::steady_clock::now() < until) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool MissionSupervisor::wait_for_system(std::optional<std::chrono::seconds> timeout)
{
  // The bringup is lifecycle-managed and comes up on its own schedule. A
  // domain expert that answers is the cheapest sign that it has.
  const auto until = std::chrono::steady_clock::now() + timeout.value_or(options_.system_timeout);
  while (keep_going() && std::chrono::steady_clock::now() < until) {
    if (!domain_->getDomain().empty()) {
      return true;
    }
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return false;
}

MissionSupervisor::Report MissionSupervisor::run(const EventCallback & on_event)
{
  const auto started = std::chrono::steady_clock::now();
  const auto logger = node_->get_logger();

  MissionRetryPolicy policy(options_.limits);
  Report report;

  const auto emit = [&](Event event) {
      if (on_event) {
        on_event(event);
      }
    };

  const auto finish = [&](Outcome outcome, const std::string & message) {
      report.outcome = outcome;
      report.attempts = policy.attempts();
      report.message = message;
      report.elapsed_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      Event event{Event::Kind::Finished, policy.attempts()};
      event.failures = report.failures;
      emit(event);
      return report;
    };

  cancelled_ = false;

  while (true) {
    if (!keep_going()) {
      return finish(Outcome::Interrupted, "interrupted before the mission finished");
    }

    policy.attempt_started();
    const int attempt = policy.attempts();
    emit({attempt == 1 ? Event::Kind::Planning : Event::Kind::Replanning, attempt});

    // Against the problem as it stands now. On a replan that is the state the
    // last attempt left, and with the epistemic solver also the belief it
    // reached.
    const auto plan = planner_->getPlan(domain_->getDomain(), problem_->getProblem());
    if (!plan.has_value()) {
      return finish(
        Outcome::NoPlan,
        attempt == 1 ? "the planner found no plan" :
        "the planner found no plan from the state the mission reached");
    }

    {
      Event event{Event::Kind::Planned, attempt};
      event.plan = &plan.value();
      emit(event);
    }

    if (plan->items.empty()) {
      // Nothing left to do. On a first attempt the goal held from the start;
      // on a replan the failed attempt got far enough before it failed.
      return finish(Outcome::AlreadyHolds, "the goal already holds; nothing left to run");
    }

    if (!executor_->start_plan_execution(plan.value())) {
      return finish(Outcome::Refused, "the executor refused the plan");
    }

    emit({Event::Kind::Executing, attempt});

    rclcpp::Rate rate(options_.check_rate_hz);
    while (keep_going() && executor_->execute_and_check_plan()) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }

    if (!keep_going()) {
      executor_->cancel_plan_execution();
      return finish(Outcome::Interrupted, "interrupted while executing");
    }

    const auto result = executor_->getResult();
    if (result && result->result == ExecutePlan::Result::SUCCESS) {
      report.failures.clear();
      return finish(Outcome::Succeeded, "mission complete");
    }

    // Which action died is the whole of what a reader needs afterwards, and
    // the executor knows it: the result carries a status per action.
    std::size_t completed = 0;
    std::string completed_names;
    report.failures.clear();
    if (result) {
      for (const auto & status : result->action_execution_status) {
        // The executor's synthetic (INIT) node, keyed ":0", stands for the
        // initial state and always reports success. Counting it would make
        // every attempt look like progress.
        if (status.action_full_name.empty() || status.action_full_name.front() == ':') {
          continue;
        }
        if (status.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
          ++completed;
          completed_names += (completed_names.empty() ? "" : ", ") + status.action_full_name;
        } else if (status.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
          report.failures.push_back({status.action_full_name, status.message_status});
          RCLCPP_WARN(
            logger, "%s failed: %s",
            status.action_full_name.c_str(), status.message_status.c_str());
        }
      }
    }

    {
      Event event{Event::Kind::AttemptFailed, attempt};
      event.failures = report.failures;
      emit(event);
    }

    if (!policy.failed(completed)) {
      return finish(
        Outcome::GaveUp,
        "mission failed after " + std::to_string(attempt) + " attempt(s): " + policy.reason());
    }

    RCLCPP_WARN(
      logger, "attempt %d failed after completing %zu action(s)%s%s; replanning from the "
      "state the mission reached", attempt, completed, completed ? ": " : "",
      completed_names.c_str());
    settle();
  }
}

}  // namespace plansys2
