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

#include "plansys2_epistemic_executor/EpistemicStateNode.hpp"

#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_epistemic_planner/action.hpp"
#include "plansys2_epistemic_planner/bitset.hpp"
#include "plansys2_epistemic_planner/formula_text.hpp"
#include "plansys2_epistemic_planner/formula.hpp"
#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/policy_plan.hpp"
#include "plansys2_epistemic_planner/product_update.hpp"
#include "plansys2_epistemic_planner/state.hpp"
#include "plansys2_epistemic_planner/state_json.hpp"

namespace plansys2
{

namespace
{

/// The planner's loader reads a path. Writing the inline task out keeps one
/// loader for both ways of supplying a task, so they cannot drift.
class TempTask
{
public:
  explicit TempTask(const std::string & contents)
  {
    path_ = std::filesystem::temp_directory_path() /
      ("eplansys-state-" + std::to_string(::getpid()) + ".json");
    std::ofstream out(path_);
    out << contents;
  }

  ~TempTask()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  TempTask(const TempTask &) = delete;
  TempTask & operator=(const TempTask &) = delete;

  std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
};

}  // namespace

EpistemicStateNode::EpistemicStateNode()
: rclcpp_lifecycle::LifecycleNode("epistemic_state")
{
  declare_parameter<std::string>("task_file", "");
  // Whether the published state carries the model as well as its shape. On by
  // default, since replanning from where a mission got to depends on it; worth
  // turning off only when nothing replans and the model is large enough for
  // the message to matter.
  declare_parameter<bool>("publish_model", true);
  declare_epddl_parameters(this, epddl_parameter_names_);
}

EpistemicStateNode::CallbackReturnT
EpistemicStateNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  publish_model_ = get_parameter("publish_model").as_bool();

  load_task_service_ = create_service<plansys2_epistemic_msgs::srv::LoadTask>(
    "epistemic_state/load_task",
    std::bind(
      &EpistemicStateNode::load_task_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  check_formula_service_ = create_service<plansys2_epistemic_msgs::srv::CheckFormula>(
    "epistemic_state/check_formula",
    std::bind(
      &EpistemicStateNode::check_formula_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  apply_action_service_ = create_service<plansys2_epistemic_msgs::srv::ApplyAction>(
    "epistemic_state/apply_action",
    std::bind(
      &EpistemicStateNode::apply_action_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  get_goal_service_ = create_service<plansys2_epistemic_msgs::srv::GetGoal>(
    "epistemic_state/get_goal",
    std::bind(
      &EpistemicStateNode::get_goal_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  set_goal_service_ = create_service<plansys2_epistemic_msgs::srv::SetGoal>(
    "epistemic_state/set_goal",
    std::bind(
      &EpistemicStateNode::set_goal_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  announce_service_ = create_service<plansys2_epistemic_msgs::srv::Announce>(
    "epistemic_state/announce",
    std::bind(
      &EpistemicStateNode::announce_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  state_pub_ = create_publisher<std_msgs::msg::String>(
    "epistemic_state/state", rclcpp::QoS(10).transient_local());

  grounder_ = EpddlGrounder(read_plank_command(this, epddl_parameter_names_));

  // A task named at configure time is the common case: one mission, one task,
  // loaded before anything asks a question about it. EPDDL sources are the
  // ordinary way to name it — the same pair of paths the planner is given, so
  // that the policy and the state it is checked against come from one source
  // — and a pre-ground task_file is the alternative.
  const auto spec = read_epddl_spec(this, epddl_parameter_names_);
  const auto task_file = get_parameter("task_file").as_string();

  if (!spec.empty() && !task_file.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "[epistemic_state] both EPDDL sources and a task_file are set; "
      "grounding the sources and ignoring %s.", task_file.c_str());
  }

  if (!spec.empty()) {
    const auto ground = grounder_.ground(spec);
    if (!ground.ok) {
      RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", ground.error.c_str());
      return CallbackReturnT::FAILURE;
    }
    try {
      TempTask temporary(ground.task_json);
      task_ = load_task(temporary.path());
      state_ = task_->init;
      RCLCPP_INFO(
        get_logger(), "[epistemic_state] ground %s: %zu worlds, %zu agents",
        spec.problem.c_str(), static_cast<std::size_t>(state_->num_worlds),
        task_->num_agents());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[epistemic_state] could not load task: %s", e.what());
      return CallbackReturnT::FAILURE;
    }
  } else if (!task_file.empty()) {
    try {
      task_ = load_task(task_file);
      state_ = task_->init;
      RCLCPP_INFO(
        get_logger(), "[epistemic_state] loaded %s: %zu worlds, %zu agents",
        task_file.c_str(), static_cast<std::size_t>(state_->num_worlds),
        task_->num_agents());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[epistemic_state] could not load task: %s", e.what());
      return CallbackReturnT::FAILURE;
    }
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

EpistemicStateNode::CallbackReturnT
EpistemicStateNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  state_pub_->on_activate();
  publish_state();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

EpistemicStateNode::CallbackReturnT
EpistemicStateNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  state_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());
  return CallbackReturnT::SUCCESS;
}

void EpistemicStateNode::publish_state()
{
  if (!state_ || !state_pub_ || !state_pub_->is_activated()) {
    return;
  }
  // Enough for a monitor to say what is going on without asking: the shape of
  // the model, what is being aimed at, and whether it is there yet. The goal
  // travels here rather than only through get_goal so that a subscriber —
  // the plan solver among them — can follow it without a service call from
  // inside a service callback.
  const auto goal = goal_text();

  std::string data =
    "{\"worlds\": " + std::to_string(state_->num_worlds) +
    ", \"designated\": " + std::to_string(state_->num_designated()) +
    ", \"agents\": " + std::to_string(task_ ? task_->num_agents() : 0) +
    ", \"atoms\": " + std::to_string(task_ ? task_->num_atoms() : 0) +
    ", \"goal\": \"" + goal + "\"" +
    ", \"goal_from_task\": " + (goal_override_ ? "false" : "true");

  if (!goal.empty()) {
    const auto & formula = goal_override_ ? goal_override_ : task_->goal;
    data += ", \"goal_holds\": ";
    data += state_->satisfies(*formula) ? "true" : "false";
  }

  // The model itself, in the shape the task format gives an initial state.
  //
  // The summary above says how big the model is; this says what it is, which
  // is what a planner needs to replan from where a mission actually got to.
  // Without it a replan starts from the state grounding produced, which is the
  // one the divergence already disproved.
  //
  // It travels on this topic for the same reason the goal does: planning
  // happens inside the planner's own service callback, and a service call from
  // there can deadlock. Deployments that never replan can turn it off, since
  // for a large model it is the bulk of the message.
  if (publish_model_ && task_) {
    data += ", \"model\": " + state_to_json(*task_, *state_);
  }

  data += "}";

  std_msgs::msg::String msg;
  msg.data = std::move(data);
  state_pub_->publish(msg);
}

std::string EpistemicStateNode::goal_text() const
{
  if (!task_) {
    return "";
  }
  const auto & formula = goal_override_ ? goal_override_ : task_->goal;
  return formula ? render_formula(*task_, *formula) : std::string();
}

void EpistemicStateNode::load_task_callback(
  const std::shared_ptr<plansys2_epistemic_msgs::srv::LoadTask::Request> request,
  std::shared_ptr<plansys2_epistemic_msgs::srv::LoadTask::Response> response)
{
  try {
    // The interned formula registry is global and outlives any one task, so a
    // node that loads several would otherwise accumulate every formula it has
    // ever seen. Clearing it here is safe because the previous task and its
    // state are dropped first, and nothing else holds formulas from them.
    task_.reset();
    state_.reset();
    // The override was a formula in the previous task's vocabulary; keeping it
    // across a load would aim at symbols the new task may not have.
    goal_override_.reset();
    formula_registry_reset();

    if (!request->task_json.empty()) {
      TempTask temporary(request->task_json);
      task_ = load_task(temporary.path());
    } else if (!request->task_file.empty()) {
      task_ = load_task(request->task_file);
    } else if (!request->epddl_domain.empty() || !request->epddl_problem.empty()) {
      EpddlSpec spec;
      spec.domain = request->epddl_domain;
      spec.problem = request->epddl_problem;
      spec.libraries = request->epddl_libraries;

      const auto ground = grounder_.ground(spec);
      if (!ground.ok) {
        response->success = false;
        response->error = ground.error;
        RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", response->error.c_str());
        return;
      }
      TempTask temporary(ground.task_json);
      task_ = load_task(temporary.path());
    } else {
      response->success = false;
      response->error = "no task given: set task_json, task_file, or the EPDDL sources";
      return;
    }

    state_ = task_->init;
    response->success = true;
    response->num_worlds = state_->num_worlds;
    response->num_agents = static_cast<std::uint32_t>(task_->num_agents());
    response->num_actions = static_cast<std::uint32_t>(task_->num_actions());

    RCLCPP_INFO(
      get_logger(), "[epistemic_state] task loaded: %u worlds, %u agents, %u actions",
      response->num_worlds, response->num_agents, response->num_actions);
    publish_state();
  } catch (const std::exception & e) {
    task_.reset();
    state_.reset();
    response->success = false;
    response->error = std::string("could not load the task: ") + e.what();
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", response->error.c_str());
  }
}

void EpistemicStateNode::check_formula_callback(
  const std::shared_ptr<plansys2_epistemic_msgs::srv::CheckFormula::Request> request,
  std::shared_ptr<plansys2_epistemic_msgs::srv::CheckFormula::Response> response)
{
  if (!task_ || !state_) {
    response->success = false;
    response->error = "no task is loaded, so there is nothing to check against";
    return;
  }

  std::string error;
  const auto formula = parse_formula(*task_, request->formula, error);
  if (!formula) {
    response->success = false;
    response->error = error;
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", error.c_str());
    return;
  }

  response->success = true;
  response->holds = state_->satisfies(*formula);
}

void EpistemicStateNode::apply_action_callback(
  const std::shared_ptr<plansys2_epistemic_msgs::srv::ApplyAction::Request> request,
  std::shared_ptr<plansys2_epistemic_msgs::srv::ApplyAction::Response> response)
{
  if (!task_ || !state_) {
    response->success = false;
    response->error = "no task is loaded, so no action can be applied";
    return;
  }

  const auto it = task_->action_index.find(request->epistemic_action);
  if (it == task_->action_index.end()) {
    response->success = false;
    response->error =
      "the task has no action named '" + request->epistemic_action + "'";
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", response->error.c_str());
    return;
  }
  const Action & action = task_->actions[it->second];

  if (!action.applicable(*state_)) {
    // One inapplicable action is not like the others. A sensing action whose
    // outcome has already been applied is inapplicable precisely because it
    // worked: its own precondition says the agent does not yet know, and the
    // agent now knows. That happens whenever the observation reaches the state
    // before the executor's own update for the same action does -- perception
    // reports the moment a region resolves, and the behavior tree applies the
    // action a little later. Answering "the model and the world disagree"
    // there fails a policy that is in fact proceeding correctly.
    //
    // So an action that has already been applied is answered with the outcome
    // it produced, and the update is not repeated. Every other inapplicable
    // action is still refused: an inapplicable action means the model and the
    // world have diverged, and applying it would bury the divergence under a
    // state nothing produced.
    const auto seen = applied_outcomes_.find(request->epistemic_action);
    if (seen != applied_outcomes_.end()) {
      response->success = true;
      response->outcome = seen->second;
      RCLCPP_INFO(
        get_logger(),
        "[epistemic_state] '%s' was already applied; its outcome was %s",
        request->epistemic_action.c_str(),
        seen->second.empty() ? "(ontic)" : seen->second.c_str());
      return;
    }

    response->success = false;
    response->error =
      "'" + request->epistemic_action + "' is not applicable in the current "
      "epistemic state; the model and the world disagree";
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", response->error.c_str());
    return;
  }

  if (action.is_ontic()) {
    auto updated = product_update(*state_, action, task_->kd45);
    if (!updated) {
      response->success = false;
      response->error =
        std::string("the update produced no state: ") +
        prune_reason_name(updated.error());
      return;
    }
    state_ = std::move(*updated);
    response->success = true;
    response->outcome = "";   // an ontic action has nothing to observe
    applied_outcomes_[request->epistemic_action] = "";
  } else {
    auto outcomes = product_update_split(*state_, action, task_->kd45);
    if (outcomes.empty()) {
      response->success = false;
      response->error = "the sensing action produced no outcome at all";
      return;
    }

    // Which outcome occurred is a question about the world, not about the
    // model. When the model designates one world it already answers it; when
    // it designates several it genuinely does not know, and the observation
    // has to come from whoever did the sensing.
    std::size_t chosen = outcomes.size();
    if (!request->observed_outcome.empty()) {
      for (std::size_t i = 0; i < outcomes.size(); ++i) {
        if (outcomes[i].first < action.events.size() &&
          action.events[outcomes[i].first].name == request->observed_outcome)
        {
          chosen = i;
          break;
        }
      }
      if (chosen == outcomes.size()) {
        response->success = false;
        response->error =
          "'" + request->observed_outcome + "' is not an outcome '" +
          request->epistemic_action + "' can produce here";
        RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", response->error.c_str());
        return;
      }
    } else if (outcomes.size() == 1) {
      chosen = 0;
    } else {
      response->success = false;
      response->error =
        "'" + request->epistemic_action + "' has " +
        std::to_string(outcomes.size()) +
        " possible outcomes here and none was observed; the model cannot "
        "choose one for the world";
      return;
    }

    const auto event = outcomes[chosen].first;
    state_ = outcomes[chosen].second;
    response->success = true;
    response->outcome = event < action.events.size() ? action.events[event].name :
      "e" + std::to_string(event);
    applied_outcomes_[request->epistemic_action] = response->outcome;
  }

  response->num_worlds = state_->num_worlds;
  response->num_designated = static_cast<std::uint32_t>(state_->num_designated());

  RCLCPP_INFO(
    get_logger(), "[epistemic_state] applied %s%s%s: %u worlds, %u designated",
    request->epistemic_action.c_str(),
    response->outcome.empty() ? "" : " -> ",
    response->outcome.c_str(),
    response->num_worlds, response->num_designated);

  publish_state();
}

void EpistemicStateNode::get_goal_callback(
  const std::shared_ptr<plansys2_epistemic_msgs::srv::GetGoal::Request> request,
  std::shared_ptr<plansys2_epistemic_msgs::srv::GetGoal::Response> response)
{
  (void)request;

  if (!task_ || !state_) {
    response->success = false;
    response->error = "no task is loaded, so there is no goal to report";
    return;
  }

  const auto & formula = goal_override_ ? goal_override_ : task_->goal;

  response->success = true;
  response->from_task = !goal_override_;
  response->goal = goal_text();
  // A task with no goal at all is not an error — it is a model to ask
  // questions of rather than a problem to solve — but nothing holds of it.
  response->holds = formula ? state_->satisfies(*formula) : false;
}

void EpistemicStateNode::set_goal_callback(
  const std::shared_ptr<plansys2_epistemic_msgs::srv::SetGoal::Request> request,
  std::shared_ptr<plansys2_epistemic_msgs::srv::SetGoal::Response> response)
{
  if (!task_ || !state_) {
    response->success = false;
    response->error = "no task is loaded, so a goal has no vocabulary to be written in";
    return;
  }

  if (request->goal.empty()) {
    goal_override_.reset();
    response->success = true;
    response->holds = task_->goal ? state_->satisfies(*task_->goal) : false;
    RCLCPP_INFO(get_logger(), "[epistemic_state] goal restored to the task's own");
    publish_state();
    return;
  }

  std::string error;
  const auto formula = parse_formula(*task_, request->goal, error);
  if (!formula) {
    // Refusing a goal that names symbols the task does not have is the point
    // of the goal living here: the alternative is a planning request that
    // fails much later with nothing to point at.
    response->success = false;
    response->error = error;
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", error.c_str());
    return;
  }

  goal_override_ = formula;

  response->success = true;
  response->holds = state_->satisfies(*formula);

  RCLCPP_INFO(
    get_logger(), "[epistemic_state] goal set to %s (%s)",
    goal_text().c_str(), response->holds ? "already holds" : "does not hold yet");

  publish_state();
}

void EpistemicStateNode::announce_callback(
  const std::shared_ptr<plansys2_epistemic_msgs::srv::Announce::Request> request,
  std::shared_ptr<plansys2_epistemic_msgs::srv::Announce::Response> response)
{
  if (!task_ || !state_) {
    response->success = false;
    response->error = "no task is loaded, so there is no model to announce into";
    return;
  }

  std::string error;
  const auto formula = parse_formula(*task_, request->formula, error);
  if (!formula) {
    response->success = false;
    response->error = error;
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", error.c_str());
    return;
  }

  // A public announcement of phi keeps exactly the worlds where phi holds. The
  // extension has to be copied out: the cache arena it lives in is invalidated
  // by the next model-checking call, and restrict_state does several.
  std::vector<bits::Word> keep;
  state_->sat_copy(*formula, keep);

  std::vector<WorldIdx> remap;
  auto restricted = restrict_state(
    *state_, bits::ConstWordSpan{keep.data(), keep.size()}, remap);

  if (restricted.num_designated() == 0) {
    // Announcing something false at every world the agents consider possible
    // is not a smaller model, it is no model: nothing would be consistent with
    // what was just said. Refusing says the announcement and the state
    // disagree, which is a reason to reload, not to keep going.
    response->success = false;
    response->error =
      "announcing '" + request->formula + "' would leave no possible world: "
      "it is false everywhere the state considers possible";
    RCLCPP_ERROR(get_logger(), "[epistemic_state] %s", response->error.c_str());
    return;
  }

  const auto worlds_before = state_->num_worlds;
  state_ = std::move(restricted);

  response->success = true;
  response->num_worlds = state_->num_worlds;
  response->num_designated = static_cast<std::uint32_t>(state_->num_designated());

  RCLCPP_INFO(
    get_logger(), "[epistemic_state] announced %s: %u worlds -> %u",
    request->formula.c_str(), worlds_before, state_->num_worlds);

  publish_state();
}

}  // namespace plansys2
