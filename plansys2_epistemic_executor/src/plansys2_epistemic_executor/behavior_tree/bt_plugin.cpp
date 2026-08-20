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

// Exposes the epistemic nodes as a BehaviorTree.CPP plugin, so a host can pick
// them up by loading a library rather than by linking against this package and
// naming each node in its factory. That is what keeps plansys2_executor free
// of any dependency on the epistemic stack while still being able to run its
// trees.

#include "behaviortree_cpp/bt_factory.h"

#include "plansys2_epistemic_executor/behavior_tree/epistemic_nodes.hpp"

BT_REGISTER_NODES(factory)
{
  plansys2::register_epistemic_nodes(factory);
}
