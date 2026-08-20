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

#include "plansys2_epistemic_planner/action_mapping.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace plansys2
{

namespace
{

/// Split "move-kitchen_r1_corridor" into {"move-kitchen", "r1", "corridor"}.
/// The head keeps any hyphens: PDDL action names may contain them, and plank
/// uses them inside a single grounded name, so only the underscore separates
/// the name from its parameters.
std::vector<std::string> split_underscores(const std::string & grounded)
{
  std::vector<std::string> parts;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= grounded.size(); ++i) {
    if (i == grounded.size() || grounded[i] == '_') {
      parts.push_back(grounded.substr(start, i - start));
      start = i + 1;
    }
  }
  return parts;
}

}  // namespace

ActionMapping ActionMapping::conventional()
{
  ActionMapping mapping;
  mapping.conventional_ = true;
  return mapping;
}

ActionMapping ActionMapping::load(const std::string & path)
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("action mapping file does not exist: " + path);
  }

  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("action mapping file could not be opened: " + path);
  }

  nlohmann::json j;
  try {
    in >> j;
  } catch (const nlohmann::json::exception & e) {
    throw std::runtime_error(
            "action mapping file is not valid JSON: " + path + ": " + e.what());
  }

  if (!j.is_object()) {
    throw std::runtime_error(
            "action mapping file must be a JSON object keyed by grounded action "
            "name: " + path);
  }

  ActionMapping mapping;

  for (const auto & [grounded, value] : j.items()) {
    MappedAction mapped;

    if (value.is_string()) {
      mapped.action = value.get<std::string>();
    } else if (value.is_object()) {
      if (!value.contains("action") || !value.at("action").is_string()) {
        throw std::runtime_error(
                "action mapping entry '" + grounded + "' must carry a string "
                "\"action\"");
      }
      mapped.action = value.at("action").get<std::string>();

      if (value.contains("duration")) {
        if (!value.at("duration").is_number()) {
          throw std::runtime_error(
                  "action mapping entry '" + grounded + "' has a non-numeric "
                  "\"duration\"");
        }
        mapped.duration = value.at("duration").get<float>();
        if (!(mapped.duration > 0.0f)) {
          throw std::runtime_error(
                  "action mapping entry '" + grounded + "' has a non-positive "
                  "\"duration\"; the executor schedules on it");
        }
      }
    } else {
      throw std::runtime_error(
              "action mapping entry '" + grounded + "' must be a string or an "
              "object");
    }

    if (mapped.action.empty()) {
      throw std::runtime_error(
              "action mapping entry '" + grounded + "' maps to an empty action");
    }

    mapping.table_.emplace(grounded, std::move(mapped));
  }

  return mapping;
}

std::optional<MappedAction> ActionMapping::translate(const std::string & grounded) const
{
  if (!conventional_) {
    const auto it = table_.find(grounded);
    if (it == table_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  const auto parts = split_underscores(grounded);
  if (parts.empty() || parts.front().empty()) {
    return std::nullopt;
  }

  MappedAction mapped;
  mapped.action = "(" + parts.front();
  for (std::size_t i = 1; i < parts.size(); ++i) {
    if (parts[i].empty()) {
      return std::nullopt;    // "ask__c1": a parameter that is not there
    }
    mapped.action += " " + parts[i];
  }
  mapped.action += ")";
  return mapped;
}

}  // namespace plansys2
