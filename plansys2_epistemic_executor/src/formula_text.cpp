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

#include "plansys2_epistemic_executor/formula_text.hpp"

#include <string>
#include <vector>

#include "plansys2_epistemic_planner/formula.hpp"

namespace plansys2
{

namespace
{

/// A cursor over the text, with just enough to read the s-expressions
/// render_formula writes. Pulling in a general s-expression library for a
/// grammar this small would be more to keep working than to keep here.
class Cursor
{
public:
  Cursor(const std::string & text, const PlanningTask & task, std::string & error)
  : text_(text), task_(task), error_(error) {}

  FormulaPtr parse()
  {
    auto formula = parse_formula();
    if (!formula) {
      return nullptr;
    }
    skip_space();
    if (at_ != text_.size()) {
      return fail("trailing text after the formula: '" + text_.substr(at_) + "'");
    }
    return formula;
  }

private:
  FormulaPtr fail(const std::string & why)
  {
    if (error_.empty()) {
      error_ = why + " (in '" + text_ + "')";
    }
    return nullptr;
  }

  void skip_space()
  {
    while (at_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[at_]))) {
      ++at_;
    }
  }

  bool eat(char c)
  {
    skip_space();
    if (at_ < text_.size() && text_[at_] == c) {
      ++at_;
      return true;
    }
    return false;
  }

  /// A bare word: an atom name, an agent name, or a connective.
  std::string token()
  {
    skip_space();
    const std::size_t start = at_;
    while (at_ < text_.size() && text_[at_] != '(' && text_[at_] != ')' &&
      !std::isspace(static_cast<unsigned char>(text_[at_])))
    {
      ++at_;
    }
    return text_.substr(start, at_ - start);
  }

  bool agent(const std::string & name, AgentIdx & out)
  {
    const auto it = task_.agent_index.find(name);
    if (it == task_.agent_index.end()) {
      fail("no agent named '" + name + "' in this task");
      return false;
    }
    out = it->second;
    return true;
  }

  FormulaPtr parse_formula()
  {
    skip_space();
    if (at_ >= text_.size()) {
      return fail("the formula ended early");
    }

    if (text_[at_] != '(') {
      const auto name = token();
      if (name.empty()) {
        return fail("expected an atom");
      }
      const auto it = task_.atom_index.find(name);
      if (it == task_.atom_index.end()) {
        return fail("no atom named '" + name + "' in this task");
      }
      return Formula::make_atom(it->second);
    }

    eat('(');
    const auto head = token();

    if (head == "true") {
      return eat(')') ? Formula::make_top() : fail("expected ')' after true");
    }
    if (head == "false") {
      return eat(')') ? Formula::make_bot() : fail("expected ')' after false");
    }

    if (head == "not") {
      auto inner = parse_formula();
      if (!inner) {
        return nullptr;
      }
      return eat(')') ? Formula::make_not(inner) : fail("expected ')' to close not");
    }

    if (head == "and" || head == "or") {
      std::vector<FormulaPtr> parts;
      while (true) {
        skip_space();
        if (at_ >= text_.size()) {
          return fail("expected ')' to close " + head);
        }
        if (text_[at_] == ')') {
          ++at_;
          break;
        }
        auto part = parse_formula();
        if (!part) {
          return nullptr;
        }
        parts.push_back(part);
      }
      if (parts.empty()) {
        // An empty conjunction is trivially true and an empty disjunction
        // trivially false, but writing one is far more likely to be a mistake
        // than an intention, and silently agreeing would hide it.
        return fail(head + " needs at least one operand");
      }
      return head == "and" ? Formula::make_and(parts) : Formula::make_or(parts);
    }

    // K and B are one modality under two frames; see the header.
    if (head == "K" || head == "B" || head == "Kw") {
      AgentIdx who = 0;
      if (!agent(token(), who)) {
        return nullptr;
      }
      auto inner = parse_formula();
      if (!inner) {
        return nullptr;
      }
      if (!eat(')')) {
        return fail("expected ')' to close " + head);
      }
      return head == "Kw" ? Formula::make_kw(who, inner) : Formula::make_belief(who, inner);
    }

    if (head == "C") {
      if (!eat('(')) {
        return fail("common knowledge needs a group, as (C (a b) formula)");
      }
      std::vector<AgentIdx> group;
      while (!eat(')')) {
        const auto name = token();
        if (name.empty()) {
          return fail("expected ')' to close the group");
        }
        AgentIdx who = 0;
        if (!agent(name, who)) {
          return nullptr;
        }
        group.push_back(who);
      }
      if (group.empty()) {
        return fail("common knowledge among nobody is not a condition");
      }
      auto inner = parse_formula();
      if (!inner) {
        return nullptr;
      }
      return eat(')') ? Formula::make_common(group, inner) : fail("expected ')' to close C");
    }

    return fail("unknown connective '" + head + "'");
  }

  const std::string & text_;
  const PlanningTask & task_;
  std::string & error_;
  std::size_t at_{0};
};

}  // namespace

FormulaPtr parse_formula(
  const PlanningTask & task, const std::string & text, std::string & error)
{
  error.clear();
  Cursor cursor(text, task, error);
  auto formula = cursor.parse();
  if (!formula && error.empty()) {
    error = "could not parse '" + text + "'";
  }
  return formula;
}

}  // namespace plansys2
