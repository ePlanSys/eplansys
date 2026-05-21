#pragma once
#include "plansys2_epistemic_planner/types.hpp"

enum class FormulaKind {
    Top,        // true
    Bot,        // false
    Atom,       // p
    Not,        // ¬φ
    And,        // φ ∧ ψ
    Or,         // φ ∨ ψ
    Belief,     // [i]φ   (B_i φ)
    Common,     // [C.G]φ (C_G φ)
    Kw,         // <Kw.i>φ  ≡  [i]φ ∨ [i]¬φ
};

struct Formula {
    FormulaKind kind;

    // Atom
    AtomIdx atom{0};

    // Belief / Kw
    AgentIdx agent{0};

    // Common knowledge — set of agent indices
    std::vector<AgentIdx> group;
    std::vector<FormulaPtr> children;

    static FormulaPtr make_top();
    static FormulaPtr make_bot();
    static FormulaPtr make_atom(AtomIdx a);
    static FormulaPtr make_not(FormulaPtr f);
    static FormulaPtr make_and(std::vector<FormulaPtr> fs);
    static FormulaPtr make_or(std::vector<FormulaPtr> fs);
    static FormulaPtr make_belief(AgentIdx ag, FormulaPtr f);
    static FormulaPtr make_common(std::vector<AgentIdx> grp, FormulaPtr f);
    static FormulaPtr make_kw(AgentIdx ag, FormulaPtr f);
};
