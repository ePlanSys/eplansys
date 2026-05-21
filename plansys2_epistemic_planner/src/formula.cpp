#include "plansys2_epistemic_planner/formula.hpp"

FormulaPtr Formula::make_top() {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Top;
    return f;
}
FormulaPtr Formula::make_bot() {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Bot;
    return f;
}
FormulaPtr Formula::make_atom(AtomIdx a) {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Atom;
    f->atom = a;
    return f;
}
FormulaPtr Formula::make_not(FormulaPtr child) {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Not;
    f->children = {child};
    return f;
}
FormulaPtr Formula::make_and(std::vector<FormulaPtr> fs) {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::And;
    f->children = std::move(fs);
    return f;
}
FormulaPtr Formula::make_or(std::vector<FormulaPtr> fs) {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Or;
    f->children = std::move(fs);
    return f;
}
FormulaPtr Formula::make_belief(AgentIdx ag, FormulaPtr child) {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Belief;
    f->agent = ag;
    f->children = {child};
    return f;
}
FormulaPtr Formula::make_common(std::vector<AgentIdx> grp, FormulaPtr child) {
    auto f = std::make_shared<Formula>();
    f->kind = FormulaKind::Common;
    f->group = std::move(grp);
    f->children = {child};
    return f;
}
    
FormulaPtr Formula::make_kw(AgentIdx ag, FormulaPtr child) {
    // <Kw.i>φ ≡ [i]φ ∨ [i]¬φ
    auto f = std::make_shared<Formula>();
    f->kind     = FormulaKind::Kw;
    f->agent    = ag;
    f->children = {child};
    return f;
}
