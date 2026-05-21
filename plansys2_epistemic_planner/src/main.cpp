#include "plansys2_epistemic_planner/parser.hpp"
#include "plansys2_epistemic_planner/validator.hpp"
#include "plansys2_epistemic_planner/search.hpp"
#include "plansys2_epistemic_planner/heuristic.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <algorithm>

static void write_plan_tree(std::ostream& out,
                            const std::shared_ptr<PlanNode>& node,
                            int indent = 0) {
    std::string pad(indent * 2, ' ');
    std::string pad2((indent + 1) * 2, ' ');
    std::string pad3((indent + 2) * 2, ' ');

    if (!node) {
        out << "null";
        return;
    }

    out << "{\n";
    out << pad2 << "\"action\": \"" << node->action << "\",\n";
    out << pad2 << "\"branches\": [\n";

    for (size_t i = 0; i < node->branches.size(); i++) {
        auto& [eid, child] = node->branches[i];

        out << pad3 << "{\n";
        out << pad3 << "  \"event\": " << eid << ",\n";
        out << pad3 << "  \"subtree\": ";
        write_plan_tree(out, child, indent + 3);
        out << "\n" << pad3 << "}";

        if (i + 1 < node->branches.size())
            out << ",";

        out << "\n";
    }

    out << pad2 << "]\n";
    out << pad << "}";
}

static void write_linear_plan(std::ostream& out,
                              const SearchResult& result) {
    out << "[";

    for (size_t i = 0; i < result.plan.size(); i++) {
        if (i > 0)
            out << ", ";

        out << "\"" << result.plan[i] << "\"";
    }

    out << "]\n";
}

enum class Strategy {
    GBFS,
    EHC,
    AOSTAR
};

static bool has_sensing_actions(const PlanningTask& task) {
    for (auto& action : task.actions) {
        if (action.designated_events.size() > 1)
            return true;
    }

    return false;
}

static int goal_modal_depth(const Formula& f) {
    switch (f.kind) {

        case FormulaKind::Belief:
        case FormulaKind::Common:
        case FormulaKind::Kw:
            return 1 + goal_modal_depth(*f.children[0]);

        case FormulaKind::And:
        case FormulaKind::Or: {
            int d = 0;

            for (auto& c : f.children)
                d = std::max(d, goal_modal_depth(*c));

            return d;
        }

        default:
            return 0;
    }
}

static Strategy select_strategy(const PlanningTask& task,
                                const std::string& heuristic_name) {

    bool sensing =
        has_sensing_actions(task);

    int depth =
        goal_modal_depth(*task.goal);

    size_t designated =
        task.init.designated.size();

    size_t worlds =
        task.init.worlds.size();

    size_t actions =
        task.actions.size();

    bool ed =
        heuristic_name == "ed";

    // Small sensing domains are exactly what AO* handles best.
    // Coin, muddy children, etc.
    if (sensing) {

        // epistemic-distance heuristic is intended for conditional planning
        if (ed && designated <= 16)
            return Strategy::AOSTAR;

        // nested epistemic goals require branching reasoning
        if (depth >= 2 && designated <= 32)
            return Strategy::AOSTAR;

        // sensing domains
        if (designated <= 8 && actions <= 32)
            return Strategy::AOSTAR;

        // large sensing spaces
        return Strategy::GBFS;
    }

    // Non-sensing domains

    // Tiny deterministic tasks
    if (designated <= 4 &&
        worlds <= 16 &&
        actions <= 12 &&
        depth <= 1)
    {
        return Strategy::EHC;
    }

    // Large state spaces favor GBFS
    if (worlds > 16)
        return Strategy::GBFS;

    // Many actions -> GBFS
    if (actions > 12)
        return Strategy::GBFS;

    return Strategy::EHC;
}

static const char* strategy_name(Strategy s) {
    switch (s) {

        case Strategy::AOSTAR:
            return "AO* (conditional, auto-selected)";

        case Strategy::EHC:
            return "EHC (enforced hill climbing, auto-selected)";

        default:
            return "GBFS (auto-selected)";
    }
}

static void usage(const char* prog) {
    std::cerr
        << "Usage:\n"
        << "  " << prog
        << " --task <task.json> --plan <plan.json> [options]\n"
        << "\n"
        << "Options:\n"
        << "  --task         Path to grounded JSON task\n"
        << "  --plan         Output plan file\n"
        << "  --heuristic    ug | ed | ks | wc\n"
        << "  --limit        Max nodes / max depth (0 = unlimited)\n"
        << "  --timeout      Timeout in seconds (AO* only)\n"
        << "  --ehc          Force EHC\n"
        << "  --gbfs         Force GBFS\n"
        << "  --conditional  Force AO*\n"
        << "  --help         Show this message\n";
}

int main(int argc, char* argv[]) {

    std::string task_path;
    std::string plan_path;

    std::string heuristic_name = "ug";

    size_t limit = 0;
    size_t timeout_secs = 0;

    bool force_conditional = false;
    bool force_ehc = false;
    bool force_gbfs = false;

    for (int i = 1; i < argc; i++) {

        std::string arg = argv[i];

        if (arg == "--task" && i + 1 < argc) {
            task_path = argv[++i];
        }

        else if (arg == "--plan" && i + 1 < argc) {
            plan_path = argv[++i];
        }

        else if (arg == "--heuristic" && i + 1 < argc) {
            heuristic_name = argv[++i];
        }

        else if (arg == "--limit" && i + 1 < argc) {
            limit = static_cast<size_t>(std::stoul(argv[++i]));
        }

        else if (arg == "--timeout" && i + 1 < argc) {
            timeout_secs = static_cast<size_t>(std::stoul(argv[++i]));
        }

        else if (arg == "--conditional") {
            force_conditional = true;
        }

        else if (arg == "--ehc") {
            force_ehc = true;
        }

        else if (arg == "--gbfs") {
            force_gbfs = true;
        }

        else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return 0;
        }

        else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    if (task_path.empty() || plan_path.empty()) {
        std::cerr << "Error: --task and --plan are required.\n";
        usage(argv[0]);
        return 1;
    }

    PlanningTask task;

    try {
        task = load_task(task_path);
    }

    catch (const std::exception& e) {
        std::cerr << "Error loading task: " << e.what() << "\n";
        return 1;
    }

    std::unique_ptr<Heuristic> h;

    if (heuristic_name == "ug") {
        h = std::make_unique<UnsatisfiedGoalHeuristic>();
        std::cerr << "[main] Heuristic: unsatisfied-goal\n";
    }

    else if (heuristic_name == "ed") {
        h = std::make_unique<EpistemicDistanceHeuristic>();
        std::cerr << "[main] Heuristic: epistemic-distance\n";
    }

    else if (heuristic_name == "ks") {
        h = std::make_unique<KnowledgeSpreadHeuristic>();
        std::cerr << "[main] Heuristic: knowledge-spread\n";
    }

    else {
        h = std::make_unique<WorldCountHeuristic>();
        std::cerr << "[main] Heuristic: world-count\n";
    }

    Strategy strategy;

    if (force_conditional)
        strategy = Strategy::AOSTAR;

    else if (force_ehc)
        strategy = Strategy::EHC;

    else if (force_gbfs)
        strategy = Strategy::GBFS;

    else {
        strategy =
            select_strategy(
                task,
                heuristic_name
            );

        std::cerr << "[main] Strategy: "
                  << strategy_name(strategy) << "\n";
    }

    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    TimePoint deadline =
        timeout_secs > 0
        ? Clock::now() + std::chrono::seconds(timeout_secs)
        : TimePoint::max();

    std::ofstream out(plan_path);

    if (!out.is_open()) {
        std::cerr << "Error: cannot open output file: "
                  << plan_path << "\n";
        return 1;
    }

    if (strategy == Strategy::AOSTAR) {

        std::cerr << "[main] Mode: AO*\n";

        auto result =
            aostar::search(task, *h, limit, deadline);

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }

        write_plan_tree(out, result->plan_tree);
        out << "\n";

        std::cerr
            << "[main] Conditional plan written to "
            << plan_path << "\n";

        auto vr = validate(task, result->plan_tree);

        if (vr.valid) {
            std::cerr
                << "[validator] OK — "
                << vr.leaves_reached << " leaves, "
                << vr.branches_checked << " branches checked\n";
        }

        else {
            std::cerr
                << "[validator] FAILED — "
                << vr.error << "\n";
        }
    }

    else if (strategy == Strategy::EHC) {

        std::cerr << "[main] Mode: EHC\n";

        auto result =
            ehc::search(task, *h, limit);

        if (!result) {
            std::cerr
                << "[main] EHC failed — falling back to GBFS\n";

            result = gbfs::search(task, *h, limit);
        }

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }

        write_linear_plan(out, *result);

        std::cerr
            << "[main] Plan written to "
            << plan_path << "\n";
    }

    else {

        std::cerr << "[main] Mode: GBFS\n";

        auto result =
            gbfs::search(task, *h, limit);

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }

        write_linear_plan(out, *result);

        std::cerr
            << "[main] Plan written to "
            << plan_path << "\n";
    }

    return 0;
}