#pragma once
#include <cstddef>
#include <limits>

// A world cap policy decides at runtime whether a proposed product update is
// allowed to proceed given the raw (pre-contraction, pre-precondition-filter)
// world count |W| * |E|.
//
// The policy object is constructed once per search from PlanningTask and
// passed by const-ref to every product_update call. This avoids threading
// a magic constant through every call site while keeping the decision
// co-located with the domain knowledge that informs it.
//
// Two concrete policies:
//
//   BoundedWorldCap(n)  —> reject if |W|*|E| > n. Used for domains where
//                         nil-event accumulation causes exponential blowup
//                         (e.g. Gossip without partial-obs detection, AO*
//                         branches in KD45 sensing tasks).
//
//   UnboundedWorldCap   —> always allow. Used for partial-obs domains where
//                         the world count grows linearly (~1.5x/step) and
//                         bisim contraction keeps it bounded in practice.
//
// The free function make_world_cap_policy(task) is the single authoritative
// place that maps task properties to a policy — no call site needs to
// inspect task.partial_obs or hardcode 512.

struct WorldCapPolicy {
    // Returns true iff the proposed update should proceed.
    bool allows(size_t worlds, size_t events) const noexcept {
        return worlds * events <= cap_;
    }

    size_t cap() const noexcept { return cap_; }

private:
    friend WorldCapPolicy make_world_cap_policy(bool partial_obs);
    explicit WorldCapPolicy(size_t cap) : cap_(cap) {}

    // 512 is intentionally loose: the largest initial model in the benchmark
    // suite has 32 worlds and |E|=2, so the first product step is 64 raw
    // worlds. A k-step plan through contracting actions never exceeds ~64
    // post-contraction worlds; the cap fires only on diverging paths where
    // nil is accumulating without progress
    static constexpr size_t kDefaultCap = 512;

    size_t cap_;
};

inline WorldCapPolicy make_world_cap_policy(bool partial_obs) {
    return WorldCapPolicy{
        partial_obs
            ? std::numeric_limits<size_t>::max()
            : WorldCapPolicy::kDefaultCap
    };
}