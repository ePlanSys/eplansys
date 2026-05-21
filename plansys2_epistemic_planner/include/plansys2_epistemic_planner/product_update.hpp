#pragma once
#include "state.hpp"
#include "action.hpp"
#include <vector>
#include <unordered_map>

// Internal result of a DEL product update: the updated state together with
// the (world,event)->new_world_id mapping that was used to build it.
//
// Keeping the mapping here lets product_update_split reuse it verbatim
// instead of re-deriving it independently (which is the root cause of the
// world-identity inconsistency bug).
struct ProductUpdateResult {
    EpistemicState state;
    // Maps encode_pair(original_world_id, event_id) -> new world id in state.
    // After KD45 seriality repair the new world ids are compacted so that
    // worlds[i].id == i always holds; the map already reflects the compacted ids.
    std::unordered_map<uint64_t, WorldIdx> pair_to_idx;
};

// Compute s ⊗ a  (DEL product update).
//
// Returns the updated epistemic state, or std::nullopt if the action
// is not applicable (no designated event's precondition holds in any
// designated world).
std::optional<EpistemicState> product_update(const EpistemicState& s,
                                              const Action& a,
                                              bool enforce_kd45 = false);

// Internal variant that also returns the pair_to_idx mapping.
// Used by product_update_split to guarantee identity consistency.
std::optional<ProductUpdateResult>
product_update_with_map(const EpistemicState& s, const Action& a,
                        bool enforce_kd45 = false);

// Sensing variant: returns one EpistemicState per designated event.
//
// For ontic actions (single designated event) this returns a vector of
// size 1, identical to what product_update returns.
// For sensing actions (multiple designated events) this partitions the
// product-update result by which designated event fired, producing one
// sub-state per event. Each sub-state shares the same world set and
// accessibility as the full product update but has a distinct designated
// set — the worlds (w,e) where e is exactly that one designated event.
//
// Returns an empty vector if the action is not applicable.
// Each element is tagged with the EventIdx that produced it.
std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a,
                     bool enforce_kd45 = false);