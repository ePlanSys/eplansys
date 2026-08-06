#pragma once

// A result type that carries *why* a computation produced nothing.
//
// product_update previously returned std::optional and collapsed three very
// different situations into a bare nullopt: the action was inapplicable in the
// state, the pre-contraction world bound was exceeded, or KD45 seriality repair
// removed every designated world. Search statistics could not tell them apart,
// which made it impossible to distinguish "the domain has no plan here" from
// "the planner pruned a branch it was allowed to prune".
//
// std::expected is the natural vocabulary type but is unavailable before
// libstdc++ 13, so this is a minimal stand-in with the same shape. When
// <expected> is present it is used directly.

#include <cstdint>
#include <optional>
#include <utility>

#if __has_include(<expected>)
#  include <expected>
#endif

enum class PruneReason : std::uint8_t {
    None = 0,
    Inapplicable,       // no designated event's precondition held anywhere
    WorldCapExceeded,   // |W| · |E| above the configured bound
    NonSerial,          // KD45 repair emptied the designated set
};

[[nodiscard]] constexpr const char* prune_reason_name(PruneReason r) noexcept {
    switch (r) {
        case PruneReason::None:             return "none";
        case PruneReason::Inapplicable:     return "inapplicable";
        case PruneReason::WorldCapExceeded: return "world-cap";
        case PruneReason::NonSerial:        return "non-serial";
    }
    return "?";
}

#if defined(__cpp_lib_expected) && __cpp_lib_expected >= 202202L

template <class T>
using Outcome = std::expected<T, PruneReason>;

template <class T>
[[nodiscard]] Outcome<T> ok(T v) { return Outcome<T>{std::move(v)}; }

template <class T>
[[nodiscard]] Outcome<T> pruned(PruneReason r) { return Outcome<T>{std::unexpect, r}; }

#else

template <class T>
class Outcome {
public:
    Outcome(T v) : value_(std::move(v)) {} 
    explicit Outcome(PruneReason r) : reason_(r) {}

    [[nodiscard]] bool has_value() const noexcept { return value_.has_value(); }
    explicit operator bool() const noexcept { return has_value(); }

    [[nodiscard]] T&        value()       &  noexcept { return *value_; }
    [[nodiscard]] const T&  value() const &  noexcept { return *value_; }
    [[nodiscard]] T&&       value()       && noexcept { return std::move(*value_); }

    [[nodiscard]] T&        operator*()       noexcept { return *value_; }
    [[nodiscard]] const T&  operator*() const noexcept { return *value_; }
    [[nodiscard]] T*        operator->()       noexcept { return &*value_; }
    [[nodiscard]] const T*  operator->() const noexcept { return &*value_; }

    [[nodiscard]] PruneReason error() const noexcept { return reason_; }

private:
    std::optional<T> value_{};
    PruneReason      reason_{PruneReason::None};
};

template <class T>
[[nodiscard]] Outcome<T> ok(T v) { return Outcome<T>{std::move(v)}; }

template <class T>
[[nodiscard]] Outcome<T> pruned(PruneReason r) { return Outcome<T>{r}; }

#endif
