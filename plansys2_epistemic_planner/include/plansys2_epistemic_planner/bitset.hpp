#pragma once

// Flat word-array bit sets.
//
// Every set-valued object in the planner (world valuations, accessibility rows,
// designated sets, satisfaction sets) is a contiguous run of 64-bit words inside
// a larger arena owned by the enclosing structure. Nothing here owns storage;
// these are free functions over std::span so that a single std::vector<Word>
// allocation can back an entire Kripke model.
//
// Rationale: the previous representation used std::unordered_set<uint32_t> per
// world and per (agent, world) pair. A 512-world, 5-agent model therefore held
// ~2600 independent hash tables, each with its own bucket array and per-element
// node allocation. Modal operators over that representation are pointer chases;
// over a bit matrix they are word-parallel AND/ANDNOT tests, 64 worlds at a time.

#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bits {

using Word = std::uint64_t;

inline constexpr std::size_t kWordBits = 64;

using WordSpan      = std::span<Word>;
using ConstWordSpan = std::span<const Word>;

[[nodiscard]] constexpr std::size_t words_for(std::size_t n) noexcept {
    return (n + kWordBits - 1) / kWordBits;
}

// Mask selecting the bits of the final word that correspond to real elements.
// Every operation that can introduce ones out of range (complement, fill) must
// re-apply this, otherwise padding bits leak into popcounts and equality tests.
[[nodiscard]] constexpr Word tail_mask(std::size_t n) noexcept {
    const std::size_t r = n % kWordBits;
    return r == 0 ? ~Word{0} : ((Word{1} << r) - Word{1});
}

constexpr void set(WordSpan s, std::size_t i) noexcept {
    s[i / kWordBits] |= Word{1} << (i % kWordBits);
}

constexpr void reset(WordSpan s, std::size_t i) noexcept {
    s[i / kWordBits] &= ~(Word{1} << (i % kWordBits));
}

[[nodiscard]] constexpr bool test(ConstWordSpan s, std::size_t i) noexcept {
    return ((s[i / kWordBits] >> (i % kWordBits)) & Word{1}) != 0;
}

constexpr void clear_all(WordSpan s) noexcept {
    for (Word& w : s) w = 0;
}

// Sets every bit below `n` and clears the padding above it.
constexpr void fill_all(WordSpan s, std::size_t n) noexcept {
    for (Word& w : s) w = ~Word{0};
    if (!s.empty()) s.back() &= tail_mask(n);
}

[[nodiscard]] constexpr bool empty(ConstWordSpan s) noexcept {
    for (Word w : s)
        if (w != 0) return false;
    return true;
}

[[nodiscard]] constexpr bool any(ConstWordSpan s) noexcept { return !empty(s); }

[[nodiscard]] constexpr std::size_t count(ConstWordSpan s) noexcept {
    std::size_t n = 0;
    for (Word w : s) n += static_cast<std::size_t>(std::popcount(w));
    return n;
}

[[nodiscard]] constexpr bool equal(ConstWordSpan a, ConstWordSpan b) noexcept {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] != b[i]) return false;
    return true;
}

// a ⊆ b
[[nodiscard]] constexpr bool subset_of(ConstWordSpan a, ConstWordSpan b) noexcept {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] & ~b[i]) return false;
    return true;
}

// a ∩ b ≠ ∅
[[nodiscard]] constexpr bool intersects(ConstWordSpan a, ConstWordSpan b) noexcept {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] & b[i]) return true;
    return false;
}

constexpr void copy_from(WordSpan dst, ConstWordSpan src) noexcept {
    for (std::size_t i = 0; i < dst.size(); ++i) dst[i] = src[i];
}

constexpr void or_into(WordSpan dst, ConstWordSpan src) noexcept {
    for (std::size_t i = 0; i < dst.size(); ++i) dst[i] |= src[i];
}

constexpr void and_into(WordSpan dst, ConstWordSpan src) noexcept {
    for (std::size_t i = 0; i < dst.size(); ++i) dst[i] &= src[i];
}

constexpr void andnot_into(WordSpan dst, ConstWordSpan src) noexcept {
    for (std::size_t i = 0; i < dst.size(); ++i) dst[i] &= ~src[i];
}

// dst := ¬src, restricted to the first `n` bits.
constexpr void complement_into(WordSpan dst, ConstWordSpan src, std::size_t n) noexcept {
    for (std::size_t i = 0; i < dst.size(); ++i) dst[i] = ~src[i];
    if (!dst.empty()) dst.back() &= tail_mask(n);
}

// Visits set bits in ascending index order. Order matters: several heuristics
// truncate their sampling after a fixed number of worlds, so a deterministic
// traversal is required for the planner to be reproducible. The old
// unordered_set-based traversal was not.
template <std::invocable<std::uint32_t> F>
constexpr void for_each(ConstWordSpan s, F&& f) {
    for (std::size_t wi = 0; wi < s.size(); ++wi) {
        Word w = s[wi];
        while (w) {
            const auto b = static_cast<std::uint32_t>(std::countr_zero(w));
            f(static_cast<std::uint32_t>(wi * kWordBits) + b);
            w &= w - 1;   // clear lowest set bit
        }
    }
}

// As for_each, but stops early when the callback returns false.
// Returns false iff the traversal was cut short.
template <std::predicate<std::uint32_t> F>
constexpr bool for_each_until(ConstWordSpan s, F&& f) {
    for (std::size_t wi = 0; wi < s.size(); ++wi) {
        Word w = s[wi];
        while (w) {
            const auto b = static_cast<std::uint32_t>(std::countr_zero(w));
            if (!f(static_cast<std::uint32_t>(wi * kWordBits) + b)) return false;
            w &= w - 1;
        }
    }
    return true;
}

// ── Mixing ────────────────────────────────────────────────────────────────
//
// splitmix64. The previous code combined raw uint32 values with a boost-style
// hash_combine, but libstdc++ implements std::hash<uint32_t> as the identity,
// so the input to the combiner had no avalanche at all. Bisimulation class
// assignment and the closed list both depend on collision behaviour, so the
// mixer is now a real finalizer.

[[nodiscard]] constexpr Word mix64(Word x) noexcept {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

} // namespace bits
