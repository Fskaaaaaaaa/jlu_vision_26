#pragma once

namespace rm_ultra_tools {

inline int limitIndex(int idx, int max) { return (idx % max + max) % max; }

struct LoopIndex {
  explicit LoopIndex(int max) : max_idx(max), current_idx(0) {}
  LoopIndex &operator++() {
    current_idx = limitIndex(current_idx + 1, max_idx);
    return *this;
  }
  LoopIndex operator++(int) {
    LoopIndex tmp(*this);
    ++(*this);
    return tmp;
  }
  LoopIndex &operator--() {
    current_idx = limitIndex(current_idx - 1, max_idx);
    return *this;
  }
  LoopIndex operator--(int) {
    LoopIndex tmp(*this);
    --(*this);
    return tmp;
  }
  operator int() const { return current_idx; }
  int max_idx;
  int current_idx;
};

} // namespace rm_ultra_tools
