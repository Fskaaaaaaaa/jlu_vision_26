#include "thread_safe_any_map.hpp"
#include <mutex>
#include <shared_mutex>

namespace tools {

AnyMap::AnyMap(
    std::initializer_list<std::pair<const std::string, std::any>> ilist)
    : map_(ilist) {}

// template <typename T>
// T AnyMap::insert_or_assign(const std::string &key, T &&value) {
//   std::scoped_lock lk{mtx_};
//   auto [it, inserted] = this->map_.insert_or_assign(
//       key, std::make_any(std::forward<T>(value))); // 完美转发
//   return std::any_cast<T>(it->second);
// }
bool AnyMap::contains(const std::string &key) {
  std::shared_lock lk{mtx_};
  // auto it = this->map_.find(key);
  // return it == this->map_.end() ? false : true;
  return this->map_.contains(key);
}
// template <typename T>
// std::pair<bool, T> AnyMap::contains(const std::string &key) {
//   std::scoped_lock lk{mtx_};
//   auto it = this->map_.find(key);
//   return std::make_pair<bool, T>(it == this->map_.end() ? false : true,
//                                  std::any_cast<T>(it->second));
// }
// template <typename T> std::optional<T> AnyMap::find(const std::string &key) {
//   std::scoped_lock lk{mtx_};
//   auto [has, value] = this->contains<T>(key);
//   return has ? std::make_optional<T>(value) : std::nullopt;
// }

void AnyMap::clear() {
  std::unique_lock lk{mtx_};
  this->map_.clear();
  return;
}

} // namespace tools
