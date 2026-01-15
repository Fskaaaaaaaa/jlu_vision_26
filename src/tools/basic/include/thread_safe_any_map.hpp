#pragma once

#include <any>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace tools {

class AnyMap {
public:
  AnyMap() = default;
  AnyMap(std::initializer_list<std::pair<const std::string, std::any>> ilist);

  // template <typename T> T insert_or_assign(const std::string &key, T
  // &&value);
  template <typename T>
  std::pair<T, bool> insert(const std::string &key, T &&value);
  bool contains(const std::string &key);
  template <typename T> std::optional<T> find(const std::string &key);

  void clear();

private:
  std::shared_mutex mtx_;
  std::unordered_map<std::string, std::any> map_;
};

template <typename T>
std::pair<T, bool> AnyMap::insert(const std::string &key, T &&value) {
  std::unique_lock lk{mtx_};
  // auto [it, inserted] = this->map_.insert_or_assign(
  //     key, std::make_any<T>(std::forward<T>(value))); // 完美转发
  // return std::any_cast<T>(it->second);
  auto [it, inserted] =
      this->map_.insert({key, std::make_any<T>(std::forward<T>(value))});
  auto res = std::any_cast<T>(it->second);
  return {res, inserted};
}

// template <typename T>
// std::pair<bool, T> AnyMap::contains(const std::string &key) {
//   std::shared_lock lk{mtx_};
//   auto it = this->map_.find(key);
//   return std::make_pair<bool, T>(it == this->map_.end() ? false : true,
//                                  std::any_cast<T>(it->second));
//                                  //这里有严重问题！！在没有找到的情况下，直接对end进行anycast只会段错误！！
// }
template <typename T> std::optional<T> AnyMap::find(const std::string &key) {
  std::shared_lock lk{mtx_};
  // return has ? std::make_optional<T>(value) : std::nullopt;
  return map_.contains(key)
             ? std::make_optional<T>(std::any_cast<T>(map_.find(key)->second))
             : std::nullopt;
}

} // namespace tools
