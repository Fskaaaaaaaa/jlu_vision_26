#pragma once

#include <nameof.hpp>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace rm_ultra_tools {

template <typename T>
concept EnumType = std::is_enum_v<T>;

class YamlCfg {
public:
  explicit YamlCfg(const YAML::Node &root) : root_(root) {}

  // ---------- path builder ----------
  YamlCfg &operator[](const std::string &key) {
    path_.emplace_back(key);
    return *this;
  }
  YamlCfg &operator[](const char *key) { return (*this)[std::string(key)]; }

  template <EnumType Enum> YamlCfg &operator[](Enum e) {
    return (*this)[std::string(nameof::nameof_enum(e))];
  }
  // ---------- get ----------
  template <typename T> std::optional<T> get() {
    std::optional<T> ret;
    YAML::Node node = resolve();
    if (!node || !node.IsDefined()) {
      ret = std::nullopt;
    }
    try {
      ret = {node.as<T>()};
    } catch (...) {
      ret = std::nullopt;
    }
    reset();
    return ret;
  }

private:
  YAML::Node resolve() const {
    YAML::Node node = root_;
    for (const auto &key : path_) {
      if (!node || !node[key]) {
        return YAML::Node();
      }
      node = node[key];
    }
    return node;
  }
  void reset() { path_.clear(); }

private:
  const YAML::Node &root_;
  std::vector<std::string> path_;
};
} // namespace rm_ultra_tools
