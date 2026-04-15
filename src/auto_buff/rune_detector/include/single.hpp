#pragma once
//单例模板
#include <memory>

template <typename T> class Single {
public:
  Single() = default;
  ~Single() = default;

  static std::unique_ptr<T> instance() {
    if (t == nullptr)
      t = std::make_unique<t>();
    return t;
  }
private:
  static std::unique_ptr<T> t;
};

template <typename T>
std::unique_ptr<T> Single<T>::t = nullptr;