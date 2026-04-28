#pragma once
// 单例模板

template <typename T> class Single {
protected:
  Single() = default;
  ~Single() = default;
public:
  static T* instance() {
    if (t == nullptr)
      t = new T();
    return t;
  }

  Single(const Single&) = delete;
  Single& operator=(const Single&) = delete;
  Single(Single&&) = delete;
  Single& operator=(Single&&) = delete;
private:
  static T* t;
};

template <typename T>
T* Single<T>::t = nullptr;