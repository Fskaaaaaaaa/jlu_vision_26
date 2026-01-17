#include "fast_tf/fast_tf.hpp"

#include <Eigen/Core>
// IWYU pragma: no_include "Eigen/src/Core/GeneralProduct.h"
// IWYU pragma: no_include "Eigen/src/Geometry/Quaternion.h"
// IWYU pragma: no_include "Eigen/src/Geometry/Translation.h"
// IWYU pragma: no_include "src/Core/DenseBase.h"
// IWYU pragma: no_include "src/Core/MatrixBase.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>

// the __has_builtin is defined under gcc and under clang
// see https://gcc.gnu.org/onlinedocs/cpp/_005f_005fhas_005fbuiltin.html
// see https://clang.llvm.org/docs/LanguageExtensions.html
// in c++20 we can use this without intrinsics:
// see https://en.cppreference.com/w/cpp/language/attributes/likely
#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#if __has_builtin(__builtin_expect)
#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)
#else
// if we don't have the feature then there is no better prediction
#define likely(x) x
#define unlikely(x) x
#endif

namespace fast_tf {
namespace detail {

/// @brief Creates a joined transform from the splitted data.
static Eigen::Isometry3d join(const transform &_split) noexcept {
  return Eigen::Isometry3d{_split.translation * _split.rotation};
}

/// @brief Creates a split transform from the joined data.
static transform split(const Eigen::Isometry3d &_join) noexcept {
  return {Eigen::Translation3d{_join.translation()},
          Eigen::Quaterniond{_join.rotation()}};
}

static_transform::static_transform(const Eigen::Isometry3d &_data) noexcept
    : data_(_data) {}

static_transform::static_transform(const transform &_data) noexcept
    : data_(join(_data)) {}

void static_transform::set(const Eigen::Isometry3d &_data) noexcept {
  data_ = _data;
}

void static_transform::set(const transform &_data) noexcept {
  data_ = join(_data);
}

counter::scoped_signal::scoped_signal(const counter &_c)
    : counter_(++_c.counter_) {}

counter::scoped_signal::~scoped_signal() { --counter_; }

dynamic_transform::dynamic_transform() noexcept
    : dur_(std::chrono::duration_cast<duration_t>(
          std::chrono::milliseconds(500))) {}

dynamic_transform::dynamic_transform(const duration_t &_dur) noexcept
    : dur_(_dur) {
  assert(dur_ >= duration_t(0) && "duration must be positive");
}

void dynamic_transform::set(const time_t &_time,
                            const transform &_data) noexcept {
  // mostlikely the data will come in in order and emplace_hint will not change
  // the entry if its already present.
  map_.emplace_hint(map_.end(), _time, _data);

  // check that the map is not empty - we need to acces the last valid element
  assert(!map_.empty() && "map cannot be empty");
  // rebalance
  auto lb = map_.lower_bound(std::prev(map_.end())->first - dur_);
  map_.erase(map_.begin(), lb);
}

void dynamic_transform::set(const time_t &_time,
                            const Eigen::Isometry3d &_data) noexcept {
  set(_time, split(_data));
}

/// @brief returns the vector interpolation between _l and _r.
/// @param _l left value.
/// @param _r right value.
/// @param _t the scale, must be in the interval [0, 1].
static Eigen::Vector3d lerp(const Eigen::Vector3d &_l,
                            const Eigen::Vector3d &_r, double _t) noexcept {
  return _l * _t + (1 - _t) * _r;
}

/// @brief returns the transform interpolation between _l and _r.
/// @param _l left value.
/// @param _r right value.
/// @param _t the scale, must be in the interval [0, 1].
static transform lerp(const transform &_l, const transform &_r,
                      double _t) noexcept {
  assert(_t >= 0 && "ratio must be bigger than 0");
  assert(_t <= 1 && "ratio must be smaller than 1");

  return {Eigen::Translation3d{
              lerp(_l.translation.vector(), _r.translation.vector(), _t)},
          _l.rotation.slerp(_t, _r.rotation)};
}

Eigen::Isometry3d dynamic_transform::get(const time_t &_time) const {
  if (_time.time_since_epoch().count() == 0) {
    if (map_.empty())
      throw std::runtime_error("no data");
    return join(std::prev(map_.end())->second);
  }

  // get the first element not smaller then _time
  // todo check if assume last helps us..
  const auto lb = map_.lower_bound(_time);

  // we will end up in this case if the map_ is either empty or if the stored
  // data is too old
  if (lb == map_.end())
    throw std::runtime_error("no matching data");

  // if the time stamps match exactly, we don't need to interpolate
  if (lb->first == _time)
    return join(lb->second);
  else {
    // check if we have two points to interpolate
    if (lb == map_.begin())
      throw std::runtime_error("no matching data: extrapolation in the past");
    // get the prev iterator
    const auto lower = std::prev(lb);
    // our count() returns a signed integer. the default duration is
    // nanoseconds. the nanoseconds must cover +/- 292 years. see
    // https://en.cppreference.com/w/cpp/chrono/duration
    const auto ratio = static_cast<double>((_time - lower->first).count()) /
                       (lb->first - lower->first).count();

    return join(lerp(lower->second, lb->second, ratio));
  }
}

transform_tree::node::node(variable_transform &&_data) noexcept
    : data(std::move(_data)) {}

transform_tree::node::node(variable_transform &&_data,
                           const node &_parent) noexcept
    : depth(_parent.depth + 1), parent(&_parent), data(std::move(_data)) {}

/// @brief factory returning either base_sequence or dynamic_transform
/// instances.
static variable_transform sequence_factory(bool _static) noexcept {
  if (_static)
    return static_transform{};
  else
    return dynamic_transform{};
}

transform_tree::pair_type
transform_tree::emplace_parent(const std::string &_parent_frame,
                               const std::string &_child_frame,
                               bool _is_static) {
  // check if the parent exists - this is the basic default case and we might
  // mark it as likely.
  if (auto parent = data_.find(_parent_frame); parent != data_.end())
    return std::make_pair(parent, false);

  // if the parent does not exist, we may try to create a new parent node.
  // this only works if the tree is either empty or if the child link is the
  // current root. in all other cases we will throw.
  if (_parent_frame.empty())
    throw std::runtime_error("a parent-frame cannot be empty");

  if (!empty()) {
    // check if the child-frame-id is the current root. if not - we must reject
    // the init-call since we already have a valid root-node.
    assert(root_ != data_.end() && "root cannot be data_.end()");
    if (root_->first != _child_frame)
      throw std::runtime_error("cannot reinit the map");

    // increment the map's depth counters
    for (auto &n : data_)
      ++n.second.depth;
  }
  // create the root node and wire up the connections. note: the root node is
  // always static.
  const auto res = data_.emplace(_parent_frame, node{static_transform{}});

  assert(res.second && "failed to create the root node");

  // in the case the child-frame-id is the current root
  if (root_ != data_.end()) {
    root_->second.parent = &res.first->second;
    // the root is always static - but if the root changes, the link might
    // become dynamic.
    if (!_is_static)
      root_->second.data = dynamic_transform{};
  }
  root_ = res.first;
  return res;
}

transform_tree::pair_type
transform_tree::emplace(const std::string &_parent_frame,
                        const std::string &_child_frame, bool _is_static) {
  // check if the input is just self-transform
  if (_parent_frame == _child_frame)
    throw std::runtime_error("self-transform detected");

  // get or create a new parent node. this call will throw if the tree would be
  // ill-formed.
  const auto parent = emplace_parent(_parent_frame, _child_frame, _is_static);

  // check if the child is known
  auto child = data_.find(_child_frame);
  if (unlikely(child == data_.end())) {
    // create a new node
    if (_child_frame.empty())
      throw std::runtime_error("a child-frame cannot be empty");

    auto res = data_.emplace(
        _child_frame, node{sequence_factory(_is_static), parent.first->second});
    assert(res.second && "failed to insert new child");
    return res;
  } else {
    // make sure that the newly added link is consistent. this will also
    // prevent us from creating cyclic graphs.
    if (child->second.parent != &parent.first->second)
      throw std::runtime_error("child cannot have multiple parents");

    return std::make_pair(child, parent.second);
  }
}

void transform_tree::merge(transform_tree &_other) {
  // calling this on an empty tree is a noop.
  if (_other.empty() || empty())
    return;

  // cn denotes the connecting link between the trees
  impl_t::iterator cn;
  if (cn = data_.find(_other.root_->first); cn != data_.end()) {
    // merge other into this
    for (auto &p : _other.data_)
      p.second.depth += cn->second.depth;

    // rewire the pointers to the root
    for (auto &p : _other.data_)
      if (p.second.parent == &_other.root_->second)
        p.second.parent = &cn->second;
  } else if (cn = _other.data_.find(root_->first); cn != _other.data_.end()) {
    // merge this into other
    for (auto &p : data_)
      p.second.depth += cn->second.depth;

    for (auto &p : _other.data_)
      if (p.second.parent == &cn->second)
        p.second.parent = &root_->second;
    root_->second = cn->second;
    root_ = _other.root_;
  } else
    throw std::runtime_error("cannot merge unconnected trees");

  // pass over the members from _other to this.
  data_.merge(_other.data_);

  // only root shall remain in the _other since this is the connecting link.
  assert(_other.data_.size() == 1 && "duplicated links detected");
  _other.data_.clear();
  _other.root_ = _other.data_.end();
}

transform_buffer::transform_buffer() : trees_(1) {}

transform_tree::pair_type
transform_buffer::emplace(const std::string &_parent_frame,
                          const std::string &_child_frame, bool _is_static) {
  // try to add the new transform to one of the existing trees. mostlikely we
  // will have only one tree.
  for (auto &tree : trees_) {
    try {
      return tree.emplace(_parent_frame, _child_frame, _is_static);
    } catch (std::runtime_error &_ex) {
      // completely normal phenomenon
      continue;
    }
  }

  // if we couldn't add the passed _tf, we will try to create a new tree
  transform_tree new_tree;
  // this might still throw - but this would be a logic error!
  const auto node = new_tree.emplace(_parent_frame, _child_frame, _is_static);
  trees_.emplace_back(std::move(new_tree));

  return node;
}

std::pair<transform_buffer::trees_t::const_iterator,
          transform_tree::const_iterator>
transform_buffer::find(const std::string &_frame) const noexcept {
  for (auto tree = trees_.begin(); tree != trees_.end(); ++tree) {
    if (auto res = tree->data_.find(_frame); res != tree->data_.end())
      return std::make_pair(tree, res);
  }
  return std::make_pair(trees_.end(), transform_tree::const_iterator{});
}

bool transform_buffer::merge(const std::string &_common_frame) {
  auto has_frame = [&_common_frame](const transform_tree &_tree) -> bool {
    return _tree.data_.count(_common_frame);
  };
  auto t1 = std::find_if(trees_.begin(), trees_.end(), has_frame);
  assert(t1 != trees_.end() && "the frame must known to at least one tree");
  auto t2 = std::find_if(std::next(t1), trees_.end(), has_frame);
  if (t2 == trees_.end())
    return false;

  // we may throw if the tree will become ill-formed
  t1->merge(*t2);
  trees_.erase(t2);
  return true;
}

void transform_buffer::merge(const std::string &_parent_frame,
                             const std::string &_child_frame) {
  // try to merge over the parent frame
  merge(_parent_frame) || merge(_child_frame);
}

void transform_buffer::set(const std::string &_parent_frame,
                           const std::string &_child_frame,
                           const time_t &_stamp_time, const transform &_tf,
                           bool _is_static) {
  std::unique_lock<std::mutex> l(m_);
  auto node = emplace(_parent_frame, _child_frame, _is_static);

  if (_is_static)
    std::get<0>(node.first->second.data).set(_tf);
  else {
    // dt is the dynamic transform
    auto &dt = std::get<1>(node.first->second.data);
    dt.set(_stamp_time, _tf);
    // check if there is someone waiting for this transform
    if (dt)
      cv_.notify_all();
  }

  // we didn't create a new node - so its fine!
  if (!node.second)
    return;

  // will throw if the tree will become ill-formed
  merge(_parent_frame, _child_frame);

  // notify if someone is waiting
  if (c_)
    cv_.notify_all();
}

void transform_buffer::set(const std::string &_parent_frame,
                           const std::string &_child_frame,
                           const time_t &_stamp_time,
                           const Eigen::Isometry3d &_tf, bool _is_static) {
  set(_parent_frame, _child_frame, _stamp_time, split(_tf), _is_static);
}

Eigen::Isometry3d transform_buffer::get_dynamic_transform(
    const time_t &_query_time, const time_t &_end_time,
    const dynamic_transform &_dyn_tr,
    std::unique_lock<std::mutex> &_lock) const {
  counter::scoped_signal signal{_dyn_tr};
  do {
    try {
      return _dyn_tr.get(_query_time);
    } catch (const std::runtime_error &_ex) {
      continue;
    }
  } while (cv_.wait_until(_lock, _end_time) != std::cv_status::timeout);
  throw std::runtime_error("no transform available");
}

static void advance(const transform_tree::node *&_node) {
  _node = _node->parent;
  assert(_node && "tree holds a nullptr");
}

Eigen::Isometry3d transform_buffer::get(const std::string &_target_frame,
                                        const std::string &_source_frame,
                                        const time_t &_query_time,
                                        const duration_t &_tolerance) const {
  if (_tolerance < duration_t{})
    throw std::runtime_error("tolerance cannot be negative");
  const auto end_time = clock_t::now() + _tolerance;

  std::unique_lock<std::mutex> l{m_};
  // NOTE:操，难怪tf查询这么慢
  // find the correct iterators
  auto target = find(_target_frame);
  auto source = find(_source_frame);

  while (!same_tree(target.first, source.first)) {
    detail::counter::scoped_signal ss{c_};
    if (cv_.wait_until(l, end_time) == std::cv_status::timeout)
      throw std::runtime_error("timeout: unconnected trees");

    // try again to connect the query
    target = find(_target_frame);
    source = find(_source_frame);
  }

  assert(same_tree(target.first, source.first) && "unconnected trees");

  const transform_tree::node *s = &source.second->second;
  const transform_tree::node *t = &target.second->second;

  // walk up the tree until we have reached the same depth for both
  Eigen::Isometry3d source_root(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target_root(Eigen::Isometry3d::Identity());

  // we need a lot of local variables for this call - so we make it a lambda.
  // note: the static_stansform returns by reference; the dynamic_transform
  // returns by value.
  auto get_transform = [&](const variable_transform &_var_tr,
                           const Eigen::Isometry3d &_right) {
    if (_var_tr.index() == 0)
      return std::get<0>(_var_tr).get() * _right;
    else
      return get_dynamic_transform(_query_time, end_time, std::get<1>(_var_tr),
                                   l) *
             _right;
  };

  // walk up with both until both branches meet
  while (t != s) {
    if (t->depth < s->depth) {
      source_root = get_transform(s->data, source_root);
      advance(s);
    } else if (t->depth > s->depth) {
      target_root = get_transform(t->data, target_root);
      advance(t);
    } else {
      source_root = get_transform(s->data, source_root);
      target_root = get_transform(t->data, target_root);

      advance(s);
      advance(t);
    }
  }

  return {target_root.inverse() * source_root};
}

} // namespace detail
} // namespace fast_tf
