// Copied from src/cone_drive_test/src/core/params.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/params.hpp"
#include "auto_control/drive/core/json_writer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace auto_control::drive {

void ParamRegistry::add(const ParamSpec & spec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index_.count(spec.name)) throw std::invalid_argument("duplicate parameter " + spec.name);
  index_[spec.name] = specs_.size();
  specs_.push_back(spec);
}

void ParamRegistry::add_order_constraint(const std::string & lower, const std::string & upper) {
  std::lock_guard<std::mutex> lock(mutex_);
  order_.emplace_back(lower, upper);
}

void ParamRegistry::add_validator(Validator validate) {
  std::lock_guard<std::mutex> lock(mutex_);
  validators_.push_back(std::move(validate));
}

std::vector<ParamSpec> ParamRegistry::list() const { std::lock_guard<std::mutex> lock(mutex_); return specs_; }

double ParamRegistry::get(const std::string & name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return specs_.at(index_.at(name)).value;
}

bool ParamRegistry::has(const std::string & name) const { std::lock_guard<std::mutex> lock(mutex_); return index_.count(name) > 0; }

std::uint64_t ParamRegistry::version() const { std::lock_guard<std::mutex> lock(mutex_); return version_; }

std::pair<bool, std::string> ParamRegistry::apply(const std::map<std::string, double> & changes) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = specs_;
  for (const auto & [name, v] : changes) {
    const auto it = index_.find(name);
    if (it == index_.end()) return {false, name + ": unknown parameter"};
    const auto & s = specs_[it->second];
    if (!std::isfinite(v)) return {false, name + ": not a number"};
    if (v < s.min || v > s.max) {
      char buf[96];
      std::snprintf(buf, sizeof(buf), ": %g outside [%g, %g]", v, s.min, s.max);
      return {false, name + buf};
    }
    if (!s.options.empty() && std::none_of(s.options.begin(), s.options.end(),
        [value = v](const auto & option) { return option.first == value; })) {
      return {false, name + ": unsupported option"};
    }
    next[it->second].value = v;
  }
  for (const auto & [lo, hi] : order_) {
    const auto a = index_.find(lo), b = index_.find(hi);
    if (a == index_.end() || b == index_.end()) continue;
    if (next[a->second].value >= next[b->second].value) return {false, lo + " must be below " + hi};
  }
  const ValueLookup value = [&](const std::string & name) { return next[index_.at(name)].value; };
  for (const auto & validate : validators_) {
    auto error = validate(value);
    if (!error.empty()) return {false, std::move(error)};
  }
  if (changes.empty()) return {true, ""};
  specs_.swap(next);
  ++version_;
  return {true, ""};
}

std::string ParamRegistry::to_yaml(const std::string & node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  // namespace(vehicle_namespace) 아래 노드에도 맞도록 /**/ 접두 (ROS 2 YAML 와일드카드)
  std::string out = "/**/" + node + ":\n  ros__parameters:\n";
  for (const auto & s : specs_) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.10g", s.value);
    std::string v = buf;
    if (v.find_first_of(".eEn") == std::string::npos) v += ".0";   // ROS가 double로 읽도록
    out += "    " + s.name + ": " + v + "\n";
  }
  return out;
}

std::string ParamRegistry::to_json() const {
  std::lock_guard<std::mutex> lock(mutex_);
  JsonWriter w;
  w.begin_object().key("version").value((long long)version_).key("params").begin_array();
  for (const auto & s : specs_) {
    w.begin_object().key("name").value(s.name).key("group").value(s.group).key("section").value(s.section)
     .key("label").value(s.label).key("unit").value(s.unit).key("min").value(s.min, 6).key("max").value(s.max, 6)
     .key("value").value(s.value, 6).key("live").value(s.live);
    if (!s.options.empty()) {
      w.key("options").begin_array();
      for (const auto & [value, label] : s.options) {
        w.begin_object().key("value").value(value, 6).key("label").value(label).end_object();
      }
      w.end_array();
    }
    w.end_object();
  }
  w.end_array().end_object();
  return w.str();
}

}  // namespace auto_control::drive
