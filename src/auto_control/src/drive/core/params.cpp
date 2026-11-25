// Copied from src/cone_drive_test/src/core/params.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/params.hpp"
#include "auto_control/drive/core/json_writer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace auto_control::drive {
namespace {
namespace fs = std::filesystem;

bool preset_timestamp(const std::string & text) {
  return text.size() == 12 && std::all_of(text.begin(), text.end(), [](char ch) {
    return ch >= '0' && ch <= '9';
  });
}

bool preset_label(const std::string & text) {
  return !text.empty() && text.size() <= 104 && std::all_of(text.begin(), text.end(), [](unsigned char ch) {
    return ch >= 0x80 || (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'z') ||
      (ch >= 'A' && ch <= 'Z') || ch == '_' || ch == '-' || ch == ' ';
  });
}

bool preset_filename(const std::string & name) {
  return name.size() > 18 && name.compare(name.size() - 5, 5, ".yaml") == 0 &&
    name[12] == '_' && preset_timestamp(name.substr(0, 12)) && preset_label(name.substr(13, name.size() - 18));
}
}  // namespace

void write_parameter_yaml(const fs::path & path, const std::string & yaml, bool replace) {
  auto tmp = (path.parent_path() / ".params-XXXXXX").string();
  const int fd = ::mkstemp(tmp.data());
  if (fd < 0) throw std::system_error(errno, std::generic_category(), "YAML 임시 파일 생성");
  ::close(fd);
  try {
    std::ofstream out(tmp, std::ios::binary);
    out << yaml;
    out.close();
    if (!out) throw std::runtime_error("YAML 파일을 쓸 수 없습니다");
    if (replace) fs::rename(tmp, path);
    else fs::create_hard_link(tmp, path);
  } catch (...) {
    std::error_code ignored;
    fs::remove(tmp, ignored);
    throw;
  }
  std::error_code ignored;
  fs::remove(tmp, ignored);
}

std::vector<std::string> ParameterPresets::list() const {
  std::vector<std::string> names;
  if (!fs::is_directory(directory_)) return names;
  for (const auto & entry : fs::directory_iterator(directory_)) {
    const auto name = entry.path().filename().string();
    if (preset_filename(name) && fs::is_regular_file(entry.symlink_status())) names.push_back(name);
  }
  std::sort(names.rbegin(), names.rend());
  return names;
}

std::string ParameterPresets::save(const std::string & timestamp, const std::string & label, const std::string & yaml) {
  if (!preset_timestamp(timestamp) || label.size() > 96 || !preset_label(label) ||
    label.find_first_not_of(' ') == std::string::npos)
    throw std::invalid_argument("이름에 글자, 숫자, 공백, 하이픈, 밑줄을 사용할 수 있습니다");
  std::lock_guard<std::mutex> lock(mutex_);
  fs::create_directories(directory_);
  const auto stem = timestamp + "_" + label;
  for (int suffix = 1; ; ++suffix) {
    const auto name = (suffix == 1 ? stem : stem + "_" + std::to_string(suffix)) + ".yaml";
    try {
      write_parameter_yaml(directory_ / name, yaml, false);
      return name;
    } catch (const fs::filesystem_error & e) {
      if (e.code() != std::errc::file_exists) throw;
    }
  }
}

fs::path ParameterPresets::path(const std::string & name) const {
  if (!preset_filename(name)) throw std::invalid_argument("프리셋 파일 이름이 올바르지 않습니다");
  const auto file = directory_ / name;
  if (!fs::is_regular_file(fs::symlink_status(file))) throw std::invalid_argument("프리셋 파일이 없습니다");
  return file;
}

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
