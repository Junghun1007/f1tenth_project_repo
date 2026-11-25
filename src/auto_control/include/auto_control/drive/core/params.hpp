// Copied from src/cone_drive_test/include/cone_drive_test/core/params.hpp (1113ML) — auto_control로 옮김
// 실시간 파라미터 레지스트리: 웹 파라미터 창의 목록·일괄 적용(원자적)·YAML 저장
#pragma once
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace auto_control::drive {

// Write completely before publishing the filename. Preset saves never replace an existing file.
void write_parameter_yaml(const std::filesystem::path & path, const std::string & yaml, bool replace);

class ParameterPresets {
public:
  explicit ParameterPresets(std::filesystem::path directory) : directory_(std::move(directory)) {}
  std::vector<std::string> list() const;
  std::string save(const std::string & timestamp, const std::string & label, const std::string & yaml);
  std::filesystem::path path(const std::string & name) const;

private:
  std::filesystem::path directory_;
  std::mutex mutex_;
};

struct ParamSpec {
  std::string name;     // ROS 파라미터 이름 (예: drive.maximum_speed_mps)
  std::string group;    // "obstacle" | "driving"
  std::string section;  // 창 안의 소제목
  std::string label;
  std::string unit;
  double min{0}, max{1}, value{0};
  bool live{true};      // false = 재시작해야 반영
  std::vector<std::pair<double, std::string>> options{};
};

class ParamRegistry {
public:
  using ValueLookup = std::function<double(const std::string &)>;
  using Validator = std::function<std::string(const ValueLookup &)>;
  void add(const ParamSpec & spec);
  void add_order_constraint(const std::string & lower, const std::string & upper);
  void add_validator(Validator validate);
  std::vector<ParamSpec> list() const;
  double get(const std::string & name) const;   // 없으면 std::out_of_range
  bool has(const std::string & name) const;
  // 하나라도 잘못되면 아무것도 바꾸지 않는다. 실패 메시지에 항목 이름 포함
  std::pair<bool, std::string> apply(const std::map<std::string, double> & changes);
  std::string to_yaml(const std::string & node_name) const;
  std::string to_json() const;
  std::uint64_t version() const;

private:
  mutable std::mutex mutex_;
  std::vector<ParamSpec> specs_;
  std::map<std::string, std::size_t> index_;
  std::vector<std::pair<std::string, std::string>> order_;
  std::vector<Validator> validators_;
  std::uint64_t version_{0};
};

}  // namespace auto_control::drive
