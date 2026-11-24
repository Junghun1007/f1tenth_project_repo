// 실시간 파라미터 레지스트리: 웹 파라미터 창의 목록·일괄 적용(원자적)·YAML 저장
#pragma once
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cone_drive_test {

struct ParamSpec {
  std::string name;     // ROS 파라미터 이름 (예: drive.maximum_speed_mps)
  std::string group;    // "obstacle" | "driving" | "camera"
  std::string section;  // 창 안의 소제목
  std::string label;
  std::string unit;
  double min{0}, max{1}, value{0};
  bool live{true};      // false = 재시작해야 반영
  bool integer{false};
  std::map<int, std::string> choices{};
};

class ParamRegistry {
public:
  void add(const ParamSpec & spec);
  void add_order_constraint(const std::string & lower, const std::string & upper);
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
  std::uint64_t version_{0};
};

}  // namespace cone_drive_test
