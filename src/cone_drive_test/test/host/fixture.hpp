// 테스트 전용: 시뮬레이터 골든 데이터 읽기 (nlohmann/json, 호스트 테스트에서만 사용)
#pragma once
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include "cone_drive_test/core/types.hpp"

inline nlohmann::json load_fixture(const std::string & name) {
  std::ifstream in(std::string(CDT_FIXTURES) + "/" + name);
  return nlohmann::json::parse(in);
}
inline cone_drive_test::Vec2 v2(const nlohmann::json & a) { return {a[0].get<double>(), a[1].get<double>()}; }
inline std::vector<cone_drive_test::Vec2> v2list(const nlohmann::json & a) {
  std::vector<cone_drive_test::Vec2> out;
  for (const auto & p : a) out.push_back(v2(p));
  return out;
}
