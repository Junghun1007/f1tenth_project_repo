// 웹 요청 본문용 최소 JSON 파서 (객체·문자열·숫자·bool·null, 배열은 건너뜀)
#pragma once
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace semi_slam {

struct JsonValue {
  enum Kind { Null, Bool, Number, String, Object, Array } kind{Null};
  bool b{false};
  double num{0};
  std::string str;
  std::map<std::string, std::shared_ptr<JsonValue>> obj;
  const JsonValue * get(const std::string & k) const { auto it = obj.find(k); return it == obj.end() ? nullptr : it->second.get(); }
};

std::optional<JsonValue> parse_json(const std::string & text);   // 형식 오류면 nullopt

}  // namespace semi_slam
