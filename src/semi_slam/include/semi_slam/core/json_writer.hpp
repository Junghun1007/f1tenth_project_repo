// 작은 JSON 작성기 (외부 라이브러리 없이 텔레메트리·API 응답 생성)
#pragma once
#include <cmath>
#include <cstdio>
#include <string>

namespace semi_slam {

class JsonWriter {
public:
  JsonWriter & begin_object() { sep(); s_ += '{'; first_ = true; return *this; }
  JsonWriter & end_object() { s_ += '}'; first_ = false; return *this; }
  JsonWriter & begin_array() { sep(); s_ += '['; first_ = true; return *this; }
  JsonWriter & end_array() { s_ += ']'; first_ = false; return *this; }
  JsonWriter & key(const char * k) { sep(); str(k); s_ += ':'; first_ = true; return *this; }
  JsonWriter & value(const std::string & v) { sep(); str(v); return *this; }
  JsonWriter & value(const char * v) { return value(std::string(v)); }
  JsonWriter & value(bool v) { sep(); s_ += v ? "true" : "false"; return *this; }
  JsonWriter & value(double v, int digits = 4) {
    sep();
    if (!std::isfinite(v)) { s_ += "null"; return *this; }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
    s_ += buf;
    return *this;
  }
  JsonWriter & value(long long v) { sep(); s_ += std::to_string(v); return *this; }
  JsonWriter & null() { sep(); s_ += "null"; return *this; }
  const std::string & str() const { return s_; }

private:
  void sep() { if (!first_) s_ += ','; first_ = false; }
  void str(const std::string & v) {
    s_ += '"';
    for (unsigned char c : v) {
      switch (c) {
        case '"': s_ += "\\\""; break;
        case '\\': s_ += "\\\\"; break;
        case '\n': s_ += "\\n"; break;
        case '\r': s_ += "\\r"; break;
        case '\t': s_ += "\\t"; break;
        default:
          if (c < 0x20) { char b[8]; std::snprintf(b, sizeof(b), "\\u%04x", c); s_ += b; }
          else s_ += char(c);
      }
    }
    s_ += '"';
  }
  std::string s_;
  bool first_{true};
};

}  // namespace semi_slam
