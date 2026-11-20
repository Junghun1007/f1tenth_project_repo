// Copied from src/cone_drive_test/src/core/mini_json.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/mini_json.hpp"
#include <cctype>
#include <cstdlib>

namespace auto_control::drive {

namespace {
struct Parser {
  const std::string & s;
  std::size_t i{0};
  int depth{0};
  void ws() { while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i; }
  bool lit(const char * w) { const std::string t(w); if (s.compare(i, t.size(), t) == 0) { i += t.size(); return true; } return false; }
  bool string(std::string & out) {
    if (i >= s.size() || s[i] != '"') return false;
    ++i;
    while (i < s.size() && s[i] != '"') {
      if (s[i] == '\\' && i + 1 < s.size()) {
        const char e = s[++i];
        out += e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e;
        if (e == 'u') { out.pop_back(); i += 4; }   // \uXXXX는 버림 (키·값에 필요 없음)
        ++i;
      } else {
        out += s[i++];
      }
    }
    if (i >= s.size()) return false;
    ++i;
    return true;
  }
  bool value(JsonValue & v) {
    if (++depth > 32) return false;
    ws();
    if (i >= s.size()) return false;
    bool ok = true;
    if (s[i] == '{') {
      v.kind = JsonValue::Object;
      ++i; ws();
      if (i < s.size() && s[i] == '}') { ++i; }
      else {
        for (;;) {
          ws();
          std::string k;
          if (!string(k)) { ok = false; break; }
          ws();
          if (i >= s.size() || s[i] != ':') { ok = false; break; }
          ++i;
          auto child = std::make_shared<JsonValue>();
          if (!value(*child)) { ok = false; break; }
          v.obj[k] = child;
          ws();
          if (i < s.size() && s[i] == ',') { ++i; continue; }
          if (i < s.size() && s[i] == '}') { ++i; break; }
          ok = false; break;
        }
      }
    } else if (s[i] == '[') {
      v.kind = JsonValue::Array;
      ++i; ws();
      if (i < s.size() && s[i] == ']') { ++i; }
      else {
        for (;;) {
          JsonValue skip;
          if (!value(skip)) { ok = false; break; }
          ws();
          if (i < s.size() && s[i] == ',') { ++i; continue; }
          if (i < s.size() && s[i] == ']') { ++i; break; }
          ok = false; break;
        }
      }
    } else if (s[i] == '"') {
      v.kind = JsonValue::String;
      ok = string(v.str);
    } else if (lit("true")) { v.kind = JsonValue::Bool; v.b = true; }
    else if (lit("false")) { v.kind = JsonValue::Bool; v.b = false; }
    else if (lit("null")) { v.kind = JsonValue::Null; }
    else {
      char * end = nullptr;
      v.num = std::strtod(s.c_str() + i, &end);
      if (end == s.c_str() + i) ok = false;
      else { v.kind = JsonValue::Number; i = std::size_t(end - s.c_str()); }
    }
    --depth;
    return ok;
  }
};
}  // namespace

std::optional<JsonValue> parse_json(const std::string & text) {
  Parser p{text};
  JsonValue v;
  if (!p.value(v)) return std::nullopt;
  p.ws();
  if (p.i != text.size()) return std::nullopt;
  return v;
}

}  // namespace auto_control::drive
