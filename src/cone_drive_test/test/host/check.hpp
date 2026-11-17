// 아주 작은 테스트 도구: TEST(name){...} 등록, CHECK / CHECK_NEAR, main은 check_main.cpp
#pragma once
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace cdt_test {
struct Case { const char * name; std::function<void()> fn; };
inline std::vector<Case> & cases() { static std::vector<Case> c; return c; }
struct Reg { Reg(const char * n, std::function<void()> f) { cases().push_back({n, std::move(f)}); } };
struct Failure { std::string msg; };
}  // namespace cdt_test

#define TEST(name) static void name(); static cdt_test::Reg reg_##name(#name, name); static void name()
#define CHECK(cond) do { if (!(cond)) { std::ostringstream o_; o_ << __FILE__ << ":" << __LINE__ << " CHECK(" #cond ")"; throw cdt_test::Failure{o_.str()}; } } while (0)
#define CHECK_NEAR(a, b, tol) do { const double a_ = (a), b_ = (b); if (!(std::abs(a_ - b_) <= (tol))) { std::ostringstream o_; \
  o_ << __FILE__ << ":" << __LINE__ << " CHECK_NEAR(" #a ", " #b ") " << a_ << " vs " << b_ << " tol " << (tol); throw cdt_test::Failure{o_.str()}; } } while (0)
