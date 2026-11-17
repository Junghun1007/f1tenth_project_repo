#include "check.hpp"
int main(int argc, char ** argv) {
  int failed = 0, run = 0;
  for (auto & c : cdt_test::cases()) {
    if (argc > 1 && std::string(argv[1]) != c.name) continue;
    ++run;
    try { c.fn(); std::cout << "[ PASS ] " << c.name << "\n"; }
    catch (const cdt_test::Failure & f) { ++failed; std::cout << "[ FAIL ] " << c.name << ": " << f.msg << "\n"; }
    catch (const std::exception & e) { ++failed; std::cout << "[ FAIL ] " << c.name << ": exception " << e.what() << "\n"; }
  }
  std::cout << run - failed << "/" << run << " passed\n";
  return failed ? 1 : 0;
}
