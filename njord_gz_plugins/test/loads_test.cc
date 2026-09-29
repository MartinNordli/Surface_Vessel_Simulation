// Checks of njord/Loads.hh (built with BUILD_TESTING and run by ctest as
// njord_loads, and by tests/test_njord_physics.py). Analytic cases are derived
// by hand; the vectors in load_vectors.csv (argument 1) come from
// physics_core.py, so both implementations must agree.
#include "njord/Loads.hh"
// Keep assert() active even in release builds.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
using namespace njord;
using loads::WindRow;

static bool Near(double a, double b) {
  return std::abs(a - b) <= 1e-12 * std::max(1.0, std::abs(b));
}

static void Analytic() {
  // Symmetric four-row table: head wind pushes back, beam wind pushes aside.
  std::vector<WindRow> table{{0, 1, 0, 0}, {90, 0, 1, 0}, {180, -1, 0, 0},
                             {270, 0, -1, 0}};
  auto c = loads::WindCoefficients(45, table);
  assert(Near(c[0], .5) && Near(c[1], .5) && Near(c[2], 0));
  // Wrap-around between the last row (270) and the first (0 = 360).
  c = loads::WindCoefficients(315, table);
  assert(Near(c[0], .5) && Near(c[1], -.5));
  assert(Near(loads::WindCoefficients(-45, table)[1], -.5));
  assert(Near(loads::WindCoefficients(720, table)[0], 1));
  // 10 m/s forward air: q = 0.5 * 1.225 * 100 = 61.25 Pa, X = q * Ax * 1.
  auto load = loads::WindLoad({10, 0, 3}, 2, 4, 5, table);
  assert(Near(load[0], 122.5) && Near(load[1], 0) && Near(load[2], 0));
  // Still air: no load.
  load = loads::WindLoad({0, 0, 0}, 2, 4, 5, table);
  assert(load[0] == 0 && load[1] == 0 && load[2] == 0);
  // 100 N forward at 1 m to port and 2 m aft of the COM: yaw moment
  // r x F = (-2, 1, 0) x (100, 0, 0) = (0, 0, -100).
  auto w = loads::ThrusterWrench(100, {-2, 1, 0}, {1, 0, 0}, {0, 0, 0});
  assert(Near(w.force.x, 100) && Near(w.torque.z, -100));
  // The same thruster measured from a COM 1 m to port has no yaw arm.
  w = loads::ThrusterWrench(100, {-2, 1, 0}, {1, 0, 0}, {0, 1, 0});
  assert(Near(w.torque.z, 0));
  bool thrown = false;
  try {
    loads::WindCoefficients(0, {{0, 1, 0, 0}});
  } catch (const std::invalid_argument &) {
    thrown = true;
  }
  assert(thrown);
}

static int Vectors(const char *path) {
  std::ifstream file(path);
  assert(file && "cannot open load_vectors.csv");
  std::vector<WindRow> table;
  std::string line;
  int checked = 0;
  while (std::getline(file, line)) {
    std::stringstream row(line);
    std::string kind, cell;
    std::getline(row, kind, ',');
    std::vector<double> v;
    while (std::getline(row, cell, ','))
      v.push_back(std::stod(cell));
    if (kind == "table") {
      table.push_back({v[0], v[1], v[2], v[3]});
      continue;
    }
    if (kind == "coeff") {
      auto c = loads::WindCoefficients(v[0], table);
      for (int i = 0; i < 3; i++)
        assert(Near(c[i], v[1 + i]));
    } else if (kind == "wind") {
      auto load = loads::WindLoad({v[0], v[1], 0}, v[2], v[3], v[4], table);
      for (int i = 0; i < 3; i++)
        assert(Near(load[i], v[5 + i]));
    } else if (kind == "thrust") {
      auto w = loads::ThrusterWrench(v[0], {v[1], v[2], v[3]}, {v[4], v[5], v[6]},
                                     {v[7], v[8], v[9]});
      double got[] = {w.force.x, w.force.y, w.force.z,
                      w.torque.x, w.torque.y, w.torque.z};
      for (int i = 0; i < 6; i++)
        assert(Near(got[i], v[10 + i]));
    } else {
      assert(false && "unknown row kind");
    }
    checked++;
  }
  return checked;
}

int main(int argc, char **argv) {
  Analytic();
  assert(argc == 2 && "usage: loads_test <load_vectors.csv>");
  int checked = Vectors(argv[1]);
  assert(checked > 20);
  std::cout << "loads: analytic cases and " << checked << " vectors OK\n";
}
