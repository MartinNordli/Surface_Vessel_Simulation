// Analytic checks of njord/Hydrostatics.hh (built with BUILD_TESTING and run
// by ctest as njord_hydrostatics). Each expected value is derived by hand.
#include "njord/Hydrostatics.hh"
// Keep assert() active even in release builds.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
using namespace njord::hydro;
int main() {
  // 4 x 2 x 2 m box centred on z = 0: half submerged at level 0 displaces
  // 8 m^3 with its centroid halfway down the wet half (z = -0.5).
  auto box = Box({4, 2, 2});
  auto half = Submerged(box, 0);
  assert(std::abs(half.volume - 8) < 1e-10);
  assert(std::abs(half.centroid.z + .5) < 1e-10);
  // Entirely dry below the hull, entirely submerged above it.
  assert(Submerged(box, -2).volume == 0);
  assert(std::abs(Submerged(box, 2).volume - 16) < 1e-10);
  // Roll the box 45 degrees about x. The wet half is then a triangular
  // prism: 8 m^3 by symmetry, centroid at one third of its sqrt(2) m depth.
  for (auto &p : box.vertices) {
    double y = p.y, z = p.z;
    p.y = (y - z) / std::sqrt(2.);
    p.z = (y + z) / std::sqrt(2.);
  }
  auto tilted = Submerged(box, 0);
  assert(std::abs(tilted.volume - 8) < 1e-10);
  assert(std::abs(tilted.centroid.z + std::sqrt(2.) / 3) < 1e-10);
  // Validate must reject an open mesh (missing face) ...
  auto invalid = box;
  invalid.faces.pop_back();
  bool caught = false;
  try {
    Validate(invalid);
  } catch (std::invalid_argument &) {
    caught = true;
  }
  assert(caught);
  // ... and a duplicated face.
  invalid = box;
  invalid.faces.push_back(invalid.faces[0]);
  caught = false;
  try {
    Validate(invalid);
  } catch (std::invalid_argument &) {
    caught = true;
  }
  assert(caught);
  // Right triangular prism: x in [0,2], y>=0,z>=0,y+z<=2.
  Mesh prism{{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}, {0, 0, 2}, {2, 0, 2}},
             {{0, 4, 2},
              {1, 3, 5},
              {0, 2, 3},
              {0, 3, 1},
              {0, 1, 5},
              {0, 5, 4},
              {2, 4, 5},
              {2, 5, 3}}};
  // Cut at z = 1: cross-section area 2 - 0.5 = 1.5 m^2 times length 2 m
  // gives 3 m^3; the centroid follows from the trapezoid section.
  Validate(prism);
  auto slice = Submerged(prism, 1);
  assert(std::abs(slice.volume - 3) < 1e-10);
  assert(std::abs(slice.centroid.x - 1) < 1e-10);
  assert(std::abs(slice.centroid.z - 4. / 9) < 1e-10);
  assert(std::abs(slice.centroid.y - 7. / 9) < 1e-10);
  // Same slice far from the origin: results must not lose precision.
  for (auto &p : prism.vertices)
    p = p + Vec{10000, -10000, 5};
  auto moved = Submerged(prism, 6);
  assert(std::abs(moved.volume - 3) < 1e-10);
  assert(std::abs(moved.centroid.z - (5 + 4. / 9)) < 1e-10);
  // One inward-wound (flipped) face must be rejected.
  invalid = box;
  std::swap(invalid.faces[0][0], invalid.faces[0][1]);
  caught = false;
  try {
    Validate(invalid);
  } catch (std::invalid_argument &) {
    caught = true;
  }
  assert(caught);
  // First-order response after one time constant: 1 - e^-1 of the step.
  assert(std::abs(Response(0, 100, 1, 1) - 100 * (1 - std::exp(-1))) < 1e-10);
  std::cout << "hydrostatics analytic checks passed\n";
}
