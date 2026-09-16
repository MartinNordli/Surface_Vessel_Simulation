#include "njord/Hydrostatics.hh"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
using namespace njord::hydro;
int main() {
  auto box = Box({4, 2, 2});
  auto half = Submerged(box, 0);
  assert(std::abs(half.volume - 8) < 1e-10);
  assert(std::abs(half.centroid.z + .5) < 1e-10);
  assert(Submerged(box, -2).volume == 0);
  assert(std::abs(Submerged(box, 2).volume - 16) < 1e-10);
  for (auto &p : box.vertices) {
    double y = p.y, z = p.z;
    p.y = (y - z) / std::sqrt(2.);
    p.z = (y + z) / std::sqrt(2.);
  }
  auto tilted = Submerged(box, 0);
  assert(std::abs(tilted.volume - 8) < 1e-10);
  assert(std::abs(tilted.centroid.z + std::sqrt(2.) / 3) < 1e-10);
  auto invalid = box;
  invalid.faces.pop_back();
  bool caught = false;
  try {
    Validate(invalid);
  } catch (std::invalid_argument &) {
    caught = true;
  }
  assert(caught);
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
  Validate(prism);
  auto slice = Submerged(prism, 1);
  assert(std::abs(slice.volume - 3) < 1e-10);
  assert(std::abs(slice.centroid.x - 1) < 1e-10);
  assert(std::abs(slice.centroid.z - 4. / 9) < 1e-10);
  assert(std::abs(slice.centroid.y - 7. / 9) < 1e-10);
  for (auto &p : prism.vertices)
    p = p + Vec{10000, -10000, 5};
  auto moved = Submerged(prism, 6);
  assert(std::abs(moved.volume - 3) < 1e-10);
  assert(std::abs(moved.centroid.z - (5 + 4. / 9)) < 1e-10);
  invalid = box;
  std::swap(invalid.faces[0][0], invalid.faces[0][1]);
  caught = false;
  try {
    Validate(invalid);
  } catch (std::invalid_argument &) {
    caught = true;
  }
  assert(caught);
  assert(std::abs(Response(0, 100, 1, 1) - 100 * (1 - std::exp(-1))) < 1e-10);
  std::cout << "hydrostatics analytic checks passed\n";
}
