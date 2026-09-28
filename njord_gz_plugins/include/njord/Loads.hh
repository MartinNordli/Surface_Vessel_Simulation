// SPDX-License-Identifier: MIT
//
// Header-only wind and thruster loads used by the NjordPhysics plugin and
// unit-tested without Gazebo by test/loads_test.cc:
//   WindCoefficients  periodic interpolation of the (cx, cy, cn) table
//   WindLoad          body-frame wind force and yaw moment
//   ThrusterWrench    force and torque of one thruster about the COM
// Units are SI (m, m/s, N, N m, deg for table angles); body frame is
// forward-left-up. Python mirrors: physics_core.wind_coefficients,
// physics_core.wind_load and physics_core.thruster_wrench; both sides are
// checked against test/load_vectors.csv.
#pragma once
#include "njord/Hydrostatics.hh"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace njord::loads {
using hydro::Vec;
// Air density used for the dynamic pressure (kg/m^3).
constexpr double kAirDensity = 1.225;
// One row of the wind table; angle (deg) is the direction the relative air
// flow moves toward in the body frame, counter-clockwise from forward.
struct WindRow {
  double angle, cx, cy, cn;
};
// Periodic linear interpolation between neighbouring rows of a table sorted
// by angle; the table wraps from its last row to its first across 360 deg.
// ``angle`` may be any value; it is wrapped to [0, 360). Returns {cx, cy, cn}.
inline std::array<double, 3> WindCoefficients(double angle,
                                              const std::vector<WindRow> &table) {
  if (table.size() < 2)
    throw std::invalid_argument("wind table requires two angles");
  angle = std::fmod(std::fmod(angle, 360.) + 360., 360.);
  auto upper = std::upper_bound(
      table.begin(), table.end(), angle,
      [](double a, const WindRow &b) { return a < b.angle; });
  auto b = upper == table.end() ? table.front() : *upper;
  auto a = upper == table.begin() ? table.back() : *(upper - 1);
  double aa = a.angle, bb = b.angle;
  if (bb <= aa)
    bb += 360;
  if (angle < aa)
    angle += 360;
  double f = (angle - aa) / (bb - aa);
  return {a.cx + f * (b.cx - a.cx), a.cy + f * (b.cy - a.cy),
          a.cn + f * (b.cn - a.cn)};
}
// Body-frame wind load for the relative air velocity ``air`` (m/s, body
// frame; only x and y count): dynamic pressure q = 0.5 rho |air_xy|^2 and
// X = q Ax cx, Y = q Ay cy, yaw moment N = q Ay L cn. Returns {X, Y, N}.
inline std::array<double, 3> WindLoad(Vec air, double areaX, double areaY,
                                      double length,
                                      const std::vector<WindRow> &table) {
  double angle = std::atan2(air.y, air.x) * 180 / M_PI;
  auto c = WindCoefficients(angle, table);
  double pressure = .5 * kAirDensity * (air.x * air.x + air.y * air.y);
  return {pressure * areaX * c[0], pressure * areaY * c[1],
          pressure * areaY * length * c[2]};
}
// Force and torque about the centre of mass ``com`` of a thrust ``force``
// (N, signed) along the unit ``axis`` at ``position`` (body frame, m).
struct Wrench {
  Vec force, torque;
};
inline Wrench ThrusterWrench(double force, Vec position, Vec axis, Vec com) {
  Vec f = axis * force;
  return {f, (position - com).cross(f)};
}
} // namespace njord::loads
