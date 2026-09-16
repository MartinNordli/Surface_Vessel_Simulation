// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>
namespace njord::hydro {
struct Vec {
  double x{}, y{}, z{};
  Vec operator+(Vec b) const { return {x + b.x, y + b.y, z + b.z}; }
  Vec operator-(Vec b) const { return {x - b.x, y - b.y, z - b.z}; }
  Vec operator*(double s) const { return {x * s, y * s, z * s}; }
  double dot(Vec b) const { return x * b.x + y * b.y + z * b.z; }
  Vec cross(Vec b) const {
    return {y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x};
  }
};
using Face = std::vector<Vec>;
struct Mesh {
  std::vector<Vec> vertices;
  std::vector<std::array<unsigned, 3>> faces;
};
struct Displacement {
  double volume{};
  Vec centroid{};
};
// Deliberately restricted to one closed, outward-oriented convex component.
// Nonconvex / multiple / overlapping CAD volumes need a separate robust
// importer.
inline void Validate(const Mesh &m) {
  if (m.vertices.size() < 4 || m.faces.size() < 4)
    throw std::invalid_argument("empty buoyancy mesh");
  std::map<std::pair<unsigned, unsigned>, int> edges;
  for (auto v : m.vertices)
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
      throw std::invalid_argument("nonfinite mesh");
  for (auto f : m.faces) {
    for (auto i : f)
      if (i >= m.vertices.size())
        throw std::invalid_argument("mesh index");
    auto a = m.vertices[f[0]],
         n = (m.vertices[f[1]] - a).cross(m.vertices[f[2]] - a);
    if (n.dot(n) < 1e-20)
      throw std::invalid_argument("degenerate triangle");
    for (auto v : m.vertices)
      if (n.dot(v - a) > 1e-9 * std::sqrt(n.dot(n)))
        throw std::invalid_argument(
            "nonconvex, intersecting or reversed mesh unsupported");
    for (int j = 0; j < 3; j++)
      if (++edges[{f[j], f[(j + 1) % 3]}] != 1)
        throw std::invalid_argument("overlapping mesh face");
  }
  for (auto e : edges)
    if (edges.find({e.first.second, e.first.first}) == edges.end())
      throw std::invalid_argument("open buoyancy mesh");
}
inline Mesh Box(Vec size) {
  if (size.x <= 0 || size.y <= 0 || size.z <= 0)
    throw std::invalid_argument("invalid box size");
  Mesh m;
  for (int z : {-1, 1})
    for (int y : {-1, 1})
      for (int x : {-1, 1})
        m.vertices.push_back({x * size.x / 2, y * size.y / 2, z * size.z / 2});
  m.faces = {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
             {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
  Validate(m);
  return m;
}
inline Displacement Submerged(const Mesh &worldMesh, double level) {
  std::vector<Face> polygons;
  std::vector<Vec> cap;
  for (auto f : worldMesh.faces) {
    Face input{worldMesh.vertices[f[0]], worldMesh.vertices[f[1]],
               worldMesh.vertices[f[2]]},
        output;
    for (unsigned i = 0; i < input.size(); i++) {
      auto a = input[i], b = input[(i + 1) % input.size()];
      bool ai = a.z <= level, bi = b.z <= level;
      if (ai)
        output.push_back(a);
      if (ai != bi) {
        auto p = a + (b - a) * ((level - a.z) / (b.z - a.z));
        output.push_back(p);
        cap.push_back(p);
      }
    }
    if (output.size() >= 3)
      polygons.push_back(output);
  }
  Face unique;
  for (auto p : cap) {
    bool exists = false;
    for (auto q : unique)
      if ((p - q).dot(p - q) < 1e-20)
        exists = true;
    if (!exists)
      unique.push_back(p);
  }
  if (unique.size() >= 3) {
    Vec c;
    for (auto p : unique)
      c = c + p;
    c = c * (1.0 / unique.size());
    std::sort(unique.begin(), unique.end(), [c](Vec a, Vec b) {
      return std::atan2(a.y - c.y, a.x - c.x) <
             std::atan2(b.y - c.y, b.x - c.x);
    });
    polygons.push_back(unique); // water surface has outward normal +Z
  }
  if (polygons.empty())
    return {};
  Vec origin = polygons[0][0], moment;
  double volume = 0;
  for (auto p : polygons)
    for (unsigned i = 1; i + 1 < p.size(); i++) {
      auto a = p[0] - origin, b = p[i] - origin, c = p[i + 1] - origin;
      double v = a.dot(b.cross(c)) / 6;
      volume += v;
      moment = moment + (a + b + c) * (v / 4);
    }
  if (volume <= 1e-14)
    return {};
  return {volume, origin + moment * (1 / volume)};
}
inline double Response(double previous, double target, double dt, double tau) {
  return tau == 0 ? target
                  : previous + (target - previous) * (-std::expm1(-dt / tau));
}
} // namespace njord::hydro
