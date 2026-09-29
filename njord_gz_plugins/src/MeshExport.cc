// Export the same loaded mesh geometry Gazebo consumes, without a renderer.
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv)
{
  try
  {
    if (argc != 2) throw std::runtime_error("usage: njord_mesh_export <mesh path>");
    const auto *mesh = gz::common::MeshManager::Instance()->Load(argv[1]);
    if (!mesh) throw std::runtime_error("Gazebo could not load mesh");
    std::vector<gz::math::Vector3d> vertices;
    std::vector<std::array<unsigned int, 3>> faces;
    for (unsigned int n = 0; n < mesh->SubMeshCount(); ++n)
    {
      const auto sub = mesh->SubMeshByIndex(n).lock();
      if (!sub || sub->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES ||
          sub->IndexCount() % 3 || !sub->HasValidIndices())
        throw std::runtime_error("Mesh must contain valid indexed triangles");
      const auto base = static_cast<unsigned int>(vertices.size());
      for (unsigned int i = 0; i < sub->VertexCount(); ++i)
      {
        const auto v = sub->Vertex(i);
        if (!std::isfinite(v.X()) || !std::isfinite(v.Y()) || !std::isfinite(v.Z()))
          throw std::runtime_error("Non-finite vertex");
        vertices.push_back(v);
      }
      for (unsigned int i = 0; i < sub->IndexCount(); i += 3)
        faces.push_back({base + sub->Index(i), base + sub->Index(i+1), base + sub->Index(i+2)});
    }
    if (faces.empty()) throw std::runtime_error("Empty mesh");
    std::cout << std::setprecision(17) << "{\"vertices\":[";
    for (size_t i = 0; i < vertices.size(); ++i)
    {
      if (i) std::cout << ',';
      const auto &v = vertices[i];
      std::cout << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
    }
    std::cout << "],\"faces\":[";
    for (size_t i = 0; i < faces.size(); ++i)
    {
      if (i) std::cout << ',';
      const auto &f = faces[i];
      std::cout << '[' << f[0] << ',' << f[1] << ',' << f[2] << ']';
    }
    std::cout << "]}\n";
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
