#include <iomanip>
#include <iostream>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Utilities.h>

int main() {
  using namespace lanelet;

  // A lanelet is defined by a left and a right boundary. Coordinates are in metres.
  LineString3d left{utils::getId(),
                    {Point3d{utils::getId(), 0., 2., 0.}, Point3d{utils::getId(), 20., 2., 0.}}};
  LineString3d right{utils::getId(),
                     {Point3d{utils::getId(), 0., -2., 0.}, Point3d{utils::getId(), 20., -2., 0.}}};
  Lanelet lanelet{utils::getId(), left, right};

  const auto centerline = lanelet.centerline();
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "Lanelet id: " << lanelet.id() << '\n';
  std::cout << "Centerline: (" << centerline.front().x() << ", " << centerline.front().y() << ") -> ("
            << centerline.back().x() << ", " << centerline.back().y() << ")\n";
  std::cout << "Length: " << geometry::length2d(lanelet) << " m\n";
  return 0;
}
