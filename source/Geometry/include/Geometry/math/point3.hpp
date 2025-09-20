#pragma once

#include "Geometry/math/vector3.hpp"

namespace Geometry::Math {

class Point3
{
public:
    Point3(const double x, const double y, const double z) : rad_vec_(x, y, z) {};
    Point3() = default;
    
    [[nodiscard]] double GetX() const { return rad_vec_.GetX(); }
    [[nodiscard]] double GetY() const { return rad_vec_.GetY(); }
    [[nodiscard]] double GetZ() const { return rad_vec_.GetZ(); }

    [[nodiscard]] Point3  operator+(const Vector3& vec)   const  { return Point3 (rad_vec_.GetX() + vec  .GetX(), rad_vec_.GetY() + vec.GetY(),   rad_vec_.GetZ() + vec  .GetZ()); };
    [[nodiscard]] Vector3 operator-(const Point3 & other) const  { return Vector3(other   .GetX() - this->GetX(), other   .GetY() - this->GetY(), other   .GetZ() - this->GetZ()); }

private:
    Vector3 rad_vec_;
};

}