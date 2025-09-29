#pragma once

#include "Geometry/math/vector3.hpp"

namespace Geometry::Math {

extern bool DoubleEq(double a, double b, double eps);

class Point3
{
public:
    Point3(const double x, const double y, const double z) : rad_vec_(x, y, z) {}
    Point3(const Vector3& rad_vec)                         : rad_vec_(rad_vec) {}
    Point3() = default;
    
    [[nodiscard]] double GetX() const { return rad_vec_.GetX(); }
    [[nodiscard]] double GetY() const { return rad_vec_.GetY(); }
    [[nodiscard]] double GetZ() const { return rad_vec_.GetZ(); }

                  double SetX(double x) { return rad_vec_.SetX(x); }
                  double SetY(double y) { return rad_vec_.SetY(y); }
                  double SetZ(double z) { return rad_vec_.SetZ(z); }

    [[nodiscard]] bool    operator== (const Point3 & other) const  { return this->rad_vec_ == other.rad_vec_; }
    [[nodiscard]] bool    operator!= (const Point3 & other) const  { return this->rad_vec_ != other.rad_vec_; }
    [[nodiscard]] Point3  operator+  (const Vector3& vec)   const  { return Point3 (rad_vec_.GetX() + vec  .GetX(), rad_vec_.GetY() + vec  .GetY(), rad_vec_.GetZ() + vec  .GetZ()); };
    [[nodiscard]] Vector3 operator-  (const Point3 & other) const  { return Vector3(this   ->GetX() - other.GetX(), this   ->GetY() - other.GetY(), this   ->GetZ() - other.GetZ()); }

private:
    Vector3 rad_vec_;
};

}