#pragma once

#include <cmath>
#include <iostream>

#include "RLogSU/logger.hpp"

namespace Geometry::Math {

class Point3;

class Vector3
{
public:
    Vector3() = default;
    Vector3(const double x, const double y, const double z) : x_(x), y_(y), z_(z) {};
    Vector3(const Point3 point);

    [[nodiscard]] double GetX   () const { return x_; }
    [[nodiscard]] double GetY   () const { return y_; }
    [[nodiscard]] double GetZ   () const { return z_; }

    [[nodiscard]] double GetLen2() const { return x_ * x_ + y_ * y_ + z_ * z_; }
    [[nodiscard]] double GetLen () const { return std::sqrt(GetLen2()); }


    [[nodiscard]] Vector3 operator+(  const Vector3& other ) const;
    [[nodiscard]] Vector3 operator-()                        const;
    [[nodiscard]] Vector3 operator-(  const Vector3& other ) const;
    [[nodiscard]] Vector3 operator/(  const double   scalar) const;
    [[nodiscard]] double  operator*(  const Vector3& other ) const;     // скалярное
    [[nodiscard]] Vector3 operator^(  const Vector3& other ) const;     // векторное


    [[nodiscard]] Vector3 Normalized() const { return *this / GetLen(); }
                  Vector3 Normalize ()       { double scale = GetLen(); x_ /= scale; y_ /= scale; z_ /= scale; return *this;}


private:
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
};


}