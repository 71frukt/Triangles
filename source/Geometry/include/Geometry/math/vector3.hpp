#pragma once

#include <cmath>

#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"

#include "Geometry/math/double_handle.hpp"

namespace Geometry::Math {

extern bool DoubleEq  (double a, double b, double eps);
extern bool DoubleZero(double a, double eps);

class Point3;

class Vector3
{
public:
    Vector3(const double x, const double y, const double z) : x_(x), y_(y), z_(z) {};
    Vector3(const Point3& point);
    Vector3() : x_(1), y_(0), z_(0) { }

    [[nodiscard]] double GetX   () const { return x_; }
    [[nodiscard]] double GetY   () const { return y_; }
    [[nodiscard]] double GetZ   () const { return z_; }

                  double SetX (double x) { return x_ = x; }
                  double SetY (double y) { return y_ = y; }
                  double SetZ (double z) { return z_ = z; }

    [[nodiscard]] double GetLen2() const { return x_ * x_ + y_ * y_ + z_ * z_; }
    [[nodiscard]] double GetLen () const { return std::sqrt(GetLen2()); }


    [[nodiscard]] bool     operator== (const Vector3& other ) const;
    [[nodiscard]] bool     operator!= (const Vector3& other ) const;
    [[nodiscard]] Vector3  operator+  (const Vector3& other ) const;
    [[nodiscard]] Vector3  operator-  ()                      const;
    [[nodiscard]] Vector3  operator-  (const Vector3& other ) const;
    [[nodiscard]] Vector3  operator*  (const double   scalar) const;
    [[nodiscard]] Vector3  operator/  (const double   scalar) const;
    [[nodiscard]] double   operator*  (const Vector3& other ) const;     // скалярное

    [[nodiscard]] Vector3  operator^  (const Vector3& other ) const;     // векторное
 
                  Vector3& operator+= (const Vector3& other ) { return *this = (*this + other ); }
                  Vector3& operator-= (const Vector3& other ) { return *this = (*this - other ); }
                  Vector3& operator*= (const double   scalar) { return *this = (*this * scalar); }
                  Vector3& operator/= (const double   scalar) { return *this = (*this / scalar); }


    [[nodiscard]] Vector3 Normalized () const;
                  Vector3 Normalize  ();

    [[nodiscard]] bool    IsZero     () const { return (DoubleZero(x_)) && DoubleZero(y_) && DoubleZero(z_); }

    [[nodiscard]] bool    Collinear  (const Vector3& other) const { return (*this ^ other).IsZero(); }

    [[nodiscard]] bool    Codirected (const Vector3& other) const
    {
        return (DoublePositive(this->x_ * other.x_))
             && DoublePositive(this->y_ * other.y_)
             && DoublePositive(this->z_ * other.z_);
    }

    [[nodiscard]] bool    Normal     (const Vector3& other) const { return DoubleZero(*this * other); }

    void Dump(const std::string& name = "some_vector") const;

private:
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
};

[[nodiscard]] Vector3 operator*(double scalar, const Vector3& vec);


}