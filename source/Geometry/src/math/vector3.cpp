#include "Geometry/math/vector3.hpp"
#include "Geometry/math/point3.hpp"

#include "RLogSU/logger.hpp"

namespace Geometry::Math {

Vector3::Vector3(const Point3& point) : x_(point.GetX()), y_(point.GetY()), z_(point.GetZ()) 
{

}

bool Vector3::operator== (const Vector3& other) const  
{
    return this->x_ == other.x_ && this->y_ == other.y_ && this->z_ == other.z_;
}

bool Vector3::operator!= (const Vector3& other) const  
{
    return !(*this == other);
}

Vector3 Vector3::operator+ (const Vector3& other) const  
{ 
    return Vector3(this->x_ + other.x_, this->y_ + other.y_, this->z_ + other.z_); 
}

Vector3 Vector3::operator- () const  
{
    return Vector3(-x_, -y_, -z_);
}

Vector3 Vector3::operator- (const Vector3& other) const
{
    return *this + (-other);
}

Vector3 Vector3::operator* (double scalar) const 
{
    return Vector3(x_ * scalar, y_ * scalar, z_ * scalar);
}

Vector3 Vector3::operator/ (double scalar) const 
{
    RLSU_VERIFY(scalar != 0);
    return Vector3(x_ / scalar, y_ / scalar, z_ / scalar);
}

double Vector3::operator* (const Vector3& other) const         // скалярное умножение 
{
    return this->x_ * other.x_ + this->y_ * other.y_ + this->z_ * other.z_;
}

Vector3 Vector3::operator^ (const Vector3& other) const         // векторное умножение 
{
    double new_x = this->y_ * other.z_ - this->z_ * other.y_;
    double new_y = this->x_ * other.z_ - this->z_ * other.x_;
    double new_z = this->x_ * other.y_ - this->y_ * other.x_;

    return Vector3(new_x, new_y, new_z);
}


}