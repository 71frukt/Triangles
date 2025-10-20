#include "Geometry/math/vector3.hpp"
#include "Geometry/math/double_handle.hpp"
#include "Geometry/math/point3.hpp"

#include "RLogSU/logger.hpp"

namespace Geometry::Math {

Vector3::Vector3(const Point3& point) : x_(point.GetX()), y_(point.GetY()), z_(point.GetZ()) 
{

}

bool Vector3::operator== (const Vector3& other) const  
{
    return DoubleEq(this->x_, other.x_) && DoubleEq(this->y_, other.y_) && DoubleEq(this->z_, other.z_);
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

Vector3 operator*(double scalar, const Vector3& vector) 
{
    return Vector3(scalar * vector.GetX(), scalar * vector.GetY(), scalar * vector.GetZ());
}

Vector3 Vector3::operator/ (double scalar) const 
{
    RLSU_ASSERT(scalar != 0);
    return Vector3(x_ / scalar, y_ / scalar, z_ / scalar);
}

double Vector3::operator* (const Vector3& other) const         // скалярное умножение 
{
    return this->x_ * other.x_ + this->y_ * other.y_ + this->z_ * other.z_;
}

Vector3 Vector3::operator^ (const Vector3& other) const         // векторное умножение 
{
    double new_x = this->y_ * other.z_ - this->z_ * other.y_;
    double new_y = this->z_ * other.x_ - this->x_ * other.z_;
    double new_z = this->x_ * other.y_ - this->y_ * other.x_;

    return Vector3(new_x, new_y, new_z);
}


Vector3 Vector3::Normalized () const
{
    if (IsZero())
        return *this;
    else
        return *this / GetLen();
}

Vector3 Vector3::Normalize()
{
    double scale = GetLen();

    if (scale == 0)
        return *this;
    
    else
    {
        x_ /= scale;    
        y_ /= scale;
        z_ /= scale;
        return *this;
    }
}

void Vector3::Dump(const std::string& name) const
{
    RLSU_LOG("\n");
    RLSU_LOG("'{}' [{}] {{\n", name, static_cast<const void*>(this));
    RLSU_BASETAB_INCREACE;

    RLSU_LOG("({:.5g}, {:.5g}, {:.5g})\n", GetX(), GetY(), GetZ());
    RLSU_LOG("length = {:.5g}\n", GetLen());
    
    RLSU_BASETAB_DECREACE;
    RLSU_LOG("}}\n");
}


}