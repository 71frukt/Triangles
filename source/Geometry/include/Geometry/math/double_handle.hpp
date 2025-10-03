#pragma once

#include <iostream>

namespace Geometry::Math {
    
constexpr double CmpEps = 1e-8;

[[nodiscard]] inline bool DoubleEq(double a, double b, double eps = CmpEps)
{
    return std::abs(a - b) <= eps;
}

[[nodiscard]] inline bool DoubleGE(double a, double b, double eps = CmpEps)
{
    return a - b >= -eps;
}

[[nodiscard]] inline bool DoubleG(double a, double b, double eps = CmpEps)
{
    return a - b > -eps;
}

[[nodiscard]] inline bool DoubleB(double a, double b, double eps = CmpEps)
{
    return a - b < -eps;
}

[[nodiscard]] inline bool DoublePositive(double a, double eps = CmpEps)
{
    return a >= -eps;
}

[[nodiscard]] inline bool DoubleZero(double a, double eps = CmpEps)
{
    return std::abs(a) <= eps;
}

} // namespace Geometry::Math