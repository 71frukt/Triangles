#pragma once

#include <iostream>

namespace Geometry::Math {

[[nodiscard]] inline bool DoubleEq(double a, double b, double eps = 1e-10)
{
    return std::abs(a - b) <= eps;
}

[[nodiscard]] inline bool DoubleGE(double a, double b, double eps = 1e-10)
{
    return a - b >= -eps;
}

[[nodiscard]] inline bool DoublePositive(double a, double eps = 1e-10)
{
    return a >= -eps;
}

[[nodiscard]] inline bool DoubleZero(double a, double eps = 1e-10)
{
    return std::abs(a) <= eps;
}

} // namespace Geometry::Math