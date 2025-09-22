#pragma once

#include <iostream>

namespace Geometry::Math {

[[nodiscard]] inline bool DoubleEq(double a, double b, double eps = 1e-7)
{
    return std::abs(a - b) <= eps * std::max(std::abs(a), std::abs(b));
}

[[nodiscard]] inline bool DoubleZero(double a, double eps = 1e-7)
{
    return std::abs(a) <= eps * a;
}

} // namespace Geometry::Math