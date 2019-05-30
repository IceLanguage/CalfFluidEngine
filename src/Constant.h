#ifndef _CalfFluidEngine_Constant_
#define _CalfFluidEngine_Constant_
#include <limits>
namespace CalfFluidEngine{
constexpr float kEpsilonF = std::numeric_limits<float>::epsilon();
constexpr double kEpsilonD = std::numeric_limits<double>::epsilon();
constexpr float kPiF = 3.14159265358979323846264338327950288f;
constexpr double kPiD = 3.14159265358979323846264338327950288;
constexpr float kMaxF = std::numeric_limits<float>::max();
constexpr double kMaxD = std::numeric_limits<double>::max();
}
#endif
