#ifndef ARM_PLANNING_COMMON_FUNCTIONS_H
#define ARM_PLANNING_COMMON_FUNCTIONS_H

template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}
#endif
