#include "utils/MathUtils.h"

#include <cmath>
#include <numbers>

double MathUtils::RoundToStep(double value, double step) {
  return floor(value / step) * step;
}