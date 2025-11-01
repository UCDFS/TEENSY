#include "helpers.h"

namespace Helper {

// Simple linear relation based on your gear ratio and wheel size
float rpm_to_kmh(float rpm) {
    return 0.01775f * rpm;
}

} // namespace Helper
