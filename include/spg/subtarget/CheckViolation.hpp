#pragma once
#include <string>
#include <unordered_map>
#include "spg/Init.hpp"

namespace spg {
namespace subtarget {

struct Violation {
    bool collisionfree = true;
    int count = 0;
    std::unordered_map<std::string, double> values;
};

// Updates the violation struct for a given field and value - EXACTLY like MATLAB
inline void updateViolation(Violation& violation, const std::string& field, double violation_value) {
    // Count positive violations
    if (violation_value > 0) {
        violation.count++;
    }
    
    // Initialize field if not exists 
    if (violation.values.find(field) == violation.values.end()) {
        violation.values[field] = 0;
    }
    
    // MATLAB logic: if violation_value > stored_value, mark as NOT collision-free
    if (violation_value > violation.values[field]) {
        violation.collisionfree = false;
    } else {
        // Update stored value to the better (lower) violation
        violation.values[field] = violation_value;
    }
}

} // namespace subtarget
} // namespace spg
