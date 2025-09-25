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

// Updates the violation struct for a given field and value
inline void updateViolation(Violation& violation, const std::string& field, double violation_value) {
    if (violation_value > 0) {
        violation.count++;
    }
    if (violation.values.find(field) == violation.values.end()) {
        violation.values[field] = violation_value;
    }
    if (violation_value > violation.values[field]) {
        violation.collisionfree = false;
    } else {
        violation.values[field] = violation_value;
    }
}

} // namespace subtarget
} // namespace spg
