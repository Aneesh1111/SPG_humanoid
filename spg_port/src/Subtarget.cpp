#include "SPG.hpp"
#include "SubtargetManager.hpp"
#include <chrono>

namespace spg {
namespace subtarget {

// Static instance for performance (avoid repeated allocation)
static std::unique_ptr<CSubtargetLayer> spLayer = nullptr;

void Subtarget(SPGState& state) {
    // Initialize on first call
    if (!spLayer) {
        spLayer = std::make_unique<CSubtargetLayer>(state.par);
    }
    
    // Run the OOP subtarget processing
    spLayer->Process(state);
}

} // namespace subtarget
} // namespace spg
