#pragma once

#include "SPG.hpp"
#include <vector>
#include <memory>

namespace spg {

// ============================================================================
// Forward Declarations
// ============================================================================

struct SCollisionResult;
class CCollisionChecker;
class IReplanStrategy;

// ============================================================================
// CSubtargetLayer - Main Interface
// ============================================================================

/**
 * @brief Manages subtarget generation with collision checking and replanning
 * 
 * Encapsulates the entire subtarget layer logic:
 * - Collision-free path checking
 * - Dynamic replanning when collision predicted
 */
class CSubtargetLayer {
public:
    // Constructor
    explicit CSubtargetLayer(const CSPGPar& parameters);
    
    /**
     * @brief Main processing method - generates subtarget from current state
     * 
     * @param state SPG state (reads robot, target, obstacles; writes subtarget)
     */
    void Process(CSPGState& state);
    
    // Query methods
    bool IsCollisionFree() const { return mCollisionFree; }
    double GetComputationTime() const { return mComputationTimeMs; }

private:
    // Parameters
    const CSPGPar& mParameters;
    
    // State
    bool mCollisionFree;
    double mComputationTimeMs;
    
    // Components (using composition)
    std::unique_ptr<CCollisionChecker> mpCollisionChecker;
    std::vector<std::unique_ptr<IReplanStrategy>> mReplanStrategies;
    
    // Private helper methods
    void InitializeStrategies();
    void CheckDirectPath(CSPGState& state);
    void ApplyReplanning(CSPGState& state);
};

// ============================================================================
// CCollisionChecker - Validates Path Safety
// ============================================================================

struct SCollisionResult {
    bool isFree;
    double minDistance;
    int obstacleIndex;
    double intersectionX;
    double intersectionY;
    
    SCollisionResult() : isFree(false), minDistance(1e10), 
                        obstacleIndex(-1), intersectionX(0), intersectionY(0) {}
};

/**
 * @brief Checks collision along path using ray-circle intersection
 */
class CCollisionChecker {
public:
    explicit CCollisionChecker(double margin);
    
    /**
     * @brief Check if straight-line path from start to goal is collision-free
     * 
     * @param state Full SPG state for comprehensive checking
     * @param start Starting position [x, y, theta]
     * @param goal Goal position [x, y, theta]
     * @param obstaclesX Obstacle x positions
     * @param obstaclesY Obstacle y positions
     * @param obstaclesVX Obstacle x velocities
     * @param obstaclesVY Obstacle y velocities
     * @param obstaclesR Obstacle radii
     * @return Collision result with detailed info
     */
    SCollisionResult CheckPath(const CSPGState& state,
                              const double* start, const double* goal,
                              const std::vector<double>& obstaclesX,
                              const std::vector<double>& obstaclesY,
                              const std::vector<double>& obstaclesVX,
                              const std::vector<double>& obstaclesVY,
                              const std::vector<double>& obstaclesR);
    
    /**
     * @brief Check single point for collision
     */
    bool IsPointSafe(double x, double y,
                    const std::vector<double>& obstaclesX,
                    const std::vector<double>& obstaclesY,
                    const std::vector<double>& obstaclesR);
    
    void SetMargin(double margin) { mMargin = margin; }
    double GetMargin() const { return mMargin; }

private:
    double mMargin;  // Safety distance around obstacles
    
    // Ray-circle intersection test
    double RayCircleIntersection(double rayStartX, double rayStartY,
                                double rayDirX, double rayDirY,
                                double circleX, double circleY,
                                double circleRadius);
};

// ============================================================================
// IReplanStrategy - Strategy Pattern for Different Replanning Methods
// ============================================================================

/**
 * @brief Abstract base class for replanning strategies
 */
class IReplanStrategy {
public:
    virtual ~IReplanStrategy() = default;
    
    /**
     * @brief Check if this strategy should be applied
     */
    virtual bool ShouldApply(const CSPGState& state, const SCollisionResult& collision) = 0;
    
    /**
     * @brief Generate new subtarget using this strategy
     * 
     * @param state Current SPG state
     * @param collision Collision info from direct path check
     * @param subtarget Output: new subtarget (modified in place)
     * @return true if replanning succeeded
     */
    virtual bool Replan(const CSPGState& state, const SCollisionResult& collision,
                       CRobotState& subtarget) = 0;
    
    virtual const char* GetName() const = 0;
};

/**
 * @brief Go-to-target strategy: Use target directly if no collision
 */
class CDirectToTargetStrategy : public IReplanStrategy {
public:
    bool ShouldApply(const CSPGState& state, const SCollisionResult& collision) override;
    bool Replan(const CSPGState& state, const SCollisionResult& collision,
               CRobotState& subtarget) override;
    const char* GetName() const override { return "DirectToTarget"; }
};

/**
 * @brief Tangent strategy: Navigate around obstacle using tangent path
 */
class CTangentStrategy : public IReplanStrategy {
public:
    bool ShouldApply(const CSPGState& state, const SCollisionResult& collision) override;
    bool Replan(const CSPGState& state, const SCollisionResult& collision,
               CRobotState& subtarget) override;
    const char* GetName() const override { return "Tangent"; }

private:
    void ComputeTangentPoints(double robotX, double robotY,
                             double obsX, double obsY, double obsR,
                             double targetX, double targetY,
                             double& tangent1X, double& tangent1Y,
                             double& tangent2X, double& tangent2Y);
};

/**
 * @brief Fallback strategy: Stop at current position for ANY stop condition
 * (collision, out of bounds, stop command, etc.)
 */
class CQuickstopStrategy : public IReplanStrategy {
public:
    bool ShouldApply(const CSPGState& state, const SCollisionResult& collision) override;
    bool Replan(const CSPGState& state, const SCollisionResult& collision,
               CRobotState& subtarget) override;
    const char* GetName() const override { return "Quickstop"; }
};

// ============================================================================
// AngleAdjuster - Handles Heading Alignment
// ============================================================================

/**
 * @brief Adjusts approach angle for smooth goal alignment
 */
class CAngleAdjuster {
public:
    /**
     * @brief Compute desired heading for approaching target
     * 
     * @param robot Current robot position [x, y, theta]
     * @param target Target position [x, y, theta]
     * @param ball Ball position [x, y] (for ball-facing requirements)
     * @return Desired heading angle (radians)
     */
    static double ComputeApproachAngle(const double* robot, const double* target,
                                      const double* ball);
};

} // namespace spg
