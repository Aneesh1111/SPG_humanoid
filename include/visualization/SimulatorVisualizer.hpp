#pragma once
#include <vector>
#include <Eigen/Dense>

struct GLFWwindow;

struct RobotState {
    Eigen::Vector3d pose; // x, y, theta
    Eigen::Vector3d vel;
};

struct ObstacleState {
    Eigen::Vector2d pos;
    double radius;
    bool active;
};

struct BallState {
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
};

struct FieldParams {
    double width, height, penalty_width, penalty_height, border_margin, circle_radius;
};

class SimulatorVisualizer {
public:
    SimulatorVisualizer(const FieldParams& params);
    ~SimulatorVisualizer();
    
    bool initialize();
    void cleanup();
    bool shouldClose() const;
    void beginFrame();
    void endFrame();
    
    void drawField();
    void drawRobot(const RobotState& robot);
    void drawObstacles(const std::vector<ObstacleState>& obstacles);
    void drawBall(const BallState& ball);
    void drawTarget(const Eigen::Vector3d& target_pos);
    void drawSubtarget(const Eigen::Vector3d& subtarget_pos);
    void drawTrajectories(const std::vector<Eigen::Vector2d>& robot_traj,
                          const std::vector<std::vector<Eigen::Vector2d>>& obs_traj);
    void render(const RobotState& robot,
                const std::vector<ObstacleState>& obstacles,
                const BallState& ball,
                const std::vector<Eigen::Vector2d>& robot_traj,
                const std::vector<std::vector<Eigen::Vector2d>>& obs_traj,
                const Eigen::Vector3d& target_pos,
                const Eigen::Vector3d& subtarget_pos,
                double simulation_time = 0.0,
                int step_count = 0,
                bool simulation_completed = false);
    
    // Get the current simulation speed multiplier (1.0 = real-time, 0.1 = 10x slower, 2.0 = 2x faster)
    float getSimulationSpeed() const { return simulation_speed_; }
    
    // Control methods for reset and step mode
    bool shouldReset() const { return reset_requested_; }
    void clearResetRequest() { reset_requested_ = false; }
    bool isStepMode() const { return step_mode_; }
    bool shouldStep() const { return step_requested_; }
    void clearStepRequest() { step_requested_ = false; }
    
private:
    FieldParams params;
    GLFWwindow* window;
    bool initialized;
    float simulation_speed_; // Simulation speed multiplier
    bool reset_requested_;   // Flag to request simulation reset
    bool step_mode_;         // Whether we're in step mode
    bool step_requested_;    // Flag to request next step in step mode
};
