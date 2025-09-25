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
    void drawTrajectories(const std::vector<Eigen::Vector2d>& robot_traj,
                          const std::vector<std::vector<Eigen::Vector2d>>& obs_traj);
    void render(const RobotState& robot,
                const std::vector<ObstacleState>& obstacles,
                const BallState& ball,
                const std::vector<Eigen::Vector2d>& robot_traj,
                const std::vector<std::vector<Eigen::Vector2d>>& obs_traj,
                const Eigen::Vector3d& target_pos);
private:
    FieldParams params;
    GLFWwindow* window;
    bool initialized;
};
