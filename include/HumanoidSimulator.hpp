#pragma once
#include "spg/Init.hpp"
#include "spg/HumanoidUtils.hpp"
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/NextSample.hpp"
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

class HumanoidSimulator {
public:
    explicit HumanoidSimulator(const spg::SPGState& initial_state);
    ~HumanoidSimulator();

    void run();
    void runWithoutVisualization(int max_steps, double timestep);

private:
    // Core simulation functions
    void updateHumanoidSetpoint();
    void integrateMotion(double dt);
    void checkTargetReached();
    
    // Visualization functions
    bool initializeWindow();
    void render();
    void renderField();
    void renderRobot();
    void renderTarget();
    void renderTrajectory();
    void renderHumanoidInfo();
    void cleanup();

    // Humanoid-specific trajectory generation
    void generateHumanoidTrajectory();
    
    // State
    spg::SPGState state_;
    std::vector<spg::setpoint::Segment> current_segments_;
    
    // Simulation control
    bool simulation_completed_;
    bool paused_;
    double simulation_time_;
    int step_count_;
    
    // Target checking
    double target_tolerance_;
    double velocity_tolerance_;
    
    // Visualization
    GLFWwindow* window_;
    bool use_visualization_;
    
    // Trajectory history for visualization
    std::vector<Eigen::Vector3d> trajectory_history_;
    size_t max_trajectory_points_;
    
    // Humanoid-specific parameters
    double robot_orientation_;
    Eigen::Vector3d effective_vmax_;
    Eigen::Vector3d effective_amax_;
    double movement_efficiency_;
};

// Implementation
HumanoidSimulator::HumanoidSimulator(const spg::SPGState& initial_state) 
    : state_(initial_state), simulation_completed_(false), paused_(false),
      simulation_time_(0.0), step_count_(0), target_tolerance_(0.1), 
      velocity_tolerance_(0.1), window_(nullptr), use_visualization_(false),
      max_trajectory_points_(1000), robot_orientation_(0.0), movement_efficiency_(1.0) {
    
    // Initialize trajectory history
    trajectory_history_.reserve(max_trajectory_points_);
    trajectory_history_.push_back(state_.input.robot.p);
    
    // Get initial robot orientation
    robot_orientation_ = state_.input.robot.IMU_orientation(2);
    
    std::cout << "🤖 HumanoidSimulator initialized" << std::endl;
    std::cout << "Robot position: [" << state_.input.robot.p.transpose() << "]" << std::endl;
    std::cout << "Target position: [" << state_.target.p.transpose() << "]" << std::endl;
    std::cout << "Robot orientation: " << robot_orientation_ * 180.0 / M_PI << "°" << std::endl;
}

HumanoidSimulator::~HumanoidSimulator() {
    cleanup();
}

void HumanoidSimulator::updateHumanoidSetpoint() {
    // Calculate movement direction for humanoid constraints
    Eigen::Vector3d movement_direction = state_.target.p - state_.input.robot.p;
    double distance_to_target = movement_direction.norm();
    
    if (distance_to_target < target_tolerance_) {
        simulation_completed_ = true;
        return;
    }
    
    // Get effective limits considering robot orientation and movement direction
    effective_vmax_ = spg::humanoid::calculateEffectiveVelocityLimits(
        state_.par, robot_orientation_, movement_direction);
    effective_amax_ = spg::humanoid::calculateEffectiveAccelerationLimits(
        state_.par, robot_orientation_, movement_direction);
    Eigen::Vector3d effective_dmax = effective_amax_; // Use same for deceleration
    
    // Calculate movement efficiency for display
    Eigen::Vector3d local_movement = spg::humanoid::globalToLocalVelocity(movement_direction, robot_orientation_);
    double sideways_component = std::abs(local_movement(0));
    double forward_component = std::abs(local_movement(1));
    double total = sideways_component + forward_component;
    if (total > 1e-6) {
        double forward_ratio = forward_component / total;
        double sideways_ratio = sideways_component / total;
        movement_efficiency_ = forward_ratio * state_.par.forward_efficiency + 
                              sideways_ratio * state_.par.sideways_efficiency;
    }
    
    // Use humanoid-aware trajectory generation
    generateHumanoidTrajectory();
}

void HumanoidSimulator::generateHumanoidTrajectory() {
    // Current state
    Eigen::Vector3d current_pos = state_.input.robot.p;
    Eigen::Vector3d current_vel = state_.input.robot.v;
    Eigen::Vector3d target_pos = state_.target.p;
    Eigen::Vector3d target_vel = state_.target.v;
    
    // Initialize segments
    current_segments_.resize(4); // acceleration, constant velocity, deceleration, stop
    
    // Use humanoid-aware balancing
    Eigen::Vector3d balanced_vmax, balanced_amax, balanced_dmax;
    spg::setpoint::balanceXY_humanoid(current_segments_, 
                                     current_pos, current_vel, 
                                     target_pos, target_vel,
                                     state_.par, robot_orientation_,
                                     balanced_vmax, balanced_amax, balanced_dmax);
    
    // Update setpoint using the first segment
    if (!current_segments_.empty() && current_segments_[0].dt.norm() > 1e-6) {
        state_.setpoint.p = current_segments_[0].p;
        state_.setpoint.v = current_segments_[0].v;
        state_.setpoint.a = current_segments_[0].a;
    } else {
        // Fallback: direct control
        Eigen::Vector3d pos_error = target_pos - current_pos;
        state_.setpoint.v = pos_error.normalized() * std::min(pos_error.norm() / state_.par.Ts, effective_vmax_.head<2>().norm());
        state_.setpoint.a = (state_.setpoint.v - current_vel) / state_.par.Ts;
    }
}

void HumanoidSimulator::integrateMotion(double dt) {
    // Simple Euler integration
    Eigen::Vector3d acceleration = state_.setpoint.a;
    
    // Apply humanoid velocity constraints
    Eigen::Vector3d desired_vel = state_.input.robot.v + acceleration * dt;
    
    // Transform to local frame to apply constraints
    Eigen::Vector3d local_vel = spg::humanoid::globalToLocalVelocity(desired_vel, robot_orientation_);
    
    // Clamp to humanoid limits
    local_vel(0) = std::max(-state_.par.vmax_move_x, std::min(state_.par.vmax_move_x, local_vel(0))); // sideways
    local_vel(1) = std::max(-state_.par.vmax_move_y, std::min(state_.par.vmax_move_y, local_vel(1))); // forward
    
    // Apply efficiency factors
    local_vel(0) *= state_.par.sideways_efficiency;
    local_vel(1) *= state_.par.forward_efficiency;
    
    // Transform back to global frame
    Eigen::Vector3d constrained_vel = spg::humanoid::localToGlobalVelocity(local_vel, robot_orientation_);
    
    // Update robot state
    state_.input.robot.v = constrained_vel;
    state_.input.robot.p += constrained_vel * dt;
    
    // Update simulation time
    simulation_time_ += dt;
    step_count_++;
    
    // Add to trajectory history
    if (trajectory_history_.size() >= max_trajectory_points_) {
        trajectory_history_.erase(trajectory_history_.begin());
    }
    trajectory_history_.push_back(state_.input.robot.p);
}

void HumanoidSimulator::checkTargetReached() {
    Eigen::Vector3d position_error = state_.input.robot.p - state_.target.p;
    double position_error_norm = position_error.norm();
    double velocity_norm = state_.input.robot.v.norm();
    
    if (position_error_norm < target_tolerance_ && velocity_norm < velocity_tolerance_) {
        simulation_completed_ = true;
        std::cout << "🎯 Target reached!" << std::endl;
        std::cout << "Final position error: " << position_error_norm << " m" << std::endl;
        std::cout << "Final velocity: " << velocity_norm << " m/s" << std::endl;
        std::cout << "Total simulation time: " << simulation_time_ << " s" << std::endl;
        std::cout << "Total steps: " << step_count_ << std::endl;
        std::cout << "Average movement efficiency: " << movement_efficiency_ * 100 << "%" << std::endl;
    }
}

bool HumanoidSimulator::initializeWindow() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    window_ = glfwCreateWindow(1200, 800, "Humanoid Robot Simulator", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    return true;
}

void HumanoidSimulator::run() {
    use_visualization_ = true;
    
    if (!initializeWindow()) {
        std::cerr << "Failed to initialize visualization" << std::endl;
        return;
    }

    std::cout << "🎮 Starting humanoid simulation with visualization..." << std::endl;
    std::cout << "Press SPACE to pause/resume, ESC to exit" << std::endl;

    while (!glfwWindowShouldClose(window_) && !simulation_completed_) {
        glfwPollEvents();

        // Handle keyboard input
        if (glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS) {
            static bool space_pressed = false;
            if (!space_pressed) {
                paused_ = !paused_;
                space_pressed = true;
            }
        } else {
            static bool space_pressed = false;
            space_pressed = false;
        }

        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            break;
        }

        // Update simulation
        if (!paused_) {
            updateHumanoidSetpoint();
            integrateMotion(state_.par.Ts);
            checkTargetReached();
        }

        // Render
        render();
    }

    cleanup();
}

void HumanoidSimulator::runWithoutVisualization(int max_steps, double timestep) {
    use_visualization_ = false;
    
    std::cout << "Running humanoid simulation without visualization..." << std::endl;
    
    for (int step = 0; step < max_steps && !simulation_completed_; ++step) {
        updateHumanoidSetpoint();
        integrateMotion(timestep);
        checkTargetReached();
        
        if (step % 50 == 0) {  // Print progress every 50 steps
            std::cout << "Step " << step << ": Robot at [" 
                      << state_.input.robot.p.transpose() << "], efficiency: " 
                      << movement_efficiency_ * 100 << "%" << std::endl;
        }
    }
}

void HumanoidSimulator::render() {
    // Clear screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);

    // Set up 2D view
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    glViewport(0, 0, width, height);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // Set camera to show field area
    double field_margin = 2.0;
    double field_width = 10.0 + field_margin;
    double field_height = 8.0 + field_margin;
    glOrtho(-field_width/2, field_width/2, -field_height/2, field_height/2, -1, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render field
    renderField();
    
    // Render trajectory
    renderTrajectory();
    
    // Render robot
    renderRobot();
    
    // Render target
    renderTarget();

    // Render ImGui interface
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    renderHumanoidInfo();
    
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window_);
}

void HumanoidSimulator::renderField() {
    // Field boundaries
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(-4.5f, -3.0f);
    glVertex2f(4.5f, -3.0f);
    glVertex2f(4.5f, 3.0f);
    glVertex2f(-4.5f, 3.0f);
    glEnd();
    
    // Center line
    glBegin(GL_LINES);
    glVertex2f(0.0f, -3.0f);
    glVertex2f(0.0f, 3.0f);
    glEnd();
    
    // Center circle
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 32; ++i) {
        float angle = 2.0f * M_PI * i / 32;
        glVertex2f(0.75f * cosf(angle), 0.75f * sinf(angle));
    }
    glEnd();
}

void HumanoidSimulator::renderTrajectory() {
    if (trajectory_history_.size() < 2) return;
    
    glColor3f(0.5f, 0.5f, 0.8f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pos : trajectory_history_) {
        glVertex2f(pos(0), pos(1));
    }
    glEnd();
}

void HumanoidSimulator::renderRobot() {
    Eigen::Vector3d pos = state_.input.robot.p;
    
    // Robot body (circle)
    glColor3f(0.2f, 0.6f, 1.0f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(pos(0), pos(1));
    for (int i = 0; i <= 32; ++i) {
        float angle = 2.0f * M_PI * i / 32;
        glVertex2f(pos(0) + 0.15f * cosf(angle), pos(1) + 0.15f * sinf(angle));
    }
    glEnd();
    
    // Robot orientation indicator
    glColor3f(1.0f, 1.0f, 0.0f);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex2f(pos(0), pos(1));
    glVertex2f(pos(0) + 0.25f * cosf(robot_orientation_ + M_PI/2), 
               pos(1) + 0.25f * sinf(robot_orientation_ + M_PI/2));
    glEnd();
    
    // Velocity vector
    Eigen::Vector3d vel = state_.input.robot.v;
    if (vel.norm() > 0.1) {
        glColor3f(0.0f, 1.0f, 0.0f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glVertex2f(pos(0), pos(1));
        glVertex2f(pos(0) + vel(0) * 0.5f, pos(1) + vel(1) * 0.5f);
        glEnd();
    }
}

void HumanoidSimulator::renderTarget() {
    Eigen::Vector3d target = state_.target.p;
    
    // Target (green circle)
    glColor3f(0.2f, 1.0f, 0.2f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(target(0), target(1));
    for (int i = 0; i <= 32; ++i) {
        float angle = 2.0f * M_PI * i / 32;
        glVertex2f(target(0) + 0.1f * cosf(angle), target(1) + 0.1f * sinf(angle));
    }
    glEnd();
    
    // Target border
    glColor3f(0.0f, 0.8f, 0.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 32; ++i) {
        float angle = 2.0f * M_PI * i / 32;
        glVertex2f(target(0) + 0.1f * cosf(angle), target(1) + 0.1f * sinf(angle));
    }
    glEnd();
}

void HumanoidSimulator::renderHumanoidInfo() {
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::SetNextWindowSize(ImVec2(350, 400));
    
    if (ImGui::Begin("🤖 Humanoid Robot Status", nullptr, 
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        
        // Simulation status
        ImGui::Text("Simulation Time: %.2f s", simulation_time_);
        ImGui::Text("Steps: %d", step_count_);
        ImGui::Text("Status: %s", paused_ ? "PAUSED" : (simulation_completed_ ? "COMPLETED" : "RUNNING"));
        
        ImGui::Separator();
        
        // Robot state
        Eigen::Vector3d pos = state_.input.robot.p;
        Eigen::Vector3d vel = state_.input.robot.v;
        ImGui::Text("Robot Position: [%.2f, %.2f, %.2f]", pos(0), pos(1), pos(2));
        ImGui::Text("Robot Velocity: [%.2f, %.2f, %.2f]", vel(0), vel(1), vel(2));
        ImGui::Text("Robot Orientation: %.1f°", robot_orientation_ * 180.0 / M_PI);
        
        ImGui::Separator();
        
        // Target info
        Eigen::Vector3d target = state_.target.p;
        double distance = (pos - target).norm();
        ImGui::Text("Target: [%.2f, %.2f, %.2f]", target(0), target(1), target(2));
        ImGui::Text("Distance to Target: %.2f m", distance);
        
        ImGui::Separator();
        
        // Humanoid constraints
        ImGui::Text("🤖 Humanoid Constraints:");
        ImGui::Text("Max Sideways (X): %.1f m/s", state_.par.vmax_move_x);
        ImGui::Text("Max Forward (Y):  %.1f m/s", state_.par.vmax_move_y);
        ImGui::Text("Forward Efficiency: %.0f%%", state_.par.forward_efficiency * 100);
        ImGui::Text("Sideways Efficiency: %.0f%%", state_.par.sideways_efficiency * 100);
        ImGui::Text("Current Efficiency: %.1f%%", movement_efficiency_ * 100);
        
        ImGui::Separator();
        
        // Movement analysis
        Eigen::Vector3d movement_dir = target - pos;
        if (movement_dir.norm() > 1e-6) {
            Eigen::Vector3d local_dir = spg::humanoid::globalToLocalVelocity(movement_dir, robot_orientation_);
            ImGui::Text("🎯 Movement Analysis:");
            ImGui::Text("Global Dir: [%.2f, %.2f]", movement_dir(0), movement_dir(1));
            ImGui::Text("Local Dir: [%.2f, %.2f]", local_dir(0), local_dir(1));
            ImGui::Text("(sideways, forward)");
            
            double forward_ratio = std::abs(local_dir(1)) / (std::abs(local_dir(0)) + std::abs(local_dir(1)) + 1e-6);
            ImGui::Text("Forward Component: %.0f%%", forward_ratio * 100);
        }
        
        ImGui::Separator();
        ImGui::Text("Controls:");
        ImGui::Text("SPACE - Pause/Resume");
        ImGui::Text("ESC - Exit");
    }
    ImGui::End();
}

void HumanoidSimulator::cleanup() {
    if (use_visualization_ && window_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        
        glfwDestroyWindow(window_);
        glfwTerminate();
        
        window_ = nullptr;
    }
}