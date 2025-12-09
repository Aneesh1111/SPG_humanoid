#include "visualization/SimulatorVisualizer.hpp"
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <iostream>

static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

SimulatorVisualizer::SimulatorVisualizer(const FieldParams& params) 
    : params(params), window(nullptr), initialized(false), simulation_speed_(1.0f), 
      reset_requested_(false), step_mode_(false), step_requested_(false) {}

SimulatorVisualizer::~SimulatorVisualizer() {
    cleanup();
}

bool SimulatorVisualizer::initialize() {
    if (initialized) return true;
    
    // Setup GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window
    window = glfwCreateWindow(1600, 1000, "SPG Simulator", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    initialized = true;
    return true;
}

void SimulatorVisualizer::cleanup() {
    if (!initialized) return;
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
    
    initialized = false;
}

bool SimulatorVisualizer::shouldClose() const {
    return window ? glfwWindowShouldClose(window) : true;
}

void SimulatorVisualizer::beginFrame() {
    if (!initialized) return;
    
    glfwPollEvents();
    
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void SimulatorVisualizer::endFrame() {
    if (!initialized) return;
    
    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
}

void SimulatorVisualizer::drawField() {
    double w = params.width * 0.5, h = params.height * 0.5;
    double px[5] = {-w, -w, w, w, -w};
    double py[5] = {-h, h, h, -h, -h};
    ImPlot::PlotLine("Field", px, py, 5);
    // Penalty areas
    double pw = params.penalty_width * 0.5, ph = params.penalty_height;
    double ppx[5] = {-pw, -pw, pw, pw, -pw};
    double ppy[5] = {h, h-ph, h-ph, h, h};
    ImPlot::PlotLine("PenaltyAreaTop", ppx, ppy, 5);
    for (int i = 0; i < 5; ++i) ppy[i] = -ppy[i];
    ImPlot::PlotLine("PenaltyAreaBottom", ppx, ppy, 5);
    // Center circle
    std::vector<double> cx, cy;
    for (int i = 0; i < 64; ++i) {
        double a = 2*M_PI*i/64;
        cx.push_back(params.circle_radius*cos(a));
        cy.push_back(params.circle_radius*sin(a));
    }
    ImPlot::PlotLine("CenterCircle", cx.data(), cy.data(), cx.size());
}

void SimulatorVisualizer::drawRobot(const RobotState& robot) {
    double x = robot.pose.x(), y = robot.pose.y(), theta = robot.pose.z();
    double r = 0.25; // robot radius
    
    // Adjust theta so 0 degrees points towards positive Y (opposition goal)
    double adjusted_theta = theta - M_PI/2;
    
    // Draw robot body as a triangle
    double rx[4], ry[4];
    rx[0] = x + r * cos(adjusted_theta);           // Front point (heading direction)
    ry[0] = y + r * sin(adjusted_theta);
    rx[1] = x + r * cos(adjusted_theta + 2.5);     // Back-left
    ry[1] = y + r * sin(adjusted_theta + 2.5);
    rx[2] = x + r * cos(adjusted_theta - 2.5);     // Back-right
    ry[2] = y + r * sin(adjusted_theta - 2.5);
    rx[3] = rx[0]; ry[3] = ry[0];                  // Close the triangle
    ImPlot::PlotLine("Robot", rx, ry, 4);
    
    // Draw heading arrow (extends further to make rotation very visible)
    double arrow_length = 0.4;
    double arrow_x[2] = {x, x + arrow_length * cos(adjusted_theta)};
    double arrow_y[2] = {y, y + arrow_length * sin(adjusted_theta)};
    ImPlot::PlotLine("Robot Heading", arrow_x, arrow_y, 2);
    
    // Draw arrowhead at the tip
    double arrowhead_size = 0.1;
    double arrowhead_angle = M_PI / 6; // 30 degrees
    double arrow_tip_x = arrow_x[1];
    double arrow_tip_y = arrow_y[1];
    
    double left_x[2] = {arrow_tip_x, 
                        arrow_tip_x - arrowhead_size * cos(adjusted_theta - arrowhead_angle)};
    double left_y[2] = {arrow_tip_y, 
                        arrow_tip_y - arrowhead_size * sin(adjusted_theta - arrowhead_angle)};
    ImPlot::PlotLine("Arrow Left", left_x, left_y, 2);
    
    double right_x[2] = {arrow_tip_x, 
                         arrow_tip_x - arrowhead_size * cos(adjusted_theta + arrowhead_angle)};
    double right_y[2] = {arrow_tip_y, 
                         arrow_tip_y - arrowhead_size * sin(adjusted_theta + arrowhead_angle)};
    ImPlot::PlotLine("Arrow Right", right_x, right_y, 2);
    
    // Draw a circle around the robot to show its footprint
    std::vector<double> circle_x, circle_y;
    int n_points = 16;
    for (int j = 0; j <= n_points; ++j) {
        double angle = 2.0 * M_PI * j / n_points;
        circle_x.push_back(x + r * cos(angle));
        circle_y.push_back(y + r * sin(angle));
    }
    ImPlot::PlotLine("Robot Circle", circle_x.data(), circle_y.data(), circle_x.size());
}

void SimulatorVisualizer::drawObstacles(const std::vector<ObstacleState>& obstacles) {
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obs = obstacles[i];
        if (obs.active) {
            // Draw obstacle as a circle with its radius
            std::vector<double> circle_x, circle_y;
            int n_points = 16; // Number of points to approximate circle
            for (int j = 0; j <= n_points; ++j) {
                double angle = 2.0 * M_PI * j / n_points;
                circle_x.push_back(obs.pos.x() + obs.radius * cos(angle));
                circle_y.push_back(obs.pos.y() + obs.radius * sin(angle));
            }
            
            // Create unique label for each obstacle
            std::string label = "Obstacle" + std::to_string(i);
            ImPlot::PlotLine(label.c_str(), circle_x.data(), circle_y.data(), circle_x.size());
            
            // Also draw center point
            ImPlot::PlotScatter(("ObstacleCenter" + std::to_string(i)).c_str(), &obs.pos.x(), &obs.pos.y(), 1);
        }
    }
}

void SimulatorVisualizer::drawBall(const BallState& ball) {
    ImPlot::PlotScatter("Ball", &ball.pos.x(), &ball.pos.y(), 1);
}

void SimulatorVisualizer::drawTrajectories(const std::vector<Eigen::Vector2d>& robot_traj,
                                           const std::vector<std::vector<Eigen::Vector2d>>& obs_traj) {
    if (!robot_traj.empty()) {
        std::vector<double> x, y;
        for (const auto& p : robot_traj) { x.push_back(p.x()); y.push_back(p.y()); }
        
        // Draw trajectory as a line
        ImPlot::PlotLine("Robot Traj Line", x.data(), y.data(), x.size());
        
        // Draw trajectory points as individual markers
        ImPlot::PlotScatter("Robot Traj Points", x.data(), y.data(), x.size());
    }
    for (const auto& traj : obs_traj) {
        if (!traj.empty()) {
            std::vector<double> x, y;
            for (const auto& p : traj) { x.push_back(p.x()); y.push_back(p.y()); }
            ImPlot::PlotLine("Obs Traj", x.data(), y.data(), x.size());
        }
    }
}

void SimulatorVisualizer::drawTarget(const Eigen::Vector3d& target_pos) {
    double x = target_pos.x(), y = target_pos.y(), theta = target_pos.z();
    
    // Draw target as a large X
    double size = 0.3;
    double x1[2] = {x - size, x + size};
    double y1[2] = {y - size, y + size};
    double x2[2] = {x - size, x + size};
    double y2[2] = {y + size, y - size};
    
    ImPlot::PlotLine("Target X1", x1, y1, 2);
    ImPlot::PlotLine("Target X2", x2, y2, 2);
    
    // Draw target center point
    ImPlot::PlotScatter("Target", &x, &y, 1);
    
    // Draw target orientation arrow (if theta is non-zero)
    if (std::abs(theta) > 0.01) {
        double adjusted_theta = theta - M_PI/2;
        double arrow_length = 0.5;
        double arrow_x[2] = {x, x + arrow_length * cos(adjusted_theta)};
        double arrow_y[2] = {y, y + arrow_length * sin(adjusted_theta)};
        ImPlot::PlotLine("Target Heading", arrow_x, arrow_y, 2);
        
        // Add arrowhead
        double arrowhead_size = 0.15;
        double arrowhead_angle = M_PI / 6;
        double arrow_tip_x = arrow_x[1];
        double arrow_tip_y = arrow_y[1];
        
        double left_x[2] = {arrow_tip_x, 
                            arrow_tip_x - arrowhead_size * cos(adjusted_theta - arrowhead_angle)};
        double left_y[2] = {arrow_tip_y, 
                            arrow_tip_y - arrowhead_size * sin(adjusted_theta - arrowhead_angle)};
        ImPlot::PlotLine("Target Arrow L", left_x, left_y, 2);
        
        double right_x[2] = {arrow_tip_x, 
                             arrow_tip_x - arrowhead_size * cos(adjusted_theta + arrowhead_angle)};
        double right_y[2] = {arrow_tip_y, 
                             arrow_tip_y - arrowhead_size * sin(adjusted_theta + arrowhead_angle)};
        ImPlot::PlotLine("Target Arrow R", right_x, right_y, 2);
    }
}

void SimulatorVisualizer::drawSubtarget(const Eigen::Vector3d& subtarget_pos) {
    double x = subtarget_pos.x(), y = subtarget_pos.y(), theta = subtarget_pos.z();
    
    // Draw subtarget as a diamond shape
    double size = 0.2;
    double diamond_x[5] = {x, x + size, x, x - size, x};
    double diamond_y[5] = {y + size, y, y - size, y, y + size};
    
    ImPlot::PlotLine("Subtarget Diamond", diamond_x, diamond_y, 5);
    
    // Draw subtarget center point
    ImPlot::PlotScatter("Subtarget", &x, &y, 1);
    
    // Draw subtarget orientation arrow (if theta is non-zero)
    if (std::abs(theta) > 0.01) {
        double adjusted_theta = theta - M_PI/2;
        double arrow_length = 0.35;
        double arrow_x[2] = {x, x + arrow_length * cos(adjusted_theta)};
        double arrow_y[2] = {y, y + arrow_length * sin(adjusted_theta)};
        ImPlot::PlotLine("Subtarget Heading", arrow_x, arrow_y, 2);
        
        // Add arrowhead
        double arrowhead_size = 0.1;
        double arrowhead_angle = M_PI / 6;
        double arrow_tip_x = arrow_x[1];
        double arrow_tip_y = arrow_y[1];
        
        double left_x[2] = {arrow_tip_x, 
                            arrow_tip_x - arrowhead_size * cos(adjusted_theta - arrowhead_angle)};
        double left_y[2] = {arrow_tip_y, 
                            arrow_tip_y - arrowhead_size * sin(adjusted_theta - arrowhead_angle)};
        ImPlot::PlotLine("Subtarget Arrow L", left_x, left_y, 2);
        
        double right_x[2] = {arrow_tip_x, 
                             arrow_tip_x - arrowhead_size * cos(adjusted_theta + arrowhead_angle)};
        double right_y[2] = {arrow_tip_y, 
                             arrow_tip_y - arrowhead_size * sin(adjusted_theta + arrowhead_angle)};
        ImPlot::PlotLine("Subtarget Arrow R", right_x, right_y, 2);
    }
}

void SimulatorVisualizer::render(const RobotState& robot,
                                 const std::vector<ObstacleState>& obstacles,
                                 const BallState& ball,
                                 const std::vector<Eigen::Vector2d>& robot_traj,
                                 const std::vector<std::vector<Eigen::Vector2d>>& obs_traj,
                                 const Eigen::Vector3d& target_pos,
                                 const Eigen::Vector3d& subtarget_pos,
                                 double simulation_time,
                                 int step_count,
                                 bool simulation_completed) {
    if (!initialized) return;
    
    // Get window size
    int window_w, window_h;
    glfwGetWindowSize(window, &window_w, &window_h);
    
    // Make the ImGui window fill the entire screen
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(window_w, window_h));
    
    ImGui::Begin("SPG Simulator", nullptr, 
                 ImGuiWindowFlags_NoTitleBar | 
                 ImGuiWindowFlags_NoResize | 
                 ImGuiWindowFlags_NoMove | 
                 ImGuiWindowFlags_NoCollapse);
    
    // Add some status information at the top
    ImGui::Text("Simulation Time: %.3f seconds", simulation_time);
    ImGui::Text("Step Count: %d", step_count);
    if (simulation_completed) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ SIMULATION COMPLETED - Robot reached target!");
    }
    ImGui::Separator();
    ImGui::Text("Robot Position: (%.3f, %.3f, %.3f)", robot.pose.x(), robot.pose.y(), robot.pose.z());
    ImGui::Text("Robot Velocity: (%.3f, %.3f, %.3f)", robot.vel.x(), robot.vel.y(), robot.vel.z());
    
    // Display rotation in both radians and degrees for clarity
    double theta_rad = robot.pose.z();
    double theta_deg = theta_rad * 180.0 / M_PI;
    ImGui::Text("Robot Orientation: %.3f rad (%.1f°)", theta_rad, theta_deg);
    
    ImGui::Text("Target Position: (%.3f, %.3f, %.3f)", target_pos.x(), target_pos.y(), target_pos.z());
    double target_theta_deg = target_pos.z() * 180.0 / M_PI;
    ImGui::Text("Target Orientation: %.3f rad (%.1f°)", target_pos.z(), target_theta_deg);
    
    ImGui::Text("Subtarget Position: (%.3f, %.3f, %.3f)", subtarget_pos.x(), subtarget_pos.y(), subtarget_pos.z());
    ImGui::Text("Ball Position: (%.3f, %.3f)", ball.pos.x(), ball.pos.y());
    
    // Count active obstacles
    int active_obstacles = 0;
    for (const auto& obs : obstacles) {
        if (obs.active) active_obstacles++;
    }
    ImGui::Text("Active Obstacles: %d / %d", active_obstacles, (int)obstacles.size());
    
    // Calculate distances
    double distance_to_target = (robot.pose.head<2>() - target_pos.head<2>()).norm();
    double distance_to_subtarget = (robot.pose.head<2>() - subtarget_pos.head<2>()).norm();
    ImGui::Text("Distance to Target: %.3f", distance_to_target);
    ImGui::Text("Distance to Subtarget: %.3f", distance_to_subtarget);
    
    // Simulation speed control
    ImGui::Text("Simulation Speed Control:");
    ImGui::SliderFloat("Speed", &simulation_speed_, 0.01f, 5.0f, "%.2fx");
    ImGui::SameLine();
    if (ImGui::Button("Reset to Real-time")) {
        simulation_speed_ = 1.0f;
    }
    ImGui::Text("(1.0x = real-time, <1.0x = slower, >1.0x = faster)");
    
    // Simulation control buttons
    ImGui::Text("Simulation Control:");
    if (ImGui::Button("🔄 Reset Simulation")) {
        reset_requested_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Step Mode", &step_mode_)) {
        // When enabling step mode, clear any pending step request
        if (step_mode_) {
            step_requested_ = false;
        }
    }
    
    // Step button (only show when in step mode)
    if (step_mode_) {
        ImGui::SameLine();
        if (ImGui::Button("➤ Next Step")) {
            step_requested_ = true;
        }
        ImGui::Text("Step Mode: Simulation paused. Click 'Next Step' to advance.");
    }
    
    ImGui::Separator();
    
    // Make the plot take up most of the remaining window space
    ImVec2 plot_size = ImVec2(window_w - 20, window_h - 180); // More space for obstacle and other status info
    
    if (ImPlot::BeginPlot("SPG Robot Field View", plot_size, ImPlotFlags_Equal)) {
        drawField();
        drawRobot(robot);
        drawObstacles(obstacles);
        drawBall(ball);
        drawTarget(target_pos);
        drawSubtarget(subtarget_pos);
        drawTrajectories(robot_traj, obs_traj);
        ImPlot::EndPlot();
    }
    ImGui::End();
}
