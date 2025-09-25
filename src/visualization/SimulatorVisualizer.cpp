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
    : params(params), window(nullptr), initialized(false) {}

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
    double rx[4], ry[4];
    rx[0] = x + r * cos(theta);
    ry[0] = y + r * sin(theta);
    rx[1] = x + r * cos(theta + 2.5);
    ry[1] = y + r * sin(theta + 2.5);
    rx[2] = x + r * cos(theta - 2.5);
    ry[2] = y + r * sin(theta - 2.5);
    rx[3] = rx[0]; ry[3] = ry[0];
    ImPlot::PlotLine("Robot", rx, ry, 4);
}

void SimulatorVisualizer::drawObstacles(const std::vector<ObstacleState>& obstacles) {
    for (const auto& obs : obstacles) {
        if (obs.active) {
            ImPlot::PlotScatter("Obstacles", &obs.pos.x(), &obs.pos.y(), 1);
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
        ImPlot::PlotLine("Robot Traj", x.data(), y.data(), x.size());
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
    double x = target_pos.x(), y = target_pos.y();
    
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
}

void SimulatorVisualizer::render(const RobotState& robot,
                                 const std::vector<ObstacleState>& obstacles,
                                 const BallState& ball,
                                 const std::vector<Eigen::Vector2d>& robot_traj,
                                 const std::vector<std::vector<Eigen::Vector2d>>& obs_traj,
                                 const Eigen::Vector3d& target_pos) {
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
    ImGui::Text("Robot Position: (%.3f, %.3f, %.3f)", robot.pose.x(), robot.pose.y(), robot.pose.z());
    ImGui::Text("Robot Velocity: (%.3f, %.3f, %.3f)", robot.vel.x(), robot.vel.y(), robot.vel.z());
    ImGui::Text("Target Position: (%.3f, %.3f, %.3f)", target_pos.x(), target_pos.y(), target_pos.z());
    ImGui::Text("Ball Position: (%.3f, %.3f)", ball.pos.x(), ball.pos.y());
    
    // Calculate distance to target
    double distance_to_target = (robot.pose.head<2>() - target_pos.head<2>()).norm();
    ImGui::Text("Distance to Target: %.3f", distance_to_target);
    ImGui::Separator();
    
    // Make the plot take up most of the remaining window space
    ImVec2 plot_size = ImVec2(window_w - 20, window_h - 140); // More space for the additional info text
    
    if (ImPlot::BeginPlot("SPG Robot Field View", plot_size, ImPlotFlags_Equal)) {
        drawField();
        drawRobot(robot);
        drawObstacles(obstacles);
        drawBall(ball);
        drawTarget(target_pos);
        drawTrajectories(robot_traj, obs_traj);
        ImPlot::EndPlot();
    }
    ImGui::End();
}
