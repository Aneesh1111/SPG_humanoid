/**
 * @file demo_humanoid_mpc.cpp
 * @brief Visual demonstration of HumanoidMPC integration with SPG
 * 
 * This demo shows how to switch between:
 * 1. Traditional MSL (Middle Size League) setpoint generation
 * 2. HumanoidMPC-based trajectory generation
 * 
 * Interactive controls:
 * - Press 'M' to toggle between MSL and HumanoidMPC modes
 * - Press 'R' to reset simulation
 * - Click to set new target
 * 
 * Command-line arguments (optional):
 *   --config <file>  : JSON config file with start_pos, target_pos, horizon, mpc_weights
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

// Simple JSON parser for config (very basic, just for our needs)
struct Config {
    Eigen::Vector3d start_pos = Eigen::Vector3d(0, 3, 0);
    Eigen::Vector3d target_pos = Eigen::Vector3d(0, -3, 0);
    int prediction_horizon = 20;
    bool has_mpc_weights = false;
    double q_pos = 1.0, q_phi = 0.1, qf_pos = 8.0, qf_phi = 1.0;
    double r_vf = 0.1, r_vs = 0.5, r_omega = 0.2;
    double s_vf = 0.2, s_vs = 0.8, s_omega = 0.4;
};

Config loadConfig(const std::string& filename) {
    Config cfg;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open config file: " << filename << std::endl;
        return cfg;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Very simple parser - look for key-value pairs
        if (line.find("\"start_pos\"") != std::string::npos) {
            std::getline(file, line);
            if (line.find("\"x\"") != std::string::npos) {
                size_t pos = line.find(":");
                cfg.start_pos.x() = std::stod(line.substr(pos + 1));
            }
            std::getline(file, line);
            if (line.find("\"y\"") != std::string::npos) {
                size_t pos = line.find(":");
                cfg.start_pos.y() = std::stod(line.substr(pos + 1));
            }
            std::getline(file, line);
            if (line.find("\"theta\"") != std::string::npos) {
                size_t pos = line.find(":");
                cfg.start_pos.z() = std::stod(line.substr(pos + 1));
            }
        } else if (line.find("\"target_pos\"") != std::string::npos) {
            std::getline(file, line);
            if (line.find("\"x\"") != std::string::npos) {
                size_t pos = line.find(":");
                cfg.target_pos.x() = std::stod(line.substr(pos + 1));
            }
            std::getline(file, line);
            if (line.find("\"y\"") != std::string::npos) {
                size_t pos = line.find(":");
                cfg.target_pos.y() = std::stod(line.substr(pos + 1));
            }
            std::getline(file, line);
            if (line.find("\"theta\"") != std::string::npos) {
                size_t pos = line.find(":");
                cfg.target_pos.z() = std::stod(line.substr(pos + 1));
            }
        } else if (line.find("\"prediction_horizon\"") != std::string::npos) {
            size_t pos = line.find(":");
            cfg.prediction_horizon = std::stoi(line.substr(pos + 1));
        } else if (line.find("\"mpc_weights\"") != std::string::npos) {
            cfg.has_mpc_weights = true;
            // Read MPC weights
            while (std::getline(file, line) && line.find("}") == std::string::npos) {
                if (line.find("\"q_pos\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.q_pos = std::stod(line.substr(pos + 1));
                } else if (line.find("\"q_phi\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.q_phi = std::stod(line.substr(pos + 1));
                } else if (line.find("\"qf_pos\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.qf_pos = std::stod(line.substr(pos + 1));
                } else if (line.find("\"qf_phi\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.qf_phi = std::stod(line.substr(pos + 1));
                } else if (line.find("\"r_vf\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.r_vf = std::stod(line.substr(pos + 1));
                } else if (line.find("\"r_vs\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.r_vs = std::stod(line.substr(pos + 1));
                } else if (line.find("\"r_omega\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.r_omega = std::stod(line.substr(pos + 1));
                } else if (line.find("\"s_vf\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.s_vf = std::stod(line.substr(pos + 1));
                } else if (line.find("\"s_vs\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.s_vs = std::stod(line.substr(pos + 1));
                } else if (line.find("\"s_omega\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.s_omega = std::stod(line.substr(pos + 1));
                }
            }
        }
    }
    return cfg;
}

int main(int argc, char** argv) {
    // Parse command-line arguments
    Config config;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config = loadConfig(argv[++i]);
        }
    }
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║      SPG Controller Mode Demonstration (Visual)             ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    std::cout << "Controls:" << std::endl;
    std::cout << "  M - Toggle between MSL and HumanoidMPC modes" << std::endl;
    std::cout << "  R - Reset simulation" << std::endl;
    std::cout << "  Click - Set new target position" << std::endl;
    std::cout << "  ESC - Exit\n" << std::endl;
    
    // Initial parameters (from config or defaults)
    Eigen::Vector3d p_initial = config.start_pos;
    Eigen::Vector3d v_initial(0, 0, 0);
    int npredict = config.prediction_horizon;
    int nobstacles = 0;
    int nintercept_positions = 15;
    Eigen::Vector2d p_initial_ball(0, 0);
    Eigen::Vector2d v_initial_ball(0, 0);

    std::cout << "Configuration:" << std::endl;
    std::cout << "  Start position: (" << p_initial.x() << ", " << p_initial.y() << ", " << p_initial.z() << ")" << std::endl;
    std::cout << "  Target position: (" << config.target_pos.x() << ", " << config.target_pos.y() << ", " << config.target_pos.z() << ")" << std::endl;
    std::cout << "  Prediction horizon: " << npredict << std::endl;

    // Initialize state using spg::Init
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, 
                                    p_initial_ball, v_initial_ball, nintercept_positions);

    // ========================================
    // Choose initial mode: MSL or HumanoidMPC
    // ========================================
    initial_state.par.use_humanoid_mpc = true;  // Start with HumanoidMPC mode
    
    // Save MPC weights to config file if provided (Set.cpp will read them)
    if (config.has_mpc_weights) {
        std::ofstream weights_file("mpc_weights_override.json");
        if (weights_file.is_open()) {
            weights_file << "{" << std::endl;
            weights_file << "  \"q_pos\": " << config.q_pos << "," << std::endl;
            weights_file << "  \"q_phi\": " << config.q_phi << "," << std::endl;
            weights_file << "  \"qf_pos\": " << config.qf_pos << "," << std::endl;
            weights_file << "  \"qf_phi\": " << config.qf_phi << "," << std::endl;
            weights_file << "  \"r_vf\": " << config.r_vf << "," << std::endl;
            weights_file << "  \"r_vs\": " << config.r_vs << "," << std::endl;
            weights_file << "  \"r_omega\": " << config.r_omega << "," << std::endl;
            weights_file << "  \"s_vf\": " << config.s_vf << "," << std::endl;
            weights_file << "  \"s_vs\": " << config.s_vs << "," << std::endl;
            weights_file << "  \"s_omega\": " << config.s_omega << std::endl;
            weights_file << "}" << std::endl;
            weights_file.close();
        }
        
        std::cout << "\nMPC Weights (custom):" << std::endl;
        std::cout << "  q_pos=" << config.q_pos << ", q_phi=" << config.q_phi << std::endl;
        std::cout << "  qf_pos=" << config.qf_pos << ", qf_phi=" << config.qf_phi << std::endl;
        std::cout << "  r_vf=" << config.r_vf << ", r_vs=" << config.r_vs << ", r_omega=" << config.r_omega << std::endl;
        std::cout << "  s_vf=" << config.s_vf << ", s_vs=" << config.s_vs << ", s_omega=" << config.s_omega << std::endl;
    }
    
    if (initial_state.par.use_humanoid_mpc) {
        std::cout << "\nStarting in HUMANOID MPC MODE" << std::endl;
        std::cout << "  • Body-frame control (forward/sideways/yaw)" << std::endl;
        std::cout << "  • MPC optimization with smoothness cost" << std::endl;
        std::cout << "  • Physical limits: vf=1.5 m/s, vs=0.5 m/s, ω=2.0 rad/s" << std::endl;
    } else {
        std::cout << "\nStarting in MSL MODE (Traditional)" << std::endl;
        std::cout << "  • Omnidirectional control" << std::endl;
        std::cout << "  • Segment-based trajectory planning" << std::endl;
        std::cout << "  • Fast computation, no optimization" << std::endl;
    }
    std::cout << std::endl;

    // Set target position (goal) from config
    Eigen::Vector3d target = config.target_pos;
    initial_state.input.robot.target = target;
    initial_state.input.robot.target_vel = Eigen::Vector3d(0, 0, 0);

    // Initialize obstacles
    std::cout << "Setting up " << nobstacles << " obstacles..." << std::endl;
    for (int i = 0; i < nobstacles; ++i) {
        // Place obstacles in a line between start and goal
        double t = (i + 1.0) / (nobstacles + 1.0);
        Eigen::Vector2d obs_pos = (1.0 - t) * p_initial.head<2>() + t * target.head<2>();
        
        // Add some offset to make path more interesting
        double offset_angle = M_PI / 2.0;
        double offset_mag = 1.5 * (i % 2 == 0 ? 1 : -1);
        obs_pos += offset_mag * Eigen::Vector2d(cos(offset_angle), sin(offset_angle));
        
        initial_state.input.obstacles.p[i] = obs_pos;
        
        // Small random velocities
        initial_state.input.obstacles.v[i] = Eigen::Vector2d(
            0.3 * cos(i * 1.5), 
            0.3 * sin(i * 1.5)
        );
        
        initial_state.input.obstacles.r[i] = 0.35;  // 35cm radius
        initial_state.input.obstacles.active[i] = true;
        
        std::cout << "  Obstacle " << i << ": (" 
                  << initial_state.input.obstacles.p[i].x() << ", " 
                  << initial_state.input.obstacles.p[i].y() << ")" << std::endl;
    }
    std::cout << std::endl;

    // Print mode information
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Press 'M' in the visualization window to toggle modes      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;

    // Create and run simulator
    SPGSimulator simulator(initial_state);
    simulator.run();

    std::cout << "\nSimulation complete." << std::endl;
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    Quick Reference                           ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nTo switch modes programmatically:" << std::endl;
    std::cout << "  d.par.use_humanoid_mpc = true;   // HumanoidMPC mode" << std::endl;
    std::cout << "  d.par.use_humanoid_mpc = false;  // MSL mode (default)" << std::endl;
    std::cout << "\nSame inputs/outputs for both modes:" << std::endl;
    std::cout << "  Input:  d.subtarget.p (goal position)" << std::endl;
    std::cout << "  Output: d.setpoint.{p,v,a} (commands)" << std::endl;
    std::cout << "\n" << std::endl;
    
    return 0;
}
