/**
 * @file demo_obstacle_mpc.cpp
 * @brief Visual demonstration of integrated obstacle avoidance MPC
 * 
 * This demo uses HumanoidObstacleAvoidanceMPC directly without the SPG pipeline.
 * It's useful for:
 * - Testing obstacle avoidance in isolation
 * - Tuning MPC weights for obstacle avoidance
 * - Benchmarking performance
 * 
 * Interactive controls:
 * - Press 'R' to reset simulation
 * - Press 'P' to pause/resume
 * - Press 'S' for step mode
 * - ESC to exit
 * 
 * Command-line arguments (optional):
 *   --config <file>  : JSON config file with start_pos, target_pos, obstacles, mpc_weights
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include "MPCObstacleAvoidingSimulator.hpp"
#include "spg/integrated_mpc/HumanoidObstacleAvoidanceMPC.hpp"

// Simple JSON parser for config
struct ObstacleConfig {
    Eigen::Vector2d position;
    double radius;
    Eigen::Vector2d velocity;
};

struct Config {
    Eigen::Vector3d start_pos = Eigen::Vector3d(0.0, -3.0, 0.0);
    Eigen::Vector3d start_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d target_pos = Eigen::Vector3d(0.0, 3.0, 0.0);
    
    std::vector<ObstacleConfig> obstacles = {
        // Default obstacles disabled - they block direct path making problem very hard
        // Enable via config file or uncomment for testing
        // {{-1.0, 0.0}, 0.3, {0.5, 0.0}},  // Left obstacle moving right
        // {{ 1.0, 0.0}, 0.3, {-0.5, 0.0}}, // Right obstacle moving left
        // {{ 0.0, 1.5}, 0.25, {0.0, 0.3}}  // Center obstacle moving up
    };
    
    // MPC parameters
    double dt = 0.15;
    int horizon = 20;
    double robot_radius = 0.3;
    double obstacle_margin = 0.2;
    
    // MPC weights (acados format)
    bool has_mpc_weights = false;
    double Q_pos = 15.0;
    double Q_theta = 2.0;
    double Q_vel = 0.5;
    double R_accel = 0.1;
    double Q_term_pos = 100.0;
    double Q_term_theta = 10.0;
    double Q_term_vel = 5.0;
    double collision_weight = 1000.0;
};

Config loadConfig(const std::string& filename) {
    Config cfg;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open config file: " << filename << std::endl;
        std::cerr << "Using default configuration" << std::endl;
        return cfg;
    }
    
    std::string line;
    bool in_obstacles = false;
    int obs_idx = 0;
    
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#' || line.find("//") == 0) continue;
        
        // Parse start position
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
        }
        // Parse target position
        else if (line.find("\"target_pos\"") != std::string::npos) {
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
        }
        // Parse MPC parameters
        else if (line.find("\"dt\"") != std::string::npos) {
            size_t pos = line.find(":");
            cfg.dt = std::stod(line.substr(pos + 1));
        }
        else if (line.find("\"horizon\"") != std::string::npos) {
            size_t pos = line.find(":");
            cfg.horizon = std::stoi(line.substr(pos + 1));
        }
        else if (line.find("\"robot_radius\"") != std::string::npos) {
            size_t pos = line.find(":");
            cfg.robot_radius = std::stod(line.substr(pos + 1));
        }
        else if (line.find("\"obstacle_margin\"") != std::string::npos) {
            size_t pos = line.find(":");
            cfg.obstacle_margin = std::stod(line.substr(pos + 1));
        }
        // Parse MPC weights
        else if (line.find("\"mpc_weights\"") != std::string::npos) {
            cfg.has_mpc_weights = true;
            while (std::getline(file, line) && line.find("}") == std::string::npos) {
                if (line.find("\"Q_pos\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.Q_pos = std::stod(line.substr(pos + 1));
                } else if (line.find("\"Q_theta\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.Q_theta = std::stod(line.substr(pos + 1));
                } else if (line.find("\"Q_vel\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.Q_vel = std::stod(line.substr(pos + 1));
                } else if (line.find("\"R_accel\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.R_accel = std::stod(line.substr(pos + 1));
                } else if (line.find("\"Q_term_pos\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.Q_term_pos = std::stod(line.substr(pos + 1));
                } else if (line.find("\"Q_term_theta\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.Q_term_theta = std::stod(line.substr(pos + 1));
                } else if (line.find("\"Q_term_vel\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.Q_term_vel = std::stod(line.substr(pos + 1));
                } else if (line.find("\"collision_weight\"") != std::string::npos) {
                    size_t pos = line.find(":"); cfg.collision_weight = std::stod(line.substr(pos + 1));
                }
            }
        }
    }
    
    return cfg;
}

int main(int argc, char** argv) {
    // Parse command-line arguments
    Config config;
    bool run_headless = false;
    int max_steps = 1000;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config = loadConfig(argv[++i]);
        } else if (arg == "--headless") {
            run_headless = true;
        } else if (arg == "--max-steps" && i + 1 < argc) {
            max_steps = std::stoi(argv[++i]);
        }
    }
    
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       Integrated Obstacle Avoidance MPC Demo               ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Start: (" << config.start_pos.x() << ", " << config.start_pos.y() 
              << ", " << config.start_pos.z() << ")" << std::endl;
    std::cout << "  Target: (" << config.target_pos.x() << ", " << config.target_pos.y() 
              << ", " << config.target_pos.z() << ")" << std::endl;
    std::cout << "  dt: " << config.dt << " s, horizon: " << config.horizon << std::endl;
    std::cout << "  Robot radius: " << config.robot_radius << " m" << std::endl;
    std::cout << "  Obstacle margin: " << config.obstacle_margin << " m" << std::endl;
    std::cout << "  Obstacles: " << config.obstacles.size() << std::endl;
    
    if (config.has_mpc_weights) {
        std::cout << "\nMPC Weights:" << std::endl;
        std::cout << "  Q_pos: " << config.Q_pos << ", Q_theta: " << config.Q_theta << std::endl;
        std::cout << "  Q_vel: " << config.Q_vel << ", R_accel: " << config.R_accel << std::endl;
        std::cout << "  Q_term_pos: " << config.Q_term_pos << ", Q_term_theta: " << config.Q_term_theta << std::endl;
        std::cout << "  Q_term_vel: " << config.Q_term_vel << std::endl;
        std::cout << "  Collision weight: " << config.collision_weight << std::endl;
    }
    std::cout << std::endl;
    
    // Set up MPC parameters
    spg::integrated_mpc::ObstacleAvoidanceMPCParams mpc_params;
    mpc_params.dt = config.dt;
    mpc_params.horizon = config.horizon;
    mpc_params.robot_radius = config.robot_radius;
    mpc_params.obstacle_margin = config.obstacle_margin;
    
    if (config.has_mpc_weights) {
        mpc_params.Q_pos = config.Q_pos;
        mpc_params.Q_theta = config.Q_theta;
        mpc_params.Q_vel = config.Q_vel;
        mpc_params.R_accel = config.R_accel;
        mpc_params.Q_term_pos = config.Q_term_pos;
        mpc_params.Q_term_theta = config.Q_term_theta;
        mpc_params.Q_term_vel = config.Q_term_vel;
        mpc_params.collision_weight = config.collision_weight;
    }
    
    // Create simulator
    MPCObstacleAvoidingSimulator simulator(
        config.start_pos,
        config.start_vel,
        config.target_pos,
        mpc_params
    );
    
    // Set obstacles
    std::vector<Eigen::Vector2d> positions;
    std::vector<double> radii;
    std::vector<Eigen::Vector2d> velocities;
    
    for (const auto& obs : config.obstacles) {
        positions.push_back(obs.position);
        radii.push_back(obs.radius);
        velocities.push_back(obs.velocity);
    }
    
    simulator.setObstacles(positions, radii, velocities);
    
    // Run simulation
    if (run_headless) {
        std::cout << "Running headless simulation (max " << max_steps << " steps)..." << std::endl;
        simulator.runWithoutVisualization(max_steps, config.dt);
        
        // Print results
        auto stats = simulator.getMPCStats();
        std::cout << "\nSimulation Results:" << std::endl;
        std::cout << "  Completed: " << (simulator.isCompleted() ? "YES" : "NO") << std::endl;
        std::cout << "  Simulation time: " << simulator.getSimulationTime() << " s" << std::endl;
        std::cout << "  MPC avg time: " << stats.avg_time_ms << " ms" << std::endl;
    } else {
        std::cout << "Starting interactive visualization..." << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  R - Reset simulation" << std::endl;
        std::cout << "  P - Pause/Resume" << std::endl;
        std::cout << "  S - Step mode" << std::endl;
        std::cout << "  ESC - Exit\n" << std::endl;
        
        simulator.run();
    }
    
    return 0;
}
