#include <iostream>
#include <Eigen/Dense>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

int main() {
    std::cout << "Starting debug demo..." << std::endl;
    
    try {
        // Initial parameters
        Eigen::Vector3d p_initial(3, 0, 0);
        Eigen::Vector3d v_initial(0, 3, 0);
        int npredict = 20;
        int nobstacles = 10;
        int nintercept_positions = 15;
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);

        std::cout << "Calling spg::Init..." << std::endl;
        auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, p_initial_ball, v_initial_ball, nintercept_positions);
        
        std::cout << "Setting target..." << std::endl;
        initial_state.target.p = Eigen::Vector3d(-1, -1, 0);
        initial_state.target.v = Eigen::Vector3d(0, 0, 0);

        std::cout << "Creating simulator..." << std::endl;
        SPGSimulator simulator(initial_state);
        
        std::cout << "Starting single step..." << std::endl;
        simulator.step();
        
        std::cout << "Step completed successfully." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return 1;
    }
    
    return 0;
}
