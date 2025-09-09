#include "barrier_rrt_main.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    BarrierRRTMain barrier_planner;
    
    // Load configuration from file if provided
    if (argc > 1) {
        std::string config_file = argv[1];
        std::cout << "Loading configuration from: " << config_file << std::endl;
        barrier_planner.loadConfig(config_file);
        
        // Load scene file
        std::string scene_file = barrier_planner.scene_name_;
        std::cout << "Loading scene from: " << scene_file << std::endl;
        barrier_planner.loadScene(scene_file);
    } else {
        std::cout << "Using default configuration" << std::endl;
        // Load default scene
        barrier_planner.loadScene("scenes/2d_barrier_circle.yaml");
    }
    
    barrier_planner.planWithBarrierRRT();
    
    return 0;
} 