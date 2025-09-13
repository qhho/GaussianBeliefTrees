#include <iostream>
#include "discrete_sst.cpp"   // directly include your planner class for testing

int main() {
    try {
        std::cout << "Running DiscreteContinuousExample with SST..." << std::endl;

        // No config file (will use defaults)
        DiscreteContinuousExample example("");

        // Run planning
        example.planWithDiscreteTime();

        std::cout << "Test completed successfully ✅" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}