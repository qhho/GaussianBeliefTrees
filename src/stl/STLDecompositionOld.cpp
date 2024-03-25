#include "STLBeliefDecomposition.h"

double square(double value){
    return value*value;
}

ompl::control::BeliefDecomposition::BeliefDecomposition(int dim, const base::RealVectorBounds &b, int discretization_level) : Decomposition(dim, b){

    discretization_level_ = discretization_level;
}

ompl::control::BeliefDecomposition::BeliefDecomposition(int dim, const base::RealVectorBounds &b) : Decomposition(dim, b){

    discretization_level_ = 1;
}

int coordToRegion(std::vector<double> coord, int discretization_level){
    int rid = -1;

    if (discretization_level == 2){
        if (coord[1] >= 90.0 && coord[0] >= 90.0){
            rid = 0; //goal region
        }
        else if (coord[1] < -90.0 && coord[0] < 90.0){
            rid = 3;
        }
        else if (coord[1] < -90.0 && coord[0] > 90.0){
            rid = 5;
        }
        else if (coord[1] < 10.0 && coord[0] < 50.0){
            rid = 2;
        }
        else if (coord[1] < 10.0 && coord[0] >= 50.0){
            rid = 4;
        }
        // else if (coord[1] > -90.0){
        //     rid = 1;
        // }
        else{
            rid = 1;
        }
    }
    else if (discretization_level == 1){
        if (coord[1] >= 90.0 && coord[0] >= 90.0){
            rid = 0; //goal region
        }
        else{
            rid = 1;
        }
    }
    return rid;
}

double ompl::control::BeliefDecomposition::getRegionVolume(int id)
{
    return 1.0;
    // if (id == 0){
    //     return 100.0;
    // }
    // else if (id == 4){
    //     return 100.0;
    // }
    // // else if (id == 2){
    // //     return 625.0;
    // // }
    // else{
    //     return 500.0;
    // }
}

void ompl::control::BeliefDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    if (discretization_level_ == 2){
        if (rid == 0){
            neighbors.push_back(1);
            // neighbors.push_back(2);
            // neighbors.push_back(3);
            // neighbors.push_back(4);
        }
        else if (rid == 1){
            neighbors.push_back(0);
            neighbors.push_back(4);
            // neighbors.push_back(4);
        }
        else if (rid == 2){
            // neighbors.push_back(0);
            neighbors.push_back(1);
            neighbors.push_back(3);
            neighbors.push_back(4);
        }
        else if (rid == 3){
            // neighbors.push_back(1);
            // neighbors.push_back(2);
            // neighbors.push_back(0);
            neighbors.push_back(2);
            neighbors.push_back(5);
            // neighbors.push_back(4);
        }
        else if (rid == 4){
            neighbors.push_back(1);
            neighbors.push_back(2);
            neighbors.push_back(5);
        }
        else if (rid == 5){
            neighbors.push_back(4);
            neighbors.push_back(3);
        }
    }
    else if (discretization_level_ == 1){
        if (rid == 0){
            neighbors.push_back(1);
        }
        else if (rid == 1){
            neighbors.push_back(0);
        }
    }
}

int ompl::control::BeliefDecomposition::locateRegion(const ompl::base::State *s) const
{
    std::vector<double> coord(6);
    project(s, coord);
    return coordToRegion(coord, discretization_level_);
}

void ompl::control::BeliefDecomposition::sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const{

    while (true){
        coord[0] = rng.uniformReal(0.0, 100.0);
        coord[1] = rng.uniformReal(-100.0, 100.0);
        if (coordToRegion(coord, discretization_level_) == rid){
            break;
        }
    }
}