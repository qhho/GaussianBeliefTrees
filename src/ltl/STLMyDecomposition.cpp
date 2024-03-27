#include "STLMyDecomposition.h"

double square(double value){
    return value*value;
}

ompl::control::MyDecomposition::MyDecomposition(int dim, const base::RealVectorBounds &b, int discretization_level) : Decomposition(dim, b){

    discretization_level_ = discretization_level;
    setNumRegions(discretization_level +2);
}

ompl::control::MyDecomposition::MyDecomposition(int dim, const base::RealVectorBounds &b) : Decomposition(dim, b){

    discretization_level_ = 1;
    setNumRegions(1);
}

int coordToRegion(std::vector<double> coord, int discretization_level){
    int rid = -1;

    if (discretization_level == 2){
        if (coord[0] >= 80.0 && coord[1] >= 90.0){ //goal region 1
            rid = 2; 
        }
        else if (coord[0] <= 20.0 && coord[1] >= 90.0){ //(coord[1] >= 80.0 && coord[0] >= 50.0 && coord[0] < 90.0){ 
            rid = 1;
        }
        else if (coord[1] < -45.0){
            rid = 3;
        }
        else{
            rid = 0;
        }
    }
    else if (discretization_level == 1){
        if (coord[1] >= 90.0 && coord[0] >= 90.0){ //goal region 1
            rid = 1; 
        }
        else{
            rid = 0;
        }
    }
    return rid;
}

double ompl::control::MyDecomposition::getRegionVolume(int id)
{
    return 1.0;
}

void ompl::control::MyDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    if (discretization_level_ == 2){
        if (rid == 0){
            neighbors.push_back(1);
            neighbors.push_back(2);
            neighbors.push_back(3);
        }
        else if (rid == 1){
            neighbors.push_back(0);
        }
        else if (rid == 2){
            neighbors.push_back(0);
        }
        else if (rid == 3){
            neighbors.push_back(0);
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

int ompl::control::MyDecomposition::locateRegion(const ompl::base::State *s) const
{
    std::vector<double> coord(3);
    project(s, coord);
    if (coord[2] > 4.){
        // std::cout << "large" << std::endl;
        return 0;
    }
    return coordToRegion(coord, discretization_level_);
}

void ompl::control::MyDecomposition::sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const{

    while (true){
        coord[0] = rng.uniformReal(0.0, 100.0);
        coord[1] = rng.uniformReal(-100.0, 100.0);
        if (coordToRegion(coord, discretization_level_) == rid){
            break;
        }
    }
}