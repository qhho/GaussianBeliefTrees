#include "STLBeliefDecomposition.h"

double square(double value){
    return value*value;
}

ompl::control::BeliefDecomposition::BeliefDecomposition(int dim, const base::RealVectorBounds &b, int discretization_level, const double p_safe) : Decomposition(dim, b){

    discretization_level_ = discretization_level;
    setNumRegions(discretization_level +1);

    // si_ = si;
	p_threshold_ = p_safe;

	// if (scene_id == "2d_underwater_2regions") {
    Scene scene = Scene();
    n_regions_ = scene.n_obstacles_;
    A_list_.resize(n_regions_); A_list_ = scene.A_list_;
    B_list_.resize(n_regions_); B_list_ = scene.B_list_;
	// }

	erf_inv_result_ = boost::math::erf_inv(1 - 2 * p_threshold_/ n_regions_);  ///4.0
}

ompl::control::BeliefDecomposition::BeliefDecomposition(int dim, const base::RealVectorBounds &b) : Decomposition(dim, b){

    discretization_level_ = 1;
    setNumRegions(1);
}

int coordToRegion(std::vector<double> coord, int discretization_level){
    int rid = -1;

    if (discretization_level == 2){
        if (coord[1] >= 90.0 && coord[0] >= 90.0){ //goal region 1
            rid = 2; 
        }
        else if (coord[0] >= 70.0 && coord[1] < 20.0 && coord[1] >= 0.0){ //(coord[1] >= 80.0 && coord[0] >= 50.0 && coord[0] < 90.0){ 
            rid = 1;
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

double ompl::control::BeliefDecomposition::getRegionVolume(int id)
{
    return 1.0;
}

void ompl::control::BeliefDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
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

double inRegion(double x, double y, double radius_, int region)
        {
        if (region == 1){
            double dx = x - 10.0;
            double dy = y - 90.0;
            double radius = 2.477*sqrt(radius_);

            if (sqrt(dx*dx + dy*dy) + radius  <= 10.0){
                // std::cout << st->as<ob::RealVectorStateSpace::StateType>()->values[0] << " " << st->as<ob::RealVectorStateSpace::StateType>()->values[1] << " " << st->as<ob::RealVectorStateSpace::StateType>()->values[3] + st->as<ob::RealVectorStateSpace::StateType>()->values[5] << std::endl;
                return true;
            }
            return false;
        }
        else{
            double dx = x - 90.0;
            double dy = y - 90.0;
            double radius = 2.477*sqrt(radius_);

            if (sqrt(dx*dx + dy*dy) + radius  <= 10.0){
                // std::cout << st->as<ob::RealVectorStateSpace::StateType>()->values[0] << " " << st->as<ob::RealVectorStateSpace::StateType>()->values[1] << " " << st->as<ob::RealVectorStateSpace::StateType>()->values[3] + st->as<ob::RealVectorStateSpace::StateType>()->values[5] << std::endl;
                return true;
            }
            return false;
        }
        // perform any operations and return a double indicating the distance to the goal
}


int ompl::control::BeliefDecomposition::locateRegion(const ompl::base::State *s) const
{
    std::vector<double> coord(4); //x, y, covx, covy
    project(s, coord);

    double x = coord[0];
    double y = coord[1];
    if (x > 100.0 || x < 0.0 || y < -50.0 || y > 100.0){
        return false;
    }

    //=========================================================================
    // Extract the component of the state and cast it to what we expect
    //=========================================================================
    double z;
    Eigen::MatrixXf PX(3, 3); PX.setZero();

    z = 4.0;
    PX(0,0) = coord[2];
    PX(1,1) = coord[3];
    PX(2,2) = 0.00000001;

    // x = 5.0;
    // y = 95.0;
    //=========================================================================
    // Probabilistic region checker
    //=========================================================================
    // for each obstacle, if in region, 

    
    if(inRegion(x, y, coord[2], 1)){
		// std::cout << x << " " << y << std::endl;
		return 1;
	}
    if(inRegion(x, y, coord[2], 2)){
		// std::cout << x << " " << y << std::endl;
		return 2;
	}

    // for (int o = 1; o < n_regions_; o++) {
    //     if (HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x, y, z, PX)) {
    //         return o+1;
    //     }
    // }


    if (coord[1] < -40.0){
            return 3;
    }
    return 0;
    // valid = true;

    // return valid;

    // return coordToRegion(coord, discretization_level_);
}

bool ompl::control::BeliefDecomposition::HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    bool valid = true;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * erf_inv_result_;

		// if(x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2) >= (B(i, 0) + b_bar)) {
        if(x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2) >= (B(i, 0) + b_bar)) {
			valid = false;
            // std::cout << valid << std::endl;
            // exit(0);
            return valid;
		}
	}
    // std::cout << valid << std::endl;
    // exit(0);
	return valid;
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