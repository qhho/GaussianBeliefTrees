#include "STLBeliefDecomposition3D.h"

double square(double value){
    return value*value;
}

ompl::control::BeliefDecomposition3D::BeliefDecomposition3D(int dim, const base::RealVectorBounds &b, int discretization_level, const double p_safe) : Decomposition(dim, b){

    discretization_level_ = discretization_level;
    setNumRegions(5);

    // si_ = si;
	p_threshold_ = p_safe;

	// OMPL_INFORM("scene is %s", scene_id.c_str());
	// if (scene_id == "2d_underwater_2regions") {
    Scene3D scene = Scene3D();
    n_regions_ = scene.n_obstacles_;
    A_list_.resize(n_regions_); A_list_ = scene.A_list_;
    B_list_.resize(n_regions_); B_list_ = scene.B_list_;
	// }

	erf_inv_result_ = boost::math::erf_inv(1 - 2 * p_threshold_/n_regions_);
}

ompl::control::BeliefDecomposition3D::BeliefDecomposition3D(int dim, const base::RealVectorBounds &b) : Decomposition(dim, b){

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

double ompl::control::BeliefDecomposition3D::getRegionVolume(int id)
{
    return 1.0;
}

void ompl::control::BeliefDecomposition3D::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    if (discretization_level_ == 2){
        if (rid == 0){
            neighbors.push_back(1);
            neighbors.push_back(2);
            neighbors.push_back(3);
            neighbors.push_back(4);
            neighbors.push_back(5);
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
        else if (rid == 4){
            neighbors.push_back(0);
        }        
        else if (rid == 5){
            neighbors.push_back(0);
        }
    }
}

int ompl::control::BeliefDecomposition3D::locateRegion(const ompl::base::State *s) const
{
    std::vector<double> coord(6); //x, y, z, covx, covy, covz
    project(s, coord);

    double x = coord[0];
    double y = coord[1];
    double z = coord[2];
    // if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0 || z < 0.0 || z > 20.0){
    //     return false;
    // }

    //=========================================================================
    // Extract the component of the state and cast it to what we expect
    //=========================================================================
    // double z;
    Eigen::MatrixXf PX(3, 3); PX.setZero();

    // z = 4.0;
    PX(0,0) = coord[3];
    PX(1,1) = coord[4];
    PX(2,2) = coord[5];

    //=========================================================================
    // Probabilistic region checker
    //=========================================================================
    // for each obstacle, if in region, 

    // for (int i =0 ; i < 6; ++i){
    //     std::cout << coord[i] << " ";
    // }
    // std::cout << std::endl;
    
    // if (coord[2] > 10.0){
    //     return 5;
    // }
    // else{
    //     std::cout << coord[2] << std::endl;
    // }

    for (int o = 0; o < n_regions_; o++) {
        if (HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x, y, z, PX)) {
            // std::cout << o+1 << " " << x << " " << y << " " << z << " " << coord[3] << " " << coord[4] << " " << coord[5] << std::endl;
            return o+1;
        }
    }

    
    return 0;
    // valid = true;

    // return valid;

    // return coordToRegion(coord, discretization_level_);
}

bool ompl::control::BeliefDecomposition3D::HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    bool valid = true;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * erf_inv_result_;

		if(B(i, 0) - (x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2)) >= b_bar) {
			// valid = false;
            // return valid;
            valid = true;
		}
        else{
            valid = false;
            return false;
        }
	}
	return valid;
}

void ompl::control::BeliefDecomposition3D::sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const{

    while (true){
        coord[0] = rng.uniformReal(0.0, 100.0);
        coord[1] = rng.uniformReal(-100.0, 100.0);
        if (coordToRegion(coord, discretization_level_) == rid){
            break;
        }
    }
}