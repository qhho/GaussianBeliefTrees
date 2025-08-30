#ifndef RN_BELIEF_SPACE_H
#define RN_BELIEF_SPACE_H

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <ompl/util/RandomNumbers.h>

namespace ob = ompl::base;

/**
 * @class RNBeliefSpace
 * @brief A state space for N-dimensional belief states with mean and covariance
 * 
 * This state space represents N-dimensional belief states with position mean vectors
 * and covariance matrices.
 */
class RNBeliefSpace : public ob::RealVectorStateSpace {
public:
    class StateType : public ob::RealVectorStateSpace::StateType {
    public:
        StateType(const Eigen::MatrixXd &sigma_init);

        // Mean access methods for any dimension
        double getComponent(unsigned int index) const;
        void setComponent(unsigned int index, double value);

        // Convenience methods for common dimensions
        double getX() const;
        double getY() const;
        double getZ() const;
        
        void setX(double x);
        void setY(double y);
        void setZ(double z);

        // Get mean vector
        Eigen::VectorXd getMatrixData() const;

        // Get dimension of state
        unsigned int getDimension() const;

        // Covariance access methods
        const Eigen::MatrixXd& getSigma() const;
        const Eigen::MatrixXd& getLambda() const;
        const Eigen::MatrixXd& getCovariance() const;
        
        void setSigma(const Eigen::MatrixXd &sigma);
        void setSigma(double val);
        void setLambda(const Eigen::MatrixXd &lambda);

        void setSigmaX(double sigma);
        void setSigmaY(double sigma);
        void SetSigmaRandom(double max);

        // Check if this state is reached by another state
        bool isReached(ob::State *state, bool relaxedConstraint = false) const;

        const Eigen::Vector2d getXY(void) const
        {
            const Eigen::Vector2d stateVec(getX(), getY());
            return stateVec;
        }

        void setCost(double cost){
            cost_ = cost;
        }

        double getCost(void) const{ 
            return cost_;
        }

        // Static parameters for distance and reachability
        static double meanNormWeight_;
        static double covNormWeight_;
        static double reachDist_;

        ompl::RNG rng_;

    private:
        Eigen::MatrixXd sigma_;  // Covariance matrix
        Eigen::MatrixXd lambda_; // Information matrix
        double cost_;
    };

    /**
     * @brief Constructor
     * @param dim Dimension of the state space
     * @param sigma_init Initial covariance matrix
     */
    RNBeliefSpace(unsigned int dim, bool wasserstein, const Eigen::MatrixXd &sigma_init);

    RNBeliefSpace(unsigned int dim, const Eigen::MatrixXd &sigma_init);

    /**
     * @brief Set parameters for distance calculation and reachability
     * @param meanNormWeight Weight for the mean difference in distance computation
     * @param covNormWeight Weight for the covariance difference in distance computation
     * @param reachDist Distance threshold for determining reachability
     */
    void setDistanceParams(double meanNormWeight, double covNormWeight, double reachDist);

    // State space operations
    ob::State* allocState() const override;
    void freeState(ob::State *state) const override;
    void copyState(ob::State *destination, const ob::State *source) const override;
    double distance(const ob::State *state1, const ob::State *state2) const override;

    // Utility methods
    void printBeliefState(const ob::State *state);

private:
    unsigned int dimension_;            // Dimension of the state space
    Eigen::MatrixXd sigma_init_;        // Initial covariance for new states
    bool wasserstein_;                    // Use Euclidean distance for state space
};

#endif // RN_BELIEF_SPACE_H