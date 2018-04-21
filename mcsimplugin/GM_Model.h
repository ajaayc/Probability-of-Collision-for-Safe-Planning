#ifndef GM_MODEL_H
#define GM_MODEL_H

#include <vector>
#include <armadillo>
#include <chrono>
#include <random>

using namespace arma;

//Represents a Gaussian Mixture
class GM_Model{
    int numGaussians;
    //Each mean is a 3 x 1
    std::vector<arma::Mat<double>> means;
    //Each covariance is a 3 x 3
    std::vector<arma::Mat<double>> covariances;

    //Random generator
    std::default_random_engine generator;
    //Weights of each item in GMM. Same as weighted_dist.probabilities
    std::vector<double> weights;
    // in this loaded die, the 6 is three times more likely:
    std::discrete_distribution<int> weighted_dist;

public:
    //Cstor
    GM_Model(){
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);
    }
    
    void initModel(int numGaussians,arma::Mat<double>& initialMean, arma::Mat<double> initialCovariance){
        this->numGaussians = numGaussians;
        this->means.resize(this->numGaussians);
        this->covariances.resize(this->numGaussians);
        
        //Initialize GMM
        for(unsigned i = 0; i < this->numGaussians; ++i){
            means[i] = initialMean;
            covariances[i] = initialCovariance;
        }

        //Initialize with equal weights
        std::vector<double> weightsn(this->numGaussians, 1.0/this->numGaussians);
        this->updateWeights(weightsn);
    }

    //Sample from GM_Model
    void sampleVal(){
        int chosen_gaussian = this->weighted_dist(generator);
        return;
    }


    //Update distribution with new weights
    void updateWeights(std::vector<double>& weights){
        this->weights = weights;
        this->weighted_dist = std::discrete_distribution<int>(weights.begin(),weights.end());
    }
        
};

#endif /* GM_MODEL_H */
