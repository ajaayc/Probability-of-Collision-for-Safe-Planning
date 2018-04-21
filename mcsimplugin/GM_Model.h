#ifndef GM_MODEL_H
#define GM_MODEL_H

#include <vector>
#include <armadillo>

using namespace arma;

//Represents a Gaussian Mixture
class GM_Model{
    int numGaussians;
    //Each mean is a 3 x 1
    std::vector<arma::Mat<double>> means;
    //Each covariance is a 3 x 3
    std::vector<arma::Mat<double>> covariances;

public:
    //Cstor
    GM_Model(){}
    
    void initModel(int numGaussians,arma::Mat<double>& initialMean, arma::Mat<double> initialCovariance){
        this->numGaussians = numGaussians;
        this->means.resize(this->numGaussians);
        this->covariances.resize(this->numGaussians);
        
        //Initialize GMM
        for(unsigned i = 0; i < this->numGaussians; ++i){
            means[i] = initialMean;
            covariances[i] = initialCovariance;
        }
    }

    //Sample from GM_Model

        
};

#endif /* GM_MODEL_H */
