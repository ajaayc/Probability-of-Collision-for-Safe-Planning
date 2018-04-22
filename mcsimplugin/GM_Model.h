#ifndef GM_MODEL_H
#define GM_MODEL_H

#include <vector>
#include <armadillo>
#include <chrono>
#include <random>

using namespace arma;

#define USEGMMDebug

#ifdef USEGMMDebug
#define GMMDebug(x) std::cout << x
#else
#define GMMDebug(x) 
#endif


void printVectorGM(std::vector<double>& inp){
    for(auto it=inp.begin(); it != inp.end(); ++it){
        GMMDebug(*it << " ";);
    }
    GMMDebug(std::endl;);
}
void printVectorGM(std::vector<int>& inp){
    for(auto it=inp.begin(); it != inp.end(); ++it){
        GMMDebug(*it << " ";);
    }
    GMMDebug(std::endl;);
}

//Represents a Gaussian Mixture
class GM_Model{
public:
    int numGaussians;
    //Each mean is a 3 x 1
    std::vector<arma::Mat<double>> means;
    //Each covariance is a 3 x 3
    std::vector<arma::Mat<double>> covariances;

private:
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
        
        GMMDebug("Initialized GMM" << std::endl;);
        GMMDebug("NumGaussians: " << this->numGaussians << std::endl;);

        //Initialize GMM
        for(unsigned i = 0; i < this->numGaussians; ++i){
            means[i] = initialMean;
            covariances[i] = initialCovariance;
            GMMDebug("Mean [" << i << "]:\n " << means[i] << std::endl;);
            GMMDebug("Covariance [" << i << "]:\n " << covariances[i] << std::endl;);
        }

        //Initialize with equal weights
        GMMDebug("Initializing Weights\n");
        std::vector<double> weightsn(this->numGaussians, 1.0/this->numGaussians);
        this->updateWeights(weightsn);
    }

    //Sample N points from GM_Model
    //Returns a vector of size numGaussians (modifies points). Each vector component
    //is a 3 x Num arma matrix, where Num is the number of points generated
    //from that Gaussian component. N is total number of points to sample
    void sampleNPoints(int N,std::vector<arma::Mat<double> >& points){
        std::vector<int> counts(this->numGaussians,0);

        GMMDebug("Sampling N points from GMM\n";);
        GMMDebug("Number of GMM components: " << this->numGaussians << "\n";);
        //Get total number of points to generate from each distribution
        for(unsigned i = 0; i < N; ++i){
            int chosen = this->weighted_dist(generator);
            //GMMDebug(chosen << " ";);
            ++counts[chosen];
        }

        GMMDebug("Counts Vector\n";);
        printVectorGM(counts);

        //Populate points
        points.resize(this->numGaussians);
        GMMDebug("Generating Points:\n";);
        for(unsigned i = 0; i < counts.size(); ++i){
            //Generate points from the distribution, with mean and covariance
            arma::Mat<double>& curr = points[i];

            //mvnrnd(X,M,C,N);
            //mvnrnd(X,M,C,N);
            mvnrnd(curr,means[i],covariances[i],counts[i]);
            GMMDebug("Points " << i << " Dimension:\n";);
            //GMMDebug(points[i] << std::endl;);
            GMMDebug(points[i].n_rows << "x" << points[i].n_cols << std::endl;);
            GMMDebug("Finished printing dimension\n";);
        }
        

        return;
    }

    //Update distribution with new weights
    void updateWeights(std::vector<double>& weights){
        this->weights = weights;
        this->weighted_dist = std::discrete_distribution<int>(weights.begin(),weights.end());
        GMMDebug("Updated GMM Weights:\n");
        printVectorGM(this->weights);
    }
        
};

#endif /* GM_MODEL_H */
