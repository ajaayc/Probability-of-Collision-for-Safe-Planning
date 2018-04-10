#ifndef MC_SIMULATOR_H
#define MC_SIMULATOR_H

#include <unordered_set>
#include <armadillo>
#include <openrave/plugin.h>

#ifdef USEDEBUG
#define Debug(x) std::cout << x
#else
#define Debug(x) 
#endif

using namespace OpenRAVE;
using namespace arma;

class MCSimulator{
    //Squared odometry noise values
    arma::Mat<double> alphas;
    //Variance of sensor noise
    double Q;

    //Landmark locations
    arma::Mat<double> landmarks;
    int numLandmarks;

    //Particles
    int numParticles;
public:
    MCSimulator(){
        alphas = ones<arma::Mat<double> >(1,4);
        std::cout << "Inside MCSimulator constructor" << std::endl;
    }

    void setNumParticles(int num){
        this->numParticles = num;
        std::cout << "C++ got numparticles: " << std::endl;
        std::cout << this->numParticles << std::endl;
    }

    void setNumLandmarks(int num){
        this->numLandmarks = num;
        std::cout << "C++ got numlandmarks: " << std::endl;
        std::cout << this->numLandmarks << std::endl;
    }

    void setLandmarks(arma::Mat<double>& landmarks){
        this->landmarks = landmarks;
        std::cout << "C++ got landmarks: " << std::endl;
        std::cout << this->landmarks << std::endl;
    }

    int getNumLandmarks(){
        return this->numLandmarks;
    }
    
    void setAlphas(const std::vector<double>& alphas){
        std::cout << "Setting alphas in C++: " << std::endl;
        for(unsigned it = 0; it < alphas.size(); ++it){
            this->alphas(0,it) = alphas[it];
        }
        std::cout << this->alphas << std::endl;
    }

    void setQ(double Q){
        this->Q = Q;
        std::cout << "C++ got Q: " << this->Q << std::endl;
    }
};


#endif /* MC_SIMULATOR_H */
