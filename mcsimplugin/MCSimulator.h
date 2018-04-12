#ifndef MC_SIMULATOR_H
#define MC_SIMULATOR_H

#include <unordered_set>
#include <armadillo>
#include <openrave/plugin.h>
#include <cassert>

#ifdef USEDEBUG
#define Debug(x) std::cout << x
#else
#define Debug(x) 
#endif

using namespace OpenRAVE;
using namespace arma;

#define MPI 3.14159265358979323846  /* pi */

typedef std::vector<double> config;

#ifdef USEDEBUG
#define Debug(x) std::cout << x
#else
#define Debug(x) 
#endif 

//Wraps angle to 0,2pi. TODO: Make more efficient
double angleWrap(double angle){
    while(angle < 0){
        angle += 2 * MPI;
    }
    while(angle > (2 * MPI)){
        angle -= 2 * MPI;
    }

    return angle;
}

//3 x 1 Arma matrix to double vector
bool arma2Vector(const arma::Mat<double>& mconfig, std::vector<double> config){
    config.resize(3);
    config[0] = (mconfig(0,0));
    config[1] = (mconfig(1,0));
    config[2] = (mconfig(2,0));
}

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
    arma::Mat<double> mcparticles;

    //Covariance and mean of state
    arma::Mat<double> covariance;
    arma::Mat<double> mu;

    //Initial mean and covariance
    arma::Mat<double> initialcovariance;
    arma::Mat<double> initialmu;

    //Trajectory and odometry
    arma::Mat<double> trajectory;
    arma::Mat<double> odometry;
    
    //pathlength
    int pathlength;

    //Pter to openrave environment
    EnvironmentBasePtr envptr;
    //Ptr to robot
    RobotBasePtr robotptr;
    //env mutex
    EnvironmentMutex& m;

public:
MCSimulator(EnvironmentBasePtr envptr):m(envptr->GetMutex()){
        alphas = ones<arma::Mat<double> >(1,4);
        std::cout << "Inside MCSimulator constructor" << std::endl;

        //Get Openrave stuff
        this->envptr = envptr;
        std::vector<RobotBasePtr> robots;

        m.lock();
        envptr->GetRobots(robots);
        m.unlock();
        
        assert(!robots.empty());
        robotptr = robots[0];
    }

    void setTrajectory(const arma::Mat<double>& trajectory){
        this->trajectory = trajectory;
        std::cout << "C++ Got Trajectory:" << std::endl;
        std::cout << this->trajectory << std::endl;

        //Set initial mean. to first column
        initialmu = trajectory.col(0);
        //Initial mean
        std::cout << "C++ got initial mean:" << std::endl;
        std::cout << initialmu << std::endl;
    }
    
    void setOdometry(const arma::Mat<double>& odometry){
        this->odometry = odometry;
        std::cout << "C++ Got Odometry:" << std::endl;
        std::cout << this->odometry << std::endl;
    }

    void setInitialCovariance(const arma::Mat<double>& cov){
        this->initialcovariance = cov;
        std::cout << "C++ Got initial covariance:" << std::endl;
        std::cout << cov << std::endl;
    }

    void setNumParticles(int num){
        this->numParticles = num;
        std::cout << "C++ got numparticles: " << std::endl;
        std::cout << this->numParticles << std::endl;
    }

    void setPathLength(int length){
        this->pathlength = length;
    }

    int getPathLength(){
        return this->pathlength;
    }

    void setNumLandmarks(int num){
        this->numLandmarks = num;
        std::cout << "C++ got numlandmarks: " << std::endl;
        std::cout << this->numLandmarks << std::endl;
    }

    void setLandmarks(const arma::Mat<double>& landmarks){
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

    //Overloaded. Takes in 3 x 1 arma matrix
    bool checkCollision(arma::Mat<double>& mconfig){
        std::vector<double> config;
        arma2Vector(mconfig, config);

        return checkCollision(config);
    }

    //Returns true if config is in collision
    bool checkCollision(const config& dcurrConfig){
        bool collisionExists = false;

        m.lock();
        //robotptr->SetActiveDOFValues(const std::vector< dReal >& values)
        Debug( "Setting ActiveDOFValues" << endl);
        robotptr->SetActiveDOFValues(dcurrConfig);
        Debug( endl << "Finished setting ActiveDOFValues" << endl);
            
        if(robotptr->CheckSelfCollision() || envptr->CheckCollision(robotptr)){
            collisionExists = true;
        }
        m.unlock();

        return collisionExists;
    }

    void initParticles(){
        //Initialize particles
        //Use mean and covariance
        mcparticles = mvnrnd(mu, covariance, numParticles);

        std::cout << "C++ made initial particles: " << std::endl;
        std::cout << mcparticles << std::endl;
    }
    
    // Run the MC simulation to get the probability of collision
    void runSimulation(){
        mu = initialmu;
        covariance = initialcovariance;
        //Initialize particles
        initParticles();
    }


};


#endif /* MC_SIMULATOR_H */
