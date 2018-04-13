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

double squareNum(double num){
    return num * num;
}

double sampleNormal(double mean,double variance){
    return mean + randn() * sqrt(variance);
}

//Wraps angle to 0,2pi. TODO: Make more efficient
inline double angleWrap(double angle){
    while(angle < 0){
        angle += 2 * MPI;
    }
    while(angle > (2 * MPI)){
        angle -= 2 * MPI;
    }

    return angle;
}

inline double roundAngle(double angle){
    return angleWrap(angle);
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
    arma::Mat<double> cov;
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

    void setInitialCovariance(const arma::Mat<double>& covin){
        this->initialcovariance = covin;
        std::cout << "C++ Got initial covariance:" << std::endl;
        std::cout << covin << std::endl;
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
        mcparticles = mvnrnd(initialmu, initialcovariance, numParticles);

        std::cout << "C++ made initial particles: " << std::endl;
        std::cout << mcparticles << std::endl;
    }
    
    // Run the MC simulation to get the probability of collision
    void runSimulation(){
        mu = initialmu;
        cov = initialcovariance;
        //Initialize particles
        initParticles();
        EKF_GaussProp();
    }

    //Applies sensor model based on given landmark id
    double observation(arma::Mat<double>& state,int landmarkid){
        arma::Mat<double> currlmk = this->landmarks.col(landmarkid);
        
        //Get distance
        arma::Mat<double> s = state(span(0,1), span(0,0)); //state[0:2]
        //TODO: check s's dimension is ok

        
        arma::Mat<double> diff = s - currlmk;

        //2 norm
        double distance = norm(diff,2);
        return distance;
    }

    double sampleObservation(arma::Mat<double>& state,int landmarkid){
        double distance = this->observation(state,landmarkid);
        //Add Gaussian noise to distance
        return (distance + sampleNormal(0,this->Q));
    }
    
    //Generates pose following a noisy control input
    arma::Mat<double> sampleOdometry(arma::Mat<double>& state,arma::Mat<double>& motioncmd){
        double drot1 = motioncmd(0,0);
        double dtrans = motioncmd(1,0);
        double drot2 = motioncmd(2,0);

        double alphas1 = this->alphas(0,0);
        double alphas2 = this->alphas(1,0);
        double alphas3 = this->alphas(2,0);
        double alphas4 = this->alphas(3,0);

        arma::Mat<double> noisymotion = zeros<arma::Mat<double>>(3,1);
        
        noisymotion(0,0) = sampleNormal(drot1,alphas1 * squareNum(drot1)+alphas2*squareNum(dtrans));
        noisymotion(1,0) = sampleNormal(dtrans,alphas3 * squareNum(dtrans)+alphas4*(squareNum(drot1)+squareNum(drot2)));
        noisymotion(2,0) = sampleNormal(drot2,alphas1 * squareNum(drot2)+alphas2*squareNum(dtrans));

        arma::Mat<double> newstate = this->prediction(state, noisymotion);

        return newstate;
    }

    //Applies odometry motion model
    arma::Mat<double> prediction(arma::Mat<double>& state,arma::Mat<double>& motioncmd){
        double drot1 = motioncmd(0,0);
        double dtrans = motioncmd(1,0);
        double drot2 = motioncmd(2,0);

        double x = state(0,0);
        double y = state(1,0);
        double theta = state(2,0);
        
        arma::Mat<double> newstate = zeros<arma::Mat<double>>(3,1);

        newstate(0,0) = x + dtrans * cos(theta + drot1);
        newstate(1,0) = y + dtrans * sin(theta + drot1);
        newstate(2,0) = theta + drot1 + drot2;

        newstate(2,0) = roundAngle(newstate(2,0));
        
        return newstate;
    }

    //Given two poses, compute the odometry command between them
    arma::Mat<double> inverseOdometry(arma::Mat<double>& p1,arma::Mat<double>& p2){
        double drot1 = atan2(p2(1,0) - p1(1,0), p2(0,0) - p1(0,0)) - p1(2,0);
        drot1 = roundAngle(drot1);

        double dtrans = sqrt(squareNum(p2(0,0) - p1(0,0)) + squareNum(p2(1,0) - p1(1,0)));

        double drot2 = p2(2,0) - p1(2,0) - drot1;
        drot2 = roundAngle(drot2);

        arma::Mat<double> inv = zeros<arma::Mat<double>>(3,1);
        inv(0,0) = drot1;
        inv(1,0) = dtrans;
        inv(2,0) = drot2;
        
        return inv;
    }

    //Jacobian of motion model with respect to control input
    //Same as V from Thrun book
    arma::Mat<double> generateV_EKF(arma::Mat<double>& prevMu,arma::Mat<double>& motioncmd){
        double drot1 = motioncmd(0,0);
        double dtrans = motioncmd(1,0);
        double drot2 = motioncmd(2,0);
        
        double prevTheta = prevMu(2,0);

        arma::Mat<double> V = eye<arma::Mat<double>>(3,3);
        V(2,0) = 1;
        V(0,0) = -dtrans *  sin(prevTheta + drot1);
        V(0,1) = cos(prevTheta + drot1);
        V(1,0) = dtrans * cos(prevTheta + drot1);
        V(1,1) = sin(prevTheta + drot1);

        return V;
    }

    arma::Mat<double> makeHRow(arma::Mat<double>& state,int landmarkid){
        double mx = this->landmarks(0,landmarkid);
        double my = this->landmarks(1,landmarkid);

        double x = state(0,0);
        double y = state(1,0);

        double diff0 = x - mx;
        double diff1 = y - my;

        double q = squareNum(diff0) + squareNum(diff1);
        
        double entry1 = -(mx - x)/sqrt(q);
        double entry2 = -(my - y)/sqrt(q);
        double entry3 = 0;

        arma::Mat<double> result = zeros<arma::Mat<double>>(3,1);
        result(0,0) = entry1;
        result(1,0) = entry2;
        result(2,0) = entry3;
        
        return result;
    }

    //Odometry noise
    arma::Mat<double> generateM_EKF(arma::Mat<double>& motioncmd){
        double drot1 = motioncmd(0,0);
        double dtrans = motioncmd(1,0);
        double drot2 = motioncmd(2,0);

        double alphas1 = this->alphas(0,0);
        double alphas2 = this->alphas(1,0);
        double alphas3 = this->alphas(2,0);
        double alphas4 = this->alphas(3,0);

        arma::Mat<double> M = zeros<arma::Mat<double>>(3,3);

        M(0,0) = alphas1 * squareNum(drot1) + alphas2 * squareNum(dtrans);
        M(1,1) = alphas3 * squareNum(dtrans) + alphas4 * squareNum(drot1) + alphas4 * squareNum(drot2);
        M(2,2) = alphas1 * squareNum(drot2) + alphas2 * squareNum(dtrans);

        //M = -M
        return M;
    }

    //Jacobian of motion model with respect to state.
    //Same as G in Thrun book
    arma::Mat<double> generateG_EKF(arma::Mat<double>& prevMu,arma::Mat<double>& motioncmd){
        double drot1 = motioncmd(0,0);
        double dtrans = motioncmd(1,0);
        double drot2 = motioncmd(2,0);

        double prevTheta = prevMu(2,0);

        arma::Mat<double> G = eye<arma::Mat<double>>(3,3);
        G(0,2) = -dtrans * sin(prevTheta + drot1);
        G(1,2) = dtrans * cos(prevTheta + drot1);

        return G;
    }

    //Compute the 3x3 control gain matrix L_t+1
    arma::Mat<double> generateL(arma::Mat<double>& nominalcurrstate,arma::Mat<double>& estimatedcurrstate,arma::Mat<double>& nominalgoalstate,arma::Mat<double>& nominalcontrol){
        //Get (estimate) of the state deviation
        arma::Mat<double> xhatt = estimatedcurrstate - nominalcurrstate;

        //Get odometry needed to move from estimated currstate to nominalgoalstate
        arma::Mat<double> urequired = this->inverseOdometry(estimatedcurrstate,nominalgoalstate);

        //Get difference between u and u*
        arma::Mat<double> ubar = urequired - nominalcontrol;
        //print 'ubar: ', ubar;

        //Find the 3x3 linear transformation L needed to move from xhatt to ubar
        arma::Mat<double> L = eye<arma::Mat<double>>(3,3);

        //TODO: Think about this divide by 0. This would occur if xhatt
        //has 0's. i.e. deviation between curr state and nominal state is 0
        L(0,0) = ubar(0,0) / (xhatt(0,0) != 0 ? xhatt(0,0) : 0.1);
        L(1,1) = ubar(1,0) / (xhatt(1,0) != 0 ? xhatt(1,0) : 0.1);
        L(2,2) = ubar(2,0) / (xhatt(2,0) != 0 ? xhatt(2,0) : 0.1);
        
        return L;
    }

    //------------------------------------------------------------
    //Stuff below this is for the actual paper
    //------------------------------------------------------------
    //trajectory is list of states for the motion plan
    //controlinputs is list of odometry commands to transition between states
    //len(controls) = len(trajectory) - 1

    void EKF_GaussProp(){
        arma::Mat<double>& trajectoryi = this->trajectory;
        arma::Mat<double>& controlinputs = this->odometry;
        
        //Initialize mean and covariance
        this->mu = this->initialmu;
        this->cov = this->initialcovariance;

        //Initialize realpath
        arma::Mat<double> realpath = zeros<arma::Mat<double>>(3,this->pathlength);

        arma::Mat<double> realstate = mu;
        //Store the real state (we don't know this in practice)
        realpath.col(0) = realstate;

        //simulate trajectory. Loop through all control inputs
        for(int i = 0; i < this->pathlength - 1; ++i){
            arma::Mat<double> control = controlinputs.col(i);
            //Get motion command
            arma::Mat<double> motionCommand = controlinputs.col(i);

            arma::Mat<double> M = this->generateM_EKF(motionCommand);
            double Q = this->Q;

            //Get control gain to move to next state
            arma::Mat<double> nominalstate = trajectoryi.col(i);
            arma::Mat<double> estimatedstate = mu;
            arma::Mat<double> nominalgoal = trajectoryi.col(i+1);
            arma::Mat<double> nominalcontrol = controlinputs.col(i);

            arma::Mat<double> gain = this->generateL(nominalstate,estimatedstate,nominalgoal,nominalcontrol);
            
            //Multiply gain by deviation in state to get deviation to add to u*
            arma::Mat<double> statedeviation = estimatedstate - nominalstate;
            //statedeviation = np.asmatrix(statedeviation).transpose()

            arma::Mat<double> controldeviation = gain * statedeviation;
            //controldeviation = controldeviation.transpose()

            //Add control deviation to nominal control
            arma::Mat<double> appliedcontrol = nominalcontrol + controldeviation;
            //appliedcontrol = appliedcontrol[0]

            //------------------------------------------------------------
            //EKF Predict. Predict where we'll go based on applied control
            arma::Mat<double> predMu;
            arma::Mat<double> predSigma;
            this->EKFpredict(mu,cov,appliedcontrol,M,Q,predMu,predSigma);
            //------------------------------------------------------------

            //Now move (with noise)
            //Add noise to odometry to go to another state
            arma::Mat<double> nextstate = this->sampleOdometry(realstate,appliedcontrol);
            realstate = nextstate;
            realpath.col(i+1) = realstate;
            //print 'realstate: ', realstate
            std::cout << "realstate: " << realstate << std::endl;
            
            arma::Mat<double> realobservations = zeros<arma::Mat<double>>(1,this->numLandmarks);
            
            //Get sensor measurements from the real state. Loop
            //through all landmarks
            for(int currlid = 0; currlid < this->numLandmarks; ++currlid){
                double z = this->sampleObservation(realstate,currlid);
                realobservations(0,currlid) = z;
            }
            
            //------------------------------------------------------------
            //EKF Update of estimated state and covariance based on the measurements
            arma::Mat<double> newmu;
            arma::Mat<double> newsigma;
            this->EKFupdate(predMu,predSigma,realobservations,Q,newmu,newsigma);
            this->mu = newmu;
            this->cov = newsigma;

            //print 'nominalstate: ', trajectoryi[i+1]
            //print 'estimatestate: ', mu
            //print 'estimatecov: ', cov
            
            std::cout << 'nominalstate: ' << trajectoryi.col(i+1) << std::endl;
            std::cout << 'estimatestate: ' <<  this->mu << std::endl;
            std::cout << 'estimatecov: ' << this->cov << std::endl;

            //
            //------------------------------------------------------------
        }
        
        //return realpath;
    }

        //Modifies predMu and predSigma. Others are untouched

        void EKFpredict(arma::Mat<double>& mu, arma::Mat<double>& Sigma,arma::Mat<double>& u, arma::Mat<double>& M, double Q, arma::Mat<double>& predMu, arma::Mat<double>& predSigma){
        //Get G matrix and V matrix
        arma::Mat<double> G = this->generateG_EKF(mu,u);
        arma::Mat<double> V = this->generateV_EKF(mu,u);

        //noise in odometry
        arma::Mat<double> R = V * M * V.t();

        //Return values
        predMu = this->prediction(mu,u);
        predSigma = G * Sigma * G.t() + R;

        return;
    }

        void EKFupdate(arma::Mat<double>& predMu,arma::Mat<double>& predSigma,arma::Mat<double>& measurements,double Q, arma::Mat<double>& newmu,arma::Mat<double>& newsigma){
        //Loop through all measurements
        for(int lid = 0; lid < measurements.n_cols; ++lid){
        double measurement = measurements(0,lid);

        //listd is landmark id. measurement is the distance
        //to the landmark recorded by sensor
        double landmark_x = this->landmarks(0,lid);
        double landmark_y = this->landmarks(1,lid);

        // Lines 10-13 of EKF Algorithm
        arma::Mat<double> H = this->makeHRow(predMu,lid);

        //Innovation / residual covariance

        arma::Mat<double> S = H * predSigma * H.t() + Q;

        // Kalman gain
        arma::Mat<double> K = predSigma * H.t() * S.i();

        //z and zhat
        double z = measurement;
        double zhat = this->observation(predMu,lid);
            
        // Correction
        double temp = z - zhat;
        predMu = predMu.t() + K * (temp);
        predSigma = (eye<arma::Mat<double>>(3,3) - K * H) * predSigma;
    }
            

        newmu = predMu;
        newsigma = predSigma;
    }

};


#endif /* MC_SIMULATOR_H */