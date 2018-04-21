#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "MCSimulator.h"

using namespace OpenRAVE;

class MCModule : public ModuleBase
{
public:
    MCSimulator sim;
    
    MCModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv),sim(penv) {
        RegisterCommand("MyCommand",boost::bind(&MCModule::MyCommand,this,_1,_2),
                        "This is an example command");
        RegisterCommand("ArmaCommand",boost::bind(&MCModule::ArmaCommand,this,_1,_2),
                        "This is testing armadillo");
        RegisterCommand("setAlphas",boost::bind(&MCModule::setAlphas,this,_1,_2),
                        "This is to initialize alphas from python");
        RegisterCommand("setQ",boost::bind(&MCModule::setQ,this,_1,_2),
                        "This is to initialize variance of sensor noise");
        RegisterCommand("setNumLandmarks",boost::bind(&MCModule::setNumLandmarks,this,_1,_2),
                        "This is to initialize number of landmark locations");
        RegisterCommand("setLandmarks",boost::bind(&MCModule::setLandmarks,this,_1,_2),
                        "This is to initialize landmark locations");
        RegisterCommand("setNumParticles",boost::bind(&MCModule::setNumParticles,this,_1,_2),
                        "This is to initialize number of landmark locations");

        RegisterCommand("setInitialCovariance",boost::bind(&MCModule::setInitialCovariance,this,_1,_2),
                        "This is to initialize first state covariance uncertainty");

        RegisterCommand("setPathLength",boost::bind(&MCModule::setPathLength,this,_1,_2),
                        "This is to initialize the length of the path");
        RegisterCommand("setTrajectory",boost::bind(&MCModule::setTrajectory,this,_1,_2),
                        "This is to initialize the trajectory");
        RegisterCommand("setOdometry",boost::bind(&MCModule::setOdometry,this,_1,_2),
                        "This is to initialize the odometry");
        RegisterCommand("runSimulation",boost::bind(&MCModule::runSimulation,this,_1,_2),
                        "This is to run a MC simulation");
        RegisterCommand("setNumGaussians",boost::bind(&MCModule::setNumGaussians,this,_1,_2),
                        "This is to set number of Gaussians in mixture");
        RegisterCommand("runGMMEstimation",boost::bind(&MCModule::runGMMEstimation,this,_1,_2),
                        "Use sampling-based GMM algorithm to estimate probability of collision");

    }
    virtual ~MCModule() {}

    //Set Number of Gaussians in mixture
    bool setNumGaussians(std::ostream& sout, std::istream& sinput){
        int num;
        sinput >> num;
        sim.setNumGaussians(num);
        return true;
    }

    //Estimates probability of collision using GMM's
    //Requires number of Gaussians and initial covariance and trajectory
    //are known
    bool runGMMEstimation(std::ostream& sout, std::istream& sinput){
        sim.runGMMEstimation();
        return true;
    }

    //Run MC simulation
    bool runSimulation(std::ostream& sout, std::istream& sinput){
        double collprop = sim.runSimulation();
        std::cout << "collprop in C++: " << collprop << std::endl;
        //Send to output
        sout << collprop;
        return true;
    }

    bool setTrajectory(std::ostream& sout, std::istream& sinput){
        int length = sim.getPathLength();

        arma::Mat<double> traj = zeros<arma::Mat<double> >(3,length);
        double val;
        
        for(int component=0; component < 3; ++component){
            for(int i=0; i < length; ++i){
                sinput >> val;
                traj(component,i) = val;
            }
        }

        sim.setTrajectory(traj);
    }
    
    bool setOdometry(std::ostream& sout, std::istream& sinput){
        int length = sim.getPathLength() - 1;

        arma::Mat<double> odom = zeros<arma::Mat<double> >(3,length);
        double val;
        
        for(int component=0; component < 3; ++component){
            for(int i=0; i < length; ++i){
                sinput >> val;
                odom(component,i) = val;
            }
        }

        sim.setOdometry(odom);
    }

    bool setPathLength(std::ostream& sout, std::istream& sinput){
        int length;
        sinput >> length;
        sim.setPathLength(length);
    }

    bool setInitialCovariance(std::ostream& sout, std::istream& sinput){
        arma::Mat<double> cov = zeros<arma::Mat<double> >(3,3);

        double val;
        
        for(int i = 0; i < 3; ++i){
            for(int j = 0; j < 3; ++j){
                sinput >> val;
                cov(i,j) = val;
            }
        }

        sim.setInitialCovariance(cov);
    }
    
    bool setNumParticles(std::ostream& sout, std::istream& sinput){
        int num;
        sinput >> num;
        sim.setNumParticles(num);
    }
    
    bool setNumLandmarks(std::ostream& sout, std::istream& sinput){
        int num;
        sinput >> num;
        sim.setNumLandmarks(num);
    }

    bool setLandmarks(std::ostream& sout, std::istream& sinput){
        std::string locations;
        int num = sim.getNumLandmarks();
        arma::Mat<double> landmarks = zeros<arma::Mat<double> >(2,num);

        for(int i = 0; i < num; ++i){
            double val;
            sinput >> val;
            landmarks(0,i) = val;
        }
        
        for(int i = 0; i < num; ++i){
            double val;
            sinput >> val;
            landmarks(1,i) = val;
        }
        
        sim.setLandmarks(landmarks);
    }

    bool setQ(std::ostream& sout, std::istream& sinput){
        double Q;
        sinput >> Q;
        sim.setQ(Q);
    }
    
    bool setAlphas(std::ostream& sout, std::istream& sinput)
        {
            std::vector<double> alphas;
            std::string curr;
            while(sinput >> curr){
                //To double
                std::string::size_type sz;     // alias of size_t
                double alph = std::stod (curr,&sz);
                alphas.push_back(alph);
            }
            sim.setAlphas(alphas);
            
            return true;
        }

    bool ArmaCommand(std::ostream& sout, std::istream& sinput)
    {
        // Initialize the random generator
        arma::arma_rng::set_seed_random();
        std::cout << "inside armacommand" << std::endl;
            
        arma::Mat<double> C = { {1, 3, 5},
                                {2, 4, 6} };

        std::cout << "C matrix: " << C << "\n";

        // Create a 4x4 random matrix and print it on the screen
        arma::Mat<double> A = arma::randu(4,4);
        std::cout << "A:\n" << A << "\n";
    
        // Multiply A with his transpose:
        std::cout << "A * A.t() =\n";
        std::cout << A * A.t() << "\n";
    
        // Access/Modify rows and columns from the array:
        A.row(0) = A.row(1) + A.row(3);
        A.col(3).zeros();
        std::cout << "add rows 1 and 3, store result in row 0, also fill 4th column with zeros:\n";
        std::cout << "A:\n" << A << "\n";
    
        // Create a new diagonal matrix using the main diagonal of A:
        arma::Mat<double>B = arma::diagmat(A);
        std::cout << "B:\n" << B << "\n";
    
        // Save matrices A and B:
        A.save("A_mat.txt", arma::arma_ascii);
        B.save("B_mat.txt", arma::arma_ascii);
        
        return true;
    }

    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "mcmodule" ) {
        return InterfaceBasePtr(new MCModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("MCModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

