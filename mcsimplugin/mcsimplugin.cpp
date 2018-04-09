#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <unordered_set>
#include <armadillo>
using namespace OpenRAVE;


class MCModule : public ModuleBase
{
public:
    MCModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&MCModule::MyCommand,this,_1,_2),
                        "This is an example command");
        RegisterCommand("ArmaCommand",boost::bind(&MCModule::ArmaCommand,this,_1,_2),
                        "This is testing armadillo");
    }
    virtual ~MCModule() {}
    
    bool ArmaCommand(std::ostream& sout, std::istream& sinput)
    {
        // Initialize the random generator
        arma::arma_rng::set_seed_random();
        std::cout << "inside armacommand" << std::endl;
            
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

