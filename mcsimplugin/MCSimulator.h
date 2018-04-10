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

class MCSimulator{


};


#endif /* MC_SIMULATOR_H */
