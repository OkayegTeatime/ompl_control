/* Author: Justin Kottinger */

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include "World.h"
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "PostProcessing.h"


namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace oc = ompl::control;

// this function sets-up an ompl planning problem for an arbtrary number of agents
oc::SimpleSetupPtr controlSimpleSetUp(const World *w)
{
    // grab the agent -- assume only one
    Agent *a = w->getAgents()[0];

    // create state and control spaces
    ob::StateSpacePtr space = createBounded2ndOrderCarStateSpace(w->getWorldDimensions()[0], w->getWorldDimensions()[1]);
    oc::ControlSpacePtr cspace = createUniform2DRealVectorControlSpace(space);

    // define a simple setup class
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);

    // set state validity checker
    ss->setStateValidityChecker(std::make_shared<isStateValid_2D>(ss->getSpaceInformation(), w, a));

    // Use the ODESolver to propagate the system.  Call SecondOrderCarODE
    // when integration has finished to normalize the orientation values using SecondOrderCarODEPostIntegration
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &SecondOrderCarODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderCarODEPostIntegration));

    // create start
    ob::ScopedState<> start(space);
    start[0] = a->getStartLocation()[0];
    start[1] = a->getStartLocation()[1];
    start[2] = 0;
    start[3] = 0;
    start[4] = a->getStartLocation()[2];

    // create goal region
    ob::GoalPtr goal (new GoalRegion2ndOrderCar(ss->getSpaceInformation(), a->getGoalLocation()[0], a->getGoalLocation()[1]));
    
    // save start and goal
    ss->setStartState(start);
    ss->setGoal(goal);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}
