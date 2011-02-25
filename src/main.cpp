#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/prm/KNearestPRMStar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const ob::SE3StateManifold::StateType *se3state =
    	state->as<ob::SE3StateManifold::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateManifold::StateType *pos = 
    	se3state->as<ob::RealVectorStateManifold::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateManifold::StateType *rot = 
    	se3state->as<ob::SO3StateManifold::StateType>(1);

    // TODO: check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define,
    // so we avoid compiler warnings
    return (void*)rot != (void*)pos;
}

void plan(void)
{
    // construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE3StateManifold());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    manifold->as<ob::SE3StateManifold>()->setBounds(bounds);

    // construct an instance of  space information from this manifold
    ob::SpaceInformationPtr si(new ob::SpaceInformation(manifold));

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));

    // create a random start state
    ob::ScopedState<> start(manifold);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(manifold);
    goal.random();

    // create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    ob::PlannerPtr planner(new og::KNearestPRMStar(si));
    ob::PlannerData plannerData;

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

	// plan for a bit so everything that needs to be initialized is
	planner->solve(0.1);
	
    // Repeat for 10000 iterations
    while (plannerData.states.size() < 10000)
    {
    	planner->as<og::BasicPRM>()->growRoadmap(0.5);
        planner->getPlannerData(plannerData);

        if (pdef->getGoal()->isAchieved())
        {
            ob::PathPtr path = pdef->getGoal()->getSolutionPath();
            std::cout << plannerData.states.size() << ',' <<
                path->length() << std::endl;
        }
        else
        {
            std::cout << plannerData.states.size() << ", inf" << std::endl;
        }
    }
}


int main(int, char **)
{
    std::cout << "ompl version: " << OMPL_VERSION << std::endl;

	plan();

    return 0;
}
