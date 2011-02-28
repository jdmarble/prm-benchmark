#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/prm/KNearestPRMStar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <omplapp/SE3RigidBodyPlanning.h>

#include <iostream>
#include <numeric>
#include <map>

#include "baswana_randomized_spanner.h"
#include "edge_mask.h"

using namespace ompl;

// For counting the size of a nested vector.
template <typename T>
unsigned int accumSizes(unsigned int acc, const std::vector<T>& vec)
{
    return vec.size() + acc;
}

int main(int, char **)
{
    // plan in SE3
    app::SE3RigidBodyPlanning setup;
    
    // create a planner for the defined space
    base::PlannerPtr planner(new geometric::KNearestPRMStar(setup.getSpaceInformation()));
    setup.setPlanner(planner);
    base::PlannerData plannerData;
    
    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/cubicles_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define start state
    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(70.57);
    start->setZ(40.62);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateManifold> goal(start);
    goal->setX(200.49);
    goal->setY(70.57);
    goal->setZ(40.62);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // Repeat for some iterations
    while (plannerData.states.size() < 1000)
    {
    	planner->as<geometric::BasicPRM>()->growRoadmap(0.5);
        planner->getPlannerData(plannerData);

        unsigned int verts = plannerData.states.size();
        unsigned int edges =
            std::accumulate(plannerData.edges.begin(), plannerData.edges.end(),
                0, accumSizes<unsigned int>) / 2;
        
        std::cout << verts << ',' << edges << std::endl;
    }

    typedef geometric::KNearestPRMStar::Graph Graph;
    typedef geometric::KNearestPRMStar::Edge Edge;

    Graph G = planner->as<geometric::KNearestPRMStar>()->getGraph();
    std::cout << "Graph vertices: " << boost::num_vertices(G) << std::endl;
    std::cout << "Graph edges: " << boost::num_edges(G) << std::endl;

    std::map<Edge, bool> spanner_map;
    typedef boost::associative_property_map<std::map<Edge, bool> > spanner_t;
    spanner_t spanner_edge(spanner_map);
    
    boost::baswana_randomized_3_spanner(G, spanner_edge);
    
    std::cout << "Spanner edges: " << spanner_map.size() << std::endl;
    
    return 0;
}
