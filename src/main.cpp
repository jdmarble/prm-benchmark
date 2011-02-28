#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/prm/KNearestPRMStar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <omplapp/SE3RigidBodyPlanning.h>

#include <iostream>
#include <numeric>

#include "baswana_randomized_spanner.h"

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

    // Repeat for 10000 iterations
    while (plannerData.states.size() < 50000)
    {
    	planner->as<geometric::BasicPRM>()->growRoadmap(0.5);
        planner->getPlannerData(plannerData);

        unsigned int verts = plannerData.states.size();
        unsigned int edges =
            std::accumulate(plannerData.edges.begin(), plannerData.edges.end(),
                0, accumSizes<unsigned int>) / 2;
        
        std::cout << verts << ',' << edges << std::endl;
    }

    geometric::KNearestPRMStar::Graph g =
            planner->as<geometric::KNearestPRMStar>()->getGraph();
    std::cout << "Total vertices: " << boost::num_vertices(g) << std::endl;
    std::cout << "Total edges: " << boost::num_edges(g) << std::endl;

    std::vector<geometric::KNearestPRMStar::Edge> spanner_edges;
    std::insert_iterator<std::vector<geometric::KNearestPRMStar::Edge> >
        insert_edge(spanner_edges, spanner_edges.begin());
    baswana_randomized_3_spanner(g, insert_edge);

    geometric::KNearestPRMStar::Graph s(boost::num_vertices(g));
    foreach(geometric::KNearestPRMStar::Edge e, spanner_edges)
    {
        const geometric::KNearestPRMStar::Graph::vertex_descriptor v1 = boost::source(e, s);
        const geometric::KNearestPRMStar::Graph::vertex_descriptor v2 = boost::target(e, s);
        const float w = 1.0;// TODO boost::get(weight, e);
        boost::add_edge(v1, v2, w, s);
    }
    std::cout << "Spanner edges: " << boost::num_edges(s) << std::endl;
    
    return 0;
}
