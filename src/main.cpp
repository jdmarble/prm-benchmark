#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/prm/KNearestPRMStar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <omplapp/SE3RigidBodyPlanning.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
namespace bacc = boost::accumulators;
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <iostream>
#include <limits>
#include <map>
#include <numeric>

#include "baswana_randomized_3_spanner.h"

using namespace ompl;

void distanceStats(const std::vector<double>& d)
{
    bacc::accumulator_set<
        double,
        bacc::features< bacc::tag::max, bacc::tag::mean > > acc;
    unsigned int disconnected = 0;
    foreach(double r, d)
    {
        // Count disconnected components
        if (r == std::numeric_limits<double>::max())
            ++disconnected;
        else
            acc(r);
    }
    std::cout << "Warning! Disconnected nodes:" << disconnected << std::endl;

    std::cout << "Mean distance from start: " << bacc::mean(acc) << std::endl;
    std::cout << "Max. distance from start: " << bacc::max(acc) << std::endl;
}

template <class Graph>
void pathStats(const Graph& G, std::vector<double>& d)
{
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typename boost::graph_traits<Graph>::vertices_size_type
        n = boost::num_vertices(G);

    // Arbitrarily select the first vertex.
    Vertex start = *boost::vertices(G).first;

    // Need scratch vector for Dijkstra's
    std::vector<Vertex> predecessor(n);
    dijkstra_shortest_paths(G, start,
        boost::predecessor_map(&predecessor[0]).distance_map(&d[0]));

    // Calculate statistics
    distanceStats(d);
}

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
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/Easy_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/Easy_env.dae";
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
    while (plannerData.states.size() < 5000)
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
    
    const Graph::vertices_size_type n = boost::num_vertices(G);
    const Graph::edges_size_type m = boost::num_edges(G);
    boost::property_map<Graph, boost::edge_weight_t>::type
            weight = boost::get(boost::edge_weight, G);
    
    std::cout << "Graph vertices: " << n << std::endl;
    std::cout << "Graph edges: " << m << std::endl;

    Graph spanner(n);
    std::vector<Edge> edge_data;
    baswana_randomized_3_spanner(G, std::back_inserter(edge_data));
    foreach(Edge e, edge_data)
    {
        const Graph::vertex_descriptor v1 = boost::source(e, G);
        const Graph::vertex_descriptor v2 = boost::target(e, G);

        Edge e_p;
        bool exists;        
        boost::tie(e_p, exists) = boost::edge(v1, v2, spanner);
        if (!exists)
            boost::add_edge(v1, v2, weight[e], spanner);
    }
    
    std::cout << "Spanner edges: " << boost::num_edges(spanner) << std::endl;
   
    std::cout << "Graph stats..." << std::endl;
    std::vector<double> d_graph(n);
    pathStats(G, d_graph);

    std::cout << "Spanner stats..." << std::endl;
    std::vector<double> d_spanner(n);
    pathStats(spanner, d_spanner);

    std::cout << "Difference stats..." << std::endl;
    // d_diff = abs(d_graph - d_spanner)
    std::vector<double> d_diff(n);
    std::vector<double>::const_iterator graph_iter(d_graph.begin());
    std::vector<double>::const_iterator spanner_iter(d_spanner.begin());
    foreach(double& n, d_diff)
        if (*graph_iter < std::numeric_limits<double>::max() &&
            *spanner_iter < std::numeric_limits<double>::max())
            n = fabs(*graph_iter++ - *spanner_iter++);
        else
        {
            n = std::numeric_limits<double>::max();
            ++graph_iter;
            ++spanner_iter;
        }
    distanceStats(d_diff);
    
    return 0;
}
