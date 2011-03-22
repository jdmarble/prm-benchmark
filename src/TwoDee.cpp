#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/RealVectorStateManifold.h>
#include <ompl/geometric/planners/prm/BGL_PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <boost/program_options.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace b = boost;
namespace po = boost::program_options;

#include "baswana_randomized_spanner.h"

typedef ob::RealVectorStateManifold::StateType R2State;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect    
    const R2State* r2state = state->as<R2State>();
    const double x = (*r2state)[0];
    const double y = (*r2state)[1];
    
    // check validity of state
    const bool inSlat = (int)(y*10)%2;
    return !(x > -0.25 && x < 0.25 && inSlat);
}

void plan(const unsigned int target_n)
{
    // construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::RealVectorStateManifold(2));

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    manifold->as<ob::RealVectorStateManifold>()->setBounds(bounds);

    // construct an instance of  space information from this manifold
    ob::SpaceInformationPtr si(new ob::SpaceInformation(manifold));

    // setting collision checking resolution to 1% of the space extent
    si->setStateValidityCheckingResolution(0.0001);

    si->setup();

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));

    // create a planner for the defined space
    ob::PlannerPtr planner(new og::BGL_PRM(si));

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cerr);

    const og::BGL_PRM::Graph& g = planner->as<og::BGL_PRM>()->getGraph();

    // Repeat for at least target_n milestones have been added.
    while (b::num_vertices(g) < target_n)
    {
        planner->as<og::BGL_PRM>()->growRoadmap(0.01);
        std::cerr << b::num_vertices(g) << ',' << b::num_edges(g) << std::endl;
    }

    BaswanaSpanner<og::BGL_PRM::Graph> spannerCalc(g, 9, false);
    const std::list<og::BGL_PRM::Edge> s = spannerCalc.calculateSpanner();

    std::cout << "index,spanner,x0,y0,x1,y1" << std::endl;
    b::property_map<og::BGL_PRM::Graph, og::BGL_PRM::vertex_state_t>::const_type
        state = boost::get(og::BGL_PRM::vertex_state_t(), g);
    b::property_map<og::BGL_PRM::Graph, b::edge_index_t>::const_type
        index = boost::get(b::edge_index, g);
    foreach(og::BGL_PRM::Edge e, b::edges(g))
    {
        std::cout << b::lexical_cast<std::string>(index[e]) << ',';

        char subgraph = '0';
        if (std::find(s.begin(), s.end(), e) != s.end())
            subgraph = '1';
        std::cout << subgraph << ',';

        R2State* s1 = state[b::source(e, g)]->as<R2State>();
        std::cout << b::lexical_cast<std::string>((*s1)[0]) << ',';
        std::cout << b::lexical_cast<std::string>((*s1)[1]) << ',';

        R2State* s2 = state[b::target(e, g)]->as<R2State>();
        std::cout << b::lexical_cast<std::string>((*s2)[0]) << ',';
        std::cout << b::lexical_cast<std::string>((*s2)[1]) << std::endl;
    }
}

int main(int argc, char* argv[])
{
    // Turn off logging to std::out. Need the stream for file output.
    ompl::msg::noOutputHandler();

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("n", po::value<unsigned int>(), "number of nodes in graph")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cerr << desc << "\n";
        return 1;
    }

    plan(vm["n"].as<unsigned int>());

    return 0;
}