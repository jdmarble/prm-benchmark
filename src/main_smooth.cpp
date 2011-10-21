#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/planners/prm/IRS2.h>
#include <ompl/geometric/planners/prm/ConnectionFilter.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/SimpleSetup.h>

#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>

#include <boost/program_options.hpp>
#include <iostream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "baswana_randomized_spanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace b = boost;
namespace po = boost::program_options;

typedef ob::RealVectorStateSpace::StateType R2State;
typedef og::PRM::Graph Graph;
typedef og::PRM::Vertex Vertex;
typedef og::PRM::Edge Edge;
typedef std::pair<ob::State*, ob::State*> Witness;


ob::PlannerPtr setupPlanner (const ob::SpaceInformationPtr si,
    const double stretch, const double epsilon, const bool baswana)
{
    ob::PlannerPtr planner;
    if (stretch > 1.0 && epsilon > 0.0)
    {
        if (baswana)
        {
            std::cerr << "Can't run Baswana algorithm with additive term.";
            exit(1);
        }
        planner.reset(new og::IRS2(si, stretch, epsilon));
    }
    else if (stretch > 1.0 && !baswana)
    {
        planner.reset(new og::PRM(si, true));
        planner->setName("IRS");

        og::PRM* prm = planner->as<og::PRM>();

        prm->setConnectionFilter(
            og::IRSFilter(prm->getRoadmap(), stretch,
                boost::bind(&og::PRM::distanceFunction, prm, _1, _2)));
    }
    else
    {
        planner.reset(new og::PRM(si, true));
        planner->setName("kPRM_star");
    }

    return planner;
}

void outputWitnessQuality (ob::PlannerPtr planner,
    const std::vector<Witness>& witnesses)
{
    std::vector<ob::PathPtr> solution_paths;
    foreach (const Witness& witness, witnesses)
    {
        planner->getProblemDefinition()
            ->setStartAndGoalStates(witness.first, witness.second);
        planner->solve(1.0);
        ob::PathPtr path =
            planner->getProblemDefinition()->getGoal()->getSolutionPath();
        solution_paths.push_back(path);
    }

    og::PathSimplifier simplifier(planner->getSpaceInformation());

    const ompl::time::point start = ompl::time::now();

    double original_cost = 0;
    double smoothed_cost = 0;
    foreach (ob::PathPtr path, solution_paths)
    {
        original_cost += path->length();
        og::PathGeometric* gPath = dynamic_cast<og::PathGeometric*>(path.get());
        gPath->subdivide();
        gPath->subdivide();
        gPath->subdivide();
        simplifier.simplifyMax(*gPath);
        smoothed_cost += path->length();
    }
    
    const double smoothing_time = ompl::time::seconds(ompl::time::now() - start);

    std::cout << " :original_cost " << original_cost / witnesses.size();
    std::cout << " :smoothed_cost " << smoothed_cost / witnesses.size();
    std::cout << " :smoothing_time " << smoothing_time / witnesses.size();
}

void generateWitnesses (const unsigned int n, ob::PlannerPtr planner,
    std::vector<Witness>& output)
{
    assert(output.empty());
    ob::SpaceInformationPtr si = planner->getSpaceInformation();
    ob::ValidStateSamplerPtr sampler = si->allocValidStateSampler();

    std::vector<ob::State*> samples;
    for (unsigned int i = 0; i < n; ++i)
    {
        ob::State* state = si->allocState();
        sampler->sample(state);

        foreach(ob::State* const goal, samples)
            output.push_back(std::make_pair(state, goal));

        samples.push_back(state);
    }

    assert(output.size() == (n*n - n)/2);
}

void runExperiment (ob::PlannerPtr planner, const std::vector<Witness>& witnesses,
    const unsigned int max_size, const double max_time,
    const std::string& environment, const double stretch, const double epsilon,
    const bool baswana)
{
    og::PRM* prm = dynamic_cast<og::PRM*>(planner.get());
    assert(prm != NULL);
    prm->resetSampler();

    const unsigned int vertex_size = sizeof(og::PRM::VertexProperties) +
        planner->getSpaceInformation()->getStateDimension() * sizeof(double);

    const unsigned int edge_size = sizeof(og::PRM::EdgeProperties) +
        2 * sizeof(Vertex);

    const Graph& g = planner->as<og::PRM>()->getRoadmap();

    std::cout << "{:environment :" << environment <<
        " :planner :" << planner->getName();

    if (stretch > 1.0)
        std::cout << " :stretch " << stretch;
    if (epsilon > 0.0)
        std::cout << " :epsilon " << epsilon;

    unsigned int size = 0;
    double time = 0;
    const double time_step = 1.0;
    while (size < max_size && time < max_time)
    {
        planner->as<og::PRM>()->growRoadmap(time_step);
        time += time_step;
    }

    if (baswana)
    {
        // Stretch must be of the form 2k+1 for some integer k > 0.
        assert(std::floor((stretch+1)/2) == (stretch+1)/2);
        unsigned int k = (stretch+1)/2;
        assert(k > 0);
        std::cout << " :k " << k << ' ' << std::endl;

        BaswanaSpanner spannerCalc(g, 1000, k, false);
        std::cout << " :after_baswana " << spannerCalc.calculateSpanner().size();
    }

    outputWitnessQuality(planner, witnesses);
    std::cout << " :m " << num_edges(g);
    std::cout << '}' << std::endl;
}

bool isR2StateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const R2State* r2state = state->as<R2State>();
    const double x = (*r2state)[0];
    const double y = (*r2state)[1];

    // check validity of state
    const bool inSlat = (int)(y*10)%2;
    return !(inSlat && x > -0.75 && x < 0.75);
}

og::SimpleSetup* setupR2()
{
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    og::SimpleSetup* setup = new og::SimpleSetup(space);

    // setting collision checking resolution to 1% of the space extent
    setup->getSpaceInformation()->setStateValidityChecker(
        boost::bind(&isR2StateValid, _1));

    return setup;
}


og::SimpleSetup* setupSE3(const std::string environment)
{
    ompl::app::SE3RigidBodyPlanning* setup
        = new ompl::app::SE3RigidBodyPlanning();

    // load the robot and the environment
    const std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/3D/" + environment + "_robot.dae";
    const std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/3D/" + environment + "_env.dae";
    setup->setRobotMesh(robot_fname.c_str());
    setup->setEnvironmentMesh(env_fname.c_str());

    return setup;
}

og::SimpleSetup* setupSE2()
{
    ompl::app::SE2RigidBodyPlanning* setup
        = new ompl::app::SE2RigidBodyPlanning();

    // load the robot and the environment
    const std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/2D/UniqueSolutionMaze_robot.dae";
    const std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR)
            + "/2D/UniqueSolutionMaze_env.dae";
    setup->setRobotMesh(robot_fname.c_str());
    setup->setEnvironmentMesh(env_fname.c_str());

    return setup;
}

int main(int argc, char* argv[])
{
    // Turn off logging to std::out. Need the stream for file output.
    ompl::msg::noOutputHandler();

    // Use the same seed for each run to get the same witnesses.
    // Later, change the seed to the one specified on the command line.
    ompl::RNG::setSeed(1);

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("environment", po::value<std::string>(), "environment name")
        ("t", po::value<double>(), "stretch factor")
        ("e", po::value<double>(), "additive error term")
        ("size", po::value<unsigned int>(), "maximum size of roadmap (bytes)")
        ("time", po::value<double>(), "maximum time (seconds)")
        ("witness", po::value<unsigned int>(), "generate (w^2 * w)/2 queries")
        ("seed", po::value<boost::uint32_t>(), "PRNG seed")
        ("baswana", po::value<int>(), "use 2k-1 randomized spanner algorithm")
    ;

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    }
    catch (boost::program_options::error& e)
    {
        std::cerr << "Can't process command line arguments: " << e.what() << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cerr << desc << "\n";
        return 1;
    }
    const std::string environment = vm["environment"].as<std::string>();
    const double stretch = vm["t"].as<double>();
    const double epsilon = vm["e"].as<double>();
    const unsigned int max_size = vm["size"].as<unsigned int>();
    const double max_time = vm["time"].as<double>();
    const unsigned int witness_samples = vm["witness"].as<unsigned int>();
    const boost::uint32_t seed = vm["seed"].as<boost::uint32_t>();
    const int baswana = vm["baswana"].as<int>();
    
    // =========================================================================

    og::SimpleSetup* setup;
    if (environment == "point")
        setup = setupR2();
    else if (environment == "maze")
        setup = setupSE2();
    else
        setup = setupSE3(environment);

    // setting collision checking resolution to 1% of the space extent
    setup->getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    ob::PlannerPtr planner = setupPlanner(
        setup->getSpaceInformation(), stretch, epsilon, baswana);
    setup->setPlanner(planner);
    setup->setup();

    std::vector<Witness> witnesses;
    generateWitnesses(witness_samples, planner, witnesses);
    // Set real seed -after- so each run has the same witnesses but different
    // results.
    for(boost::uint32_t i; i < seed; ++i)
        ompl::RNG discard(); // Hack to advance the seeds...
    
    // =========================================================================

    runExperiment(planner, witnesses, max_size, max_time,
        environment, stretch, epsilon, baswana);
}