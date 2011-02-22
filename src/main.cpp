/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include <omplapp/config.h>
#include <omplapp/SE3RigidBodyPlanning.h>
#include <ompl/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/BasicPRM.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

#include <ros/ros.h>
#include <ros/param.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace ompl;

template <typename T>
T requireGet(const std::string& param_name)
{
    T t;
    if (!ros::param::get(param_name, t))
    {
        ROS_ERROR_STREAM("Can't find required parameter " << param_name << ".");
    }

    return t;
}

void getState(const std::string& param_name, base::ScopedState<base::SE3StateManifold>& state)
{
    XmlRpc::XmlRpcValue elements = requireGet<XmlRpc::XmlRpcValue>(param_name);
    ROS_ASSERT(elements.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(elements.size() == 7);
    
    for (int i = 0; i < elements.size(); ++i)
        ROS_ASSERT(elements[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    state->setXYZ(
        static_cast<double>(elements[0]),
        static_cast<double>(elements[1]),
        static_cast<double>(elements[2])
    );

    state->rotation().setAxisAngle(
        static_cast<double>(elements[3]),
        static_cast<double>(elements[4]),
        static_cast<double>(elements[5]),
        static_cast<double>(elements[6])
    );

    ROS_DEBUG_STREAM("Set state to " << state);
}

void benchmark_setup(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
    double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = requireGet<std::string>("environment");
    {
        const std::string root_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/" + benchmark_name;
        const std::string robot_fname = root_fname + "_robot.dae";
        const std::string env_fname = root_fname + "_env.dae";
        setup.setRobotMesh(robot_fname.c_str());
        setup.setEnvironmentMesh(env_fname.c_str());
    }

    {
        base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
        getState("start", start);
        base::ScopedState<base::SE3StateManifold> goal(start);
        getState("goal", goal);
        setup.setStartAndGoalStates(start, goal);
    }

    {
        const double resolution = requireGet<double>("StateValidityCheckingResolution");
        setup.getSpaceInformation()->setStateValidityCheckingResolution(resolution);
    }
    
    {
        base::ValidStateSamplerAllocator allocator;
        
        const std::string sampler = requireGet<std::string>("sampler");
        if (sampler == "UniformValidStateSampler")
            allocator = base::UniformValidStateSampler::allocator();
        else if (sampler == "GaussianValidStateSampler")
            allocator = base::GaussianValidStateSampler::allocator();
        else if (sampler == "ObstacleBasedValidStateSampler")
            allocator = base::ObstacleBasedValidStateSampler::allocator();
        else
            ROS_ERROR_STREAM("Invalid sampler name: " << sampler);

        setup.getSpaceInformation()->setValidStateSamplerAllocator(allocator);
        benchmark_name.append("_" + sampler);
    }

    runtime_limit = requireGet<double>("runtime_limit");
    memory_limit = requireGet<double>("memory_limit");
    run_count = requireGet<int>("run_count");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CS790Ass01");
    ros::NodeHandle node;

    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;
    benchmark_setup(benchmark_name, setup, runtime_limit, memory_limit, run_count);

    // create the benchmark object and add all the planners we'd like to run
    Benchmark b(setup, benchmark_name);
    b.addPlanner(base::PlannerPtr(new geometric::BasicPRM(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::SBL(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(setup.getSpaceInformation())));

    // run all planners on the benchmark problem
    b.benchmark(runtime_limit, memory_limit, run_count, true);
    b.saveResultsToFile((benchmark_name + ".log").c_str());

    return 0;
}
