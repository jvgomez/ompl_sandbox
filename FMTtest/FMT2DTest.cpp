/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Stanford University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Javier V. Gomez */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/PredefinedStateSampler.h>

#include <ompl/config.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool heuristics = false;
bool nearestK = true;
bool cacheCC = false;

struct Path2D
{
    std::vector<ob::State*> spath_;
    double time_, cost_;
};

struct Config
{
    double max_time_;
    std::size_t exp_;
    std::vector<std::size_t> n_samples_;
};

void preSampleFree(const ob::SpaceInformationPtr si, const size_t n, std::vector<const ob::State*> &states)
{
    size_t statesSampled = 0;
    ob::StateSamplerPtr sampler = si->allocStateSampler();
    states.reserve(n);
    ob::State* s = si->allocState();

    while (statesSampled < n)
    {
        sampler->sampleUniform(s);
        bool collision_free = si->isValid(s);
        if (collision_free)
        {
            ++statesSampled;
            states.push_back(s);
            s = si->allocState();
        } // If collision free
    } // While statesSampled < n
}

struct PredefinedStateSamplerAllocator
{
    PredefinedStateSamplerAllocator(std::vector<const ob::State*> states) : states_(states) {}

    PredefinedStateSamplerAllocator(const char *filename)
    {
        ob::RealVectorStateSpace ss(2);
        std::ifstream ifs;
        ifs.open(filename);

        std::cout << std::fixed;

        if (ifs.is_open())
        {
            double x,y;
            while(ifs.good())
            {
                ifs >> x >> y;
                ob::State* s = ss.allocState();
                s->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
                s->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;
                states_.push_back(s);
            }
        }
        else
            std::cout << "Error opening file " << filename << "!" << std::endl;
        ifs.close();
    }

    ob::StateSamplerPtr operator()(const ob::StateSpace* space)
    {
        return ob::StateSamplerPtr(new ob::PredefinedStateSampler(space, states_));
    }

    void saveSamples(const char *filename)
    {
        std::ofstream ofs;
        ofs.open(filename, std::ofstream::trunc);
        ofs << std::fixed;

        for (std::size_t i = 0; i < states_.size(); ++i)
        {
            ofs << states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0] << "\t"
                << states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1];

            if (i != states_.size()-1)
                ofs << std::endl;
        }
        ofs.close();
    }

    std::vector<const ob::State*> states_;
};

void loadData(const char *filename, std::vector<Path2D> &paths, Config &config)
{
    ob::RealVectorStateSpace ss(2);

    std::ifstream ifs;
    ifs.open(filename);
    if (ifs.is_open())
    {
        // Reading header of the results file.
        ifs >> config.exp_ >> config.max_time_;
        double val;
        for (unsigned int i = 0; i < config.exp_; ++i)
        {
            ifs >> val;
            config.n_samples_.push_back(val);
        }

        // Reading paths.
        double x,y;
        for (unsigned int i = 0; i < config.exp_; ++i)
        {
            Path2D path;
            std::size_t n_states;
            ifs >> path.cost_ >> path.time_ >> n_states;
            // Reading states of each path.
            for (std::size_t j = 0; j < n_states; ++j)
            {
                ob::State *s = ss.allocState();
                ifs >> x >> y;
                s->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
                s->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;
                path.spath_.push_back(s);
            }
            paths.push_back(path);
        }
    }
    else
        std::cout << "Error opening file " << filename << "!" << std::endl;
}

std::size_t differentPaths(const std::vector<ob::State*> p1, const std::vector<ob::State*> p2)
{
    std::size_t diff = 0;

    if (p1.size() != p2.size())
        return abs(p1.size() - p2.size());
    else
    {
        ob::RealVectorStateSpace ss(2);
        for (std::size_t i = 0; i < p1.size(); ++i)
        {
            if (!ss.equalStates(p1[i], p2[i]))
                ++diff;
        }
    }

    return diff;
}

template<typename planner_t>
class Test2D
{
public:

    Test2D(const char *ppm_file)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

            // set state validity checking for this space
            ss_->setStateValidityChecker(boost::bind(&Test2D::isStateValid, this, _1));
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

            // configuring underlying planner
            ss_->setPlanner(ob::PlannerPtr( new planner_t(ss_->getSpaceInformation()) ));
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col, double max_time)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);
        ss_->solve(max_time);

        if (ss_->haveSolutionPath())
            return true;
        else
            return false;
    }

    const ob::SpaceInformationPtr getSpaceInformation() const
   {
       return ss_->getSpaceInformation();
   }

    const og::SimpleSetupPtr getSetup() const
   {
       return ss_;
   }

    const ob::PlannerPtr getPlanner() const
    {
        return ss_->getPlanner();
    }

    void setStateSamplerAllocator(const ob::StateSamplerAllocator ssa)
    {
        ss_->getStateSpace()->setStateSamplerAllocator(ssa);
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
};

int main(int argc, char ** argv)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);

    // Reading states to use in the test.
    PredefinedStateSamplerAllocator pssa("fmttest_samples_k.txt");

    if (pssa.states_.empty())
        return 1;

    // Reading paths and configuration.
    Config config;
    std::vector<Path2D> paths;
    loadData("fmttest_paths_k.txt", paths, config);

    if (paths.empty())
        return 1;

    // Executing sampler to test and caching results.
    std::vector<Path2D> result_paths;
    result_paths.reserve(config.exp_);
    for (size_t i = 0; i < config.exp_; ++i)
    {
        Test2D<og::FMT> test_planner((path / "ppm/floor.ppm").string().c_str());

        test_planner.getPlanner()->as<og::FMT>()->setNumSamples(config.n_samples_[i]);
        test_planner.getPlanner()->as<og::FMT>()->setNearestK(nearestK);
        //test_planner.getPlanner()->as<og::FMT>()->setHeuristics(heuristics);
        //test_planner.getPlanner()->as<og::FMT>()->setCacheCC(cacheCC);
        test_planner.setStateSamplerAllocator(pssa);

        if (test_planner.plan(0,0, 1859, 1568, config.max_time_)
            && test_planner.getSetup()->haveExactSolutionPath())
        {
            Path2D path;
            std::vector<ob::State*> spath = test_planner.getSetup()->getSolutionPath().getStates();
            // A deep copy of the states is required.
            for(std::size_t j = 0; j < spath.size(); ++j)
                path.spath_.push_back(test_planner.getSpaceInformation()->cloneState(spath[j]));

            path.cost_ = test_planner.getSetup()->getSolutionPath().length();
            path.time_ = test_planner.getSetup()->getLastPlanComputationTime();
            result_paths.push_back(path);
        }
    }

    // Analyzing results.
    std::cout << std::endl << "***** ANALYZING RESULTS *****" << std::endl << std::endl;
    bool warn = false;
    for (size_t i = 0; i < config.exp_; ++i)
    {
        bool local_warn = false;
        std::cout << "[TEST " << i << "] ------------------- "<< std::endl;
        if (abs(paths[i].cost_ - result_paths[i].cost_) > std::numeric_limits<double>::epsilon())
        {
            warn = true;
            local_warn = true;
            std::cout << "[WARNING]: Path costs are different:" << std::endl;
            std::cout << "\t" << "Exp: " << i << std::endl;
            std::cout << "\t" << "N: " << config.n_samples_[i] << std::endl;
            std::cout << "\t" << "Real cost: " << paths[i].cost_ << std::endl;
            std::cout << "\t" << "Test cost: " << result_paths[i].cost_ << std::endl;
            std::cout << "\t" << "Ratio test/real: " << (double)result_paths[i].cost_ / paths[i].cost_ << std::endl << std::endl;
        }

        if ((paths[i].time_/result_paths[i].time_) < 0.5 || (result_paths[i].time_/paths[i].time_) < 0.5 )
        {
            warn = true;
            local_warn = true;
            std::cout << "[WARNING]: Computation times differ too much:" << std::endl;
            std::cout << "\t" << "Exp: " << i << std::endl;
            std::cout << "\t" << "N: " << config.n_samples_[i] << std::endl;
            std::cout << "\t" << "Real time: " << paths[i].time_ << std::endl;
            std::cout << "\t" << "Test time: " << result_paths[i].time_ << std::endl;
            std::cout << "\t" << "Ratio test/real: " << (double)result_paths[i].time_ / paths[i].time_ << std::endl << std::endl;
        }

        std::size_t diff_states = differentPaths(paths[i].spath_, result_paths[i].spath_);
        if (diff_states)
        {
            warn = true;
            local_warn = true;
            std::cout << "[WARNING]: Paths have " <<diff_states << " different state." << std::endl;
            std::cout << "\t" << "Exp: " << i << std::endl;
            std::cout << "\t" << "N: " << config.n_samples_[i] << std::endl << std::endl;
        }

        if (!local_warn)
            std::cout << "[TEST OK]" << std::endl << std::endl;
    }

    if (warn)
    {
        std::cout << "THERE WERE WARNINGS! Test NOT successfull." << std::endl;
        return 1;
    }
    else
    {
        std::cout << "*** Test OK! ***" << std::endl;
        return 0;
    }
}
