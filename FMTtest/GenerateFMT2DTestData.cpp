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
bool cacheCC = true;

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
    PredefinedStateSamplerAllocator(const std::vector<const ob::State*> &states) : states_(states) {}

    ob::StateSamplerPtr operator()(const ob::StateSpace* space)
    {
        return ob::StateSamplerPtr(new ob::PredefinedStateSampler(space, states_));
    }

    void saveSamples(const char * filename)
    {
        std::ofstream ofs;
        ofs.open(filename, std::ofstream::trunc);
        ofs << std::fixed;

        for (std::size_t i = 0; i < states_.size(); ++i)
        {
            ofs << states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0] << " "
                << states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1];

            if (i != states_.size()-1)
                ofs << std::endl;
        }
        ofs.close();
    }

    const std::vector<const ob::State*>& states_;
};


template<typename planner_t>
class Test2DDataGenerator
{
public:

    Test2DDataGenerator(const char *ppm_file)
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
            ss_->setStateValidityChecker(boost::bind(&Test2DDataGenerator::isStateValid, this, _1));
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

    size_t exp = 3, n_samples[] = {1000, 5000, 10000};
    double max_time = 20.0;

    // Local scope to load the state sampler
    std::vector<const ob::State*> states;
    {
        Test2DDataGenerator<og::FMT> aux((path / "ppm/floor.ppm").string().c_str());
        preSampleFree(aux.getSpaceInformation(), n_samples[exp-1], states);
    }
    PredefinedStateSamplerAllocator pssa(states);
    pssa.saveSamples("fmttest_samples.txt");

    // Saving header of the results file.
    std::stringstream fmtss;
    fmtss << std::fixed;
    fmtss << exp << " " << max_time;
    for (size_t i = 0; i < exp; ++i)
        fmtss << " " << n_samples[i];
    fmtss << std::endl;

    for (size_t i = 0; i < exp; ++i)
    {
        Test2DDataGenerator<og::FMT> test_planner((path / "ppm/floor.ppm").string().c_str());
        test_planner.getPlanner()->as<og::FMT>()->setNumSamples(n_samples[i]);
        test_planner.setStateSamplerAllocator(pssa);

        test_planner.getPlanner()->as<og::FMT>()->setNearestK(nearestK);
        //test_planner.getPlanner()->as<og::FMT>()->setHeuristics(heuristics);
        //test_planner.getPlanner()->as<og::FMT>()->setCacheCC(cacheCC);

        if (test_planner.plan(0,0, 1859, 1568, max_time)
            && test_planner.getSetup()->haveExactSolutionPath())
        {
            const double cost = test_planner.getSetup()->getSolutionPath().length();
            const double time = test_planner.getSetup()->getLastPlanComputationTime();
            const unsigned int n_states = test_planner.getSetup()->getSolutionPath().getStateCount();
            fmtss << cost << " " << time << " " << n_states << std::endl;
            test_planner.getSetup()->getSolutionPath().printAsMatrix(fmtss);
        }
        else
        {
            std::cout << "***** PLANNING FAILED ***** Please run again." << std::endl;
            return 1;
        }
    }

    std::ofstream fmtssofs;
    fmtssofs.open("fmttest_paths.txt", std::ofstream::trunc);
    fmtssofs << fmtss.rdbuf();
    fmtssofs.close();

    return 0;
}
