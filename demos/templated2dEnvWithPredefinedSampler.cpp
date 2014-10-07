/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

// TODO: encapsulate heuristics in the MotionCompare function.
// TODO: udpdate documentation for name changes (set_w, set_h)
// TODO: ask if I should do this on the FMT as well.
// TODO: think about the possibility of inherit from FMT.
// TODO: remove saveTree function.

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/FMTHeuristic.h>
#include <ompl/util/PPM.h>
#include <ompl/base/PredefinedStateSampler.h>


#include <ompl/config.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

    const std::vector<const ob::State*>& states_;
};


template<typename planner_t>
class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const char *ppm_file)
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
            ss_->setStateValidityChecker(boost::bind(&Plane2DEnvironment::isStateValid, this, _1));
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

            // configuring underlying planner
            //og::FMTHeuristic* fmt = new og::FMTHeuristic(ss_->getSpaceInformation());
            ss_->setPlanner(ob::PlannerPtr( new planner_t(ss_->getSpaceInformation()) ));
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
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
        ss_->solve();

        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath())
        {
            /*ss_->simplifySolution();
            og::PathGeometric &p = ss_->getSolutionPath();
            ss_->getPathSimplifier()->simplifyMax(p);
            ss_->getPathSimplifier()->smoothBSpline(p);*/
            return true;
        }
        else
            return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
    }

    const ob::SpaceInformationPtr getSpaceInformation() const
    {
        return ss_->getSpaceInformation();
    }

    void saveTree(const std::string &filename)
    {
        ss_->getPlanner()->as<og::FMTHeuristic>()->saveTree(filename);
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

    //Plane2DEnvironment<og::FMT> env((path / "ppm/empty.ppm").string().c_str());
    //Plane2DEnvironment<og::FMTHeuristic> env2((path / "ppm/empty.ppm").string().c_str());
    Plane2DEnvironment<og::FMT> env((path / "ppm/floor.ppm").string().c_str());
    Plane2DEnvironment<og::FMTHeuristic> env2((path / "ppm/floor.ppm").string().c_str());

    // Filling the predefined sampler.
    const size_t n = 1000;
    std::vector<const ob::State*> states;
    preSampleFree(env.getSpaceInformation(), n, states);
    PredefinedStateSamplerAllocator pssa(states);
    env.setStateSamplerAllocator(pssa);
    env2.setStateSamplerAllocator(pssa);

    if (env.plan(atoi(argv[1]), atoi(argv[2]),atoi(argv[3]),atoi(argv[4])))
    {
        env.recordSolution();
        env.saveTree("tree.txt");
        env.save("result_demo.ppm");
    }

    if (env2.plan(atoi(argv[1]), atoi(argv[2]),atoi(argv[3]),atoi(argv[4])))
    {
        env2.recordSolution();
        env2.saveTree("tree2.txt");
        env2.save("result_demo2.ppm");
    }

    return 0;
}
