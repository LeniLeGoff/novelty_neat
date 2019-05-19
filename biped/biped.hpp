/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified by L.K. Le Goff
 */

#ifndef BIPED_HPP
#define BIPED_HPP

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>

#define SIM_STABLE_MIN_STEPS 100 // How long to wait before the sim become stable
#define TARGET_VISU_HZ	60.

const int default_ik_iterations = 4500;

const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::osg;
using namespace dart::utils;
using namespace dart::math;

namespace biped {




class Controller
{
public:

    static const std::vector<std::string> joint_names;
    static const std::vector<std::string> dof_names;
    /// Constructor
    Controller(SkeletonPtr biped, Joint::ActuatorType at);


    /// Reset the desired dof position to the current position
    void setTargetPositions(const Eigen::VectorXd& pose)
    {
        mTargetPositions = pose;
    }

    /// Clear commanding forces
    void clearForces()
    {
        _forces.setZero();
    }

    /// Add commanding forces from PD controllers
    void addPDForces();

    /// Add commanind forces from Stable-PD controllers
    void addSPDForces();

    /// add commanding forces from ankle strategy
    void addAnkleStrategyForces();

    // Set initial configuration
    void setInitialPose(SkeletonPtr biped);

    void setActuators();

    void setCommands(const std::string& joint_name, double value);

    const SkeletonPtr& get_biped(){return _biped;}

protected:
    /// The biped Skeleton that we will be controlling
    SkeletonPtr _biped;

    /// Joint forces for the biped (output of the Controller)
    Eigen::VectorXd _forces;

    /// Control gains for the proportional error terms in the PD controller
    Eigen::MatrixXd mKp;

    /// Control gains for the derivative error terms in the PD controller
    Eigen::MatrixXd mKd;

    /// Target positions for the PD controllers
    Eigen::VectorXd mTargetPositions;

    Joint::ActuatorType _actuator_type;


};

class Simulation
{
public:
    /// Constructor
    Simulation(Joint::ActuatorType at, const std::string &model_path);

    void init_visu();
    void update(int time_idx);
    void reset();

    void enable_spd(bool b){_spd_enabled = b;}
    void enable_ankle_strat(bool b){_ankle_strat_enabled = b;}

    std::unique_ptr<Controller> _controller;
    WorldPtr _world;

protected:

    /// Number of iterations before clearing a force entry
    int _force_count_down;

    /// Whether a force should be applied in the positive or negative direction
    bool _positive_sign;

    unsigned int _steps_per_frame;
    double _timestep;

    ImGuiViewer* _osgViewer;
    WorldNode* _osgWorldNode;

    bool _spd_enabled = false;
    bool _ankle_strat_enabled = false;

    SkeletonPtr _createFloor();
    // Load a biped model and enable joint limits and self-collision
    SkeletonPtr _loadBiped(const std::string &model_path);
    // Solve for a balanced pose using IK
    Eigen::VectorXd _solveIK(SkeletonPtr biped);
};



}//biped

#endif //BIPED_HPP
