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

#ifndef LEGGED_ROBOT_HPP
#define LEGGED_ROBOT_HPP

#include "parameters.hpp"

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>


#ifdef VISU
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>
#endif

#define SIM_STABLE_MIN_STEPS 100 // How long to wait before the sim become stable
#define TARGET_VISU_HZ	60.

const int default_ik_iterations = 4500;

const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
#ifdef VISU
using namespace dart::gui;
using namespace dart::gui::osg;
#endif
using namespace dart::utils;
using namespace dart::math;

namespace legged {

class Controller
{
public:

    /// Constructor
    Controller(SkeletonPtr robot, Joint::ActuatorType at);


    /// Reset the desired dof position to the current position
    void setTargetPositions(const Eigen::VectorXd& pose)
    {
        mTargetPositions = pose;
    }

    // /// Clear commanding forces
    // void clearForces()
    // {
    //     _forces.setZero();
    // }


    // Set initial configuration
    void setInitialPose(SkeletonPtr robot);

    void setActuators();

    void setCommands(const std::vector<double> &cmds);
    void setCommands(const std::vector<float> &cmds);
    void setCommand(const std::string& joint_name, double value);

    const SkeletonPtr& get_model(){return _robot_model;}
    const std::string& get_model_name(){return _model_name;}
    const std::vector<std::string>& get_dofs_names(){return _dof_names;}
    const std::vector<std::string>& get_joints_names(){return _joint_names;}

protected:
    /// The biped Skeleton that we will be controlling
    SkeletonPtr _robot_model;
    std::string _model_name;
    std::vector<std::string> _dof_names;
    std::vector<std::string> _joint_names;

    // /// Joint forces for the biped (output of the Controller)
    // Eigen::VectorXd _forces;

    // /// Control gains for the proportional error terms in the PD controller
    // Eigen::MatrixXd mKp;

    // /// Control gains for the derivative error terms in the PD controller
    // Eigen::MatrixXd mKd;

    /// Target positions for the PD controllers
    Eigen::VectorXd mTargetPositions;

    Joint::ActuatorType _actuator_type;


};

class Simulation
{
public:
    /// Constructor
    Simulation(Joint::ActuatorType at, const std::string &model_path);

#ifdef VISU
    void init_visu();
#endif
    void update(int time_idx);
    void reset();

    void enable_self_collision(bool b){
        _self_collision = b;
        if(_self_collision)
            std::cout << "SELF COLLISION ENABLED" << std::endl;
        else std::cout << "SELF COLLISION DISABLED" << std::endl;
    }

    std::unique_ptr<Controller> _controller;
    WorldPtr _world;

protected:

    /// Number of iterations before clearing a force entry
    int _force_count_down;

    /// Whether a force should be applied in the positive or negative direction
    bool _positive_sign;

    unsigned int _steps_per_frame;
    double _timestep;

    bool _self_collision = true;

#ifdef VISU
    ImGuiViewer* _osgViewer;
    WorldNode* _osgWorldNode;
#endif


    SkeletonPtr _createFloor();
    // Load a biped model and enable joint limits and self-collision
    SkeletonPtr _load_model(const std::string &model_path);

    template<typename Param> 
    void _set_self_collision(){_self_collision = Param::simu::self_collision;}
};



}//legged

#endif //LEGGED_ROBOT_HPP
