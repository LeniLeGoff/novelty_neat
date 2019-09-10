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

#include "legged_robot.hpp"


using namespace legged;
namespace dart_constr = dart::constraint;



Controller::Controller(SkeletonPtr robot, Joint::ActuatorType at) :
    _actuator_type(at)
{
    _robot_model = robot;
    _model_name = robot->getName();
    int nDofs = _robot_model->getNumDofs();

    int i = 0;
    for(auto dof : robot->getDofs()){
      if(i > 5)
        _dof_names.push_back(dof->getName());
      i++;
    }

    for(auto joints : robot->getJoints())
      _joint_names.push_back(joints->getName());

    // _forces = Eigen::VectorXd::Zero(nDofs);

    setTargetPositions(_robot_model->getPositions());
}




// Set initial configuration (Lesson 2 Answer)
void Controller::setInitialPose(SkeletonPtr biped)
{
//  TODO
//   biped->setPosition(biped->getDof("j_thigh_left_z")->
//                      getIndexInSkeleton(), 0.15);
//   biped->setPosition(biped->getDof("j_thigh_right_z")->
//                      getIndexInSkeleton(), 0.15);
//   biped->setPosition(biped->getDof("j_shin_left")->
//                      getIndexInSkeleton(), -0.4);
//   biped->setPosition(biped->getDof("j_shin_right")->
//                      getIndexInSkeleton(), -0.4);
//   biped->setPosition(biped->getDof("j_heel_left_1")->
//                      getIndexInSkeleton(), 0.25);
//   biped->setPosition(biped->getDof("j_heel_right_1")->
//                      getIndexInSkeleton(), 0.25);
}

void Controller::setActuators(){
    for(Joint* joints : _robot_model->getJoints()){
        joints->setActuatorType(_actuator_type);
    }
}

void Controller::setCommands(const std::vector<double> &cmds){
  for(int i = 0; i < _dof_names.size(); i++){
    setCommand(_dof_names[i],cmds[i]);
  }
}

void Controller::setCommands(const std::vector<float> &cmds){
  for(int i = 0; i < _dof_names.size(); i++){
    setCommand(_dof_names[i],cmds[i]);
  }
}


void Controller::setCommand(const std::string &dof_name, double value){
    int index = _robot_model->getDof(dof_name)->getIndexInSkeleton();
    _robot_model->setCommand(index, value);
}

Simulation::Simulation(Joint::ActuatorType at, const std::string &model_path)
    : _force_count_down(0),
      _positive_sign(true)
{
    //_osgViewer = nullptr;
    //_osgWorldNode = nullptr;


    SkeletonPtr robot_model = _load_model(model_path);
    SkeletonPtr floor = _create_environment();

    _world = std::make_shared<World>();
    _world->setName("legged robot world");
    _world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));


    dart_constr::BoxedLcpSolverPtr lcp_solver(new dart_constr::DantzigBoxedLcpSolver);
    dart_constr::PgsBoxedLcpSolverPtr pgs_solver(new dart_constr::PgsBoxedLcpSolver);
    std::unique_ptr<dart_constr::ConstraintSolver> constr_solver(new dart_constr::BoxedLcpConstraintSolver(lcp_solver,pgs_solver));

    _world->setConstraintSolver(std::move(constr_solver));

    _world->getConstraintSolver()->setCollisionDetector(
                    dart::collision::DARTCollisionDetector::create());
    


    _world->addSkeleton(floor);
    _world->addSkeleton(robot_model);

    _controller = std::make_unique<Controller>(robot_model,at);

    _set_self_collision<Params>();
    // if(_self_collision)
    //   std::cout << "SELF COLLISION ENABLED" << std::endl;
    // else std::cout << "SELF COLLISION DISABLED" << std::endl;

}


#ifdef VISU
void Simulation::init_visu(){
    // Create a WorldNode
    _osgWorldNode = new dart::gui::osg::WorldNode(_world, nullptr);
    _osgWorldNode->setNumStepsPerCycle(1); // ??
    // Create a Viewer and set it up with the WorldNode
    _osgViewer = new dart::gui::osg::ImGuiViewer();
    _osgViewer->addWorldNode(_osgWorldNode);


    // Set the dimensions for the window
    _osgViewer->setUpViewInWindow(0, 0, 1280, 960);

    // Set the window name
    _osgViewer->realize();
    osgViewer::Viewer::Windows windows;
    _osgViewer->getWindows(windows);
    windows.front()->setWindowName("Legged Robot");

    // Adjust the viewpoint of the Viewer
    _osgViewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3d( 5.0, 3.0, 5.0 ), // Where the camera is
      ::osg::Vec3d( 0.0, 0.0, 1.0 ), // Where it is looking at
      ::osg::Vec3d( 0.0, 0.0, 1.0 ) // What is the "up" direction
    );
    // We need to re-dirty the CameraManipulator by passing it into the viewer
    // again, so that the viewer knows to update its HomePosition setting
    _osgViewer->setCameraManipulator(_osgViewer->getCameraManipulator());
    _steps_per_frame = std::ceil(1./(_world->getTimeStep()*TARGET_VISU_HZ));
}
#endif

void Simulation::update(int time_idx)
{
    // _controller->clearForces();

    _world->step();


#ifdef VISU
    if(_osgViewer)
    {
      if(time_idx % _steps_per_frame == 0){
          _osgViewer->frame();
      }
    }
#endif
}

void Simulation::reset(){
    _world->getSkeleton(_controller->get_model_name())->resetPositions();
    _world->getSkeleton(_controller->get_model_name())->resetVelocities();
    _world->getSkeleton(_controller->get_model_name())->resetAccelerations();

}

// Load a robot model and enable joint limits and self-collision
SkeletonPtr Simulation::_load_model(const std::string& model_path)
{

  DartLoader loader;
  SkeletonPtr robot = loader.parseSkeleton(model_path);
  assert(robot != nullptr);


  // Set joint limits
  for(std::size_t i = 0; i < robot->getNumJoints(); ++i)
    robot->getJoint(i)->setPositionLimitEnforced(true);

  // Enable self collision check but ignore adjacent bodies
  if(_self_collision){
    robot->enableSelfCollisionCheck();
    robot->disableAdjacentBodyCheck();
  }

  return robot;
}

SkeletonPtr Simulation::_create_environment()
{
  SkeletonPtr env = Skeleton::create("environment");

  //create floor
  BodyNodePtr floor =
      env->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  auto floorShape
      = floor->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  floorShape->getVisualAspect()->setColor(dart::Color::Black());

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  floor->getParentJoint()->setTransformFromParentBodyNode(tf);


  //create walls
  BodyNodePtr wall1 =
      env->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  double wall_width = 10;
  double wall_height = 3;
  double wall_thick = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(wall_width, wall_height, wall_thick)));



  auto wallShape
    = 
  return floor;
}
