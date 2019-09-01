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

#include "biped.hpp"


using namespace biped;
namespace dart_constr = dart::constraint;

const std::vector<std::string> Controller::joint_names = {
    "j_thigh_left",
    "j_shin_left",
    "j_heel_left",
    "j_toe_left",
    "j_thigh_right",
    "j_shin_right",
    "j_heel_right",
    "j_toe_right"
};

const std::vector<std::string> Controller::dof_names = {
    "j_thigh_left_x",
    "j_thigh_left_y",
    "j_thigh_left_z",
    "j_shin_left",
    "j_heel_left_1",
    "j_heel_left_2",
    "j_toe_left",
    "j_thigh_right_x",
    "j_thigh_right_y",
    "j_thigh_right_z",
    "j_shin_right",
    "j_heel_right_1",
    "j_heel_right_2",
    "j_toe_right"
};

Controller::Controller(SkeletonPtr biped, Joint::ActuatorType at) :
    _actuator_type(at)
{
    _biped = biped;
    int nDofs = _biped->getNumDofs();

    _forces = Eigen::VectorXd::Zero(nDofs);

    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

    for(std::size_t i = 0; i < 6; ++i)
    {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }

    for(std::size_t i = 6; i < _biped->getNumDofs(); ++i)
    {
        mKp(i, i) = 1000;
        mKd(i, i) = 50;
    }

    setTargetPositions(_biped->getPositions());
}


/// Add commanding forces from PD controllers
void Controller::addPDForces()
{
    Eigen::VectorXd q = _biped->getPositions();
    Eigen::VectorXd dq = _biped->getVelocities();

    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;

    _forces += p + d;
    _biped->setForces(_forces);
}

/// Add commanind forces from Stable-PD controllers
void Controller::addSPDForces()
{
    Eigen::VectorXd q = _biped->getPositions();
    Eigen::VectorXd dq = _biped->getVelocities();

    Eigen::MatrixXd invM = (_biped->getMassMatrix()
                            + mKd * _biped->getTimeStep()).inverse();
    Eigen::VectorXd p =
            -mKp * (q + dq * _biped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot =
            invM * (-_biped->getCoriolisAndGravityForces()
                    + p + d + _biped->getConstraintForces());

    _forces += p + d - mKd * qddot * _biped->getTimeStep();
    _biped->setForces(_forces);
}

/// add commanding forces from ankle strategy (Lesson 4 Answer)
void Controller::addAnkleStrategyForces()
{
    Eigen::Vector3d COM = _biped->getCOM();
    // Approximated center of pressure in sagittal axis
    Eigen::Vector3d offset(0.05, 0, 0);
    Eigen::Vector3d COP = _biped->getBodyNode("h_heel_left")->
            getTransform() * offset;
    double diff = COM[0] - COP[0];

    Eigen::Vector3d dCOM = _biped->getCOMLinearVelocity();
    Eigen::Vector3d dCOP =  _biped->getBodyNode("h_heel_left")->
            getLinearVelocity(offset);
    double dDiff = dCOM[0] - dCOP[0];

    int lHeelIndex = _biped->getDof("j_heel_left_1")->getIndexInSkeleton();
    int rHeelIndex = _biped->getDof("j_heel_right_1")->getIndexInSkeleton();
    int lToeIndex = _biped->getDof("j_toe_left")->getIndexInSkeleton();
    int rToeIndex = _biped->getDof("j_toe_right")->getIndexInSkeleton();
    if(diff < 0.1 && diff >= 0.0) {
        // Feedback rule for recovering forward push
        double k1 = 200.0;
        double k2 = 100.0;
        double kd = 10;
        _forces[lHeelIndex] += -k1 * diff - kd * dDiff;
        _forces[lToeIndex] += -k2 * diff - kd * dDiff;
        _forces[rHeelIndex] += -k1 * diff - kd * dDiff;
        _forces[rToeIndex] += -k2 * diff - kd * dDiff;
    }else if(diff > -0.2 && diff < -0.05) {
        // Feedback rule for recovering backward push
        double k1 = 2000.0;
        double k2 = 100.0;
        double kd = 100;
        _forces[lHeelIndex] += -k1 * diff - kd * dDiff;
        _forces[lToeIndex] += -k2 * diff - kd * dDiff;
        _forces[rHeelIndex] += -k1 * diff - kd * dDiff;
        _forces[rToeIndex] += -k2 * diff - kd * dDiff;
    }
    _biped->setForces(_forces);
}


// Set initial configuration (Lesson 2 Answer)
void Controller::setInitialPose(SkeletonPtr biped)
{
  biped->setPosition(biped->getDof("j_thigh_left_z")->
                     getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_thigh_right_z")->
                     getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_shin_left")->
                     getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->
                     getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_heel_left_1")->
                     getIndexInSkeleton(), 0.25);
  biped->setPosition(biped->getDof("j_heel_right_1")->
                     getIndexInSkeleton(), 0.25);
}

void Controller::setActuators(){
    //right leg
    Joint* r_thigh = _biped->getJoint("j_thigh_right");
    Joint* r_shin = _biped->getJoint("j_shin_right");
    Joint* r_heel = _biped->getJoint("j_heel_right");

    r_thigh->setActuatorType(_actuator_type);
    r_shin->setActuatorType(_actuator_type);
    r_heel->setActuatorType(_actuator_type);

    //left leg
    Joint* l_thigh = _biped->getJoint("j_thigh_left");
    Joint* l_shin = _biped->getJoint("j_shin_left");
    Joint* l_heel = _biped->getJoint("j_heel_left");

    l_thigh->setActuatorType(_actuator_type);
    l_shin->setActuatorType(_actuator_type);
    l_heel->setActuatorType(_actuator_type);
}

void Controller::setCommands(const std::string &joint_name, double value){
    int index = _biped->getDof(joint_name)->getIndexInSkeleton();
    _biped->setCommand(index, value);
}

Simulation::Simulation(Joint::ActuatorType at, const std::string &model_path)
    : _force_count_down(0),
      _positive_sign(true)
{
    //_osgViewer = nullptr;
    //_osgWorldNode = nullptr;


    SkeletonPtr biped = _loadBiped(model_path);
    SkeletonPtr floor = _createFloor();

    Eigen::VectorXd balancedPose = _solveIK(biped);
    biped->setPositions(balancedPose);

    _world = std::make_shared<World>();
    _world->setName("biped world");
    _world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));


    dart_constr::BoxedLcpSolverPtr lcp_solver(new dart_constr::DantzigBoxedLcpSolver);
    dart_constr::PgsBoxedLcpSolverPtr pgs_solver(new dart_constr::PgsBoxedLcpSolver);
    std::unique_ptr<dart_constr::ConstraintSolver> constr_solver(new dart_constr::BoxedLcpConstraintSolver(lcp_solver,pgs_solver));

    _world->setConstraintSolver(std::move(constr_solver));

    _world->getConstraintSolver()->setCollisionDetector(
                    dart::collision::DARTCollisionDetector::create());
    


    _world->addSkeleton(floor);
    _world->addSkeleton(biped);

    _controller = std::make_unique<Controller>(biped,at);

    _set_self_collision<Params>();
    if(_self_collision)
      std::cout << "SELF COLLISION ENABLED" << std::endl;
    else std::cout << "SELF COLLISION DISABLED" << std::endl;

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
    windows.front()->setWindowName("Biped Walking");

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
    _controller->clearForces();

    // if(_spd_enabled)
    //     _controller->addSPDForces();

    // if(_ankle_strat_enabled)
    //     _controller->addAnkleStrategyForces();



//    // Apply body forces based on user input, and color the body shape red
//    if(_force_count_down > 0)
//    {
//        BodyNode* bn = _world->getSkeleton("biped")->getBodyNode("h_pelvis");
//        auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
//        shapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

//        if(_positive_sign)
//            bn->addExtForce(default_force * Eigen::Vector3d::UnitX(),
//                            bn->getCOM(), false, false);
//        else
//            bn->addExtForce(-default_force*Eigen::Vector3d::UnitX(),
//                            bn->getCOM(), false, false);

//        --_force_count_down;
//    }
    _world->step();

    // If we have a viewer (visualization mode), update it
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
    _world->getSkeleton("biped")->resetPositions();
    _world->getSkeleton("biped")->resetVelocities();
    _world->getSkeleton("biped")->resetAccelerations();

}

// Load a biped model and enable joint limits and self-collision
SkeletonPtr Simulation::_loadBiped(const std::string& model_path)
{
  // Create the world with a skeleton
  WorldPtr world = SkelParser::readWorld(model_path);
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  // Set joint limits
  for(std::size_t i = 0; i < biped->getNumJoints(); ++i)
    biped->getJoint(i)->setPositionLimitEnforced(true);

  // Enable self collision check but ignore adjacent bodies
  if(_self_collision){
    biped->enableSelfCollisionCheck();
    biped->disableAdjacentBodyCheck();
  }

  return biped;
}


// Solve for a balanced pose using IK
Eigen::VectorXd Simulation::_solveIK(SkeletonPtr biped)
{
  // Modify the intial pose to one-foot stance before IK
  biped->setPosition(biped->getDof("j_shin_right")->
                     getIndexInSkeleton(), -1.4);
//  biped->setPosition(biped->getDof("j_bicep_left_x")->
//                     getIndexInSkeleton(), 0.8);
//  biped->setPosition(biped->getDof("j_bicep_right_x")->
//                     getIndexInSkeleton(), -0.8);

  Eigen::VectorXd newPose = biped->getPositions();
  BodyNodePtr leftHeel = biped->getBodyNode("h_heel_left");
  BodyNodePtr leftToe = biped->getBodyNode("h_toe_left");
  double initialHeight = -0.8;

  for(std::size_t i = 0; i < default_ik_iterations; ++i)
  {
    Eigen::Vector3d deviation = biped->getCOM() - leftHeel->getCOM();
    Eigen::Vector3d localCOM = leftHeel->getCOM(leftHeel);
    LinearJacobian jacobian = biped->getCOMLinearJacobian() -
        biped->getLinearJacobian(leftHeel, localCOM);

    // Sagittal deviation
    double error = deviation[0];
    Eigen::VectorXd gradient = jacobian.row(0);
    Eigen::VectorXd newDirection = -0.2 * error * gradient;

    // Lateral deviation
    error = deviation[2];
    gradient = jacobian.row(2);
    newDirection += -0.2 * error * gradient;

    // Position constraint on four (approximated) corners of the left foot
    Eigen::Vector3d offset(0.0, -0.04, -0.03);
    error = (leftHeel->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
    newDirection += -0.2 * error * gradient;

    offset[2] = 0.03;
    error = (leftHeel->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
    newDirection += -0.2 * error * gradient;

    offset[0] = 0.04;
    error = (leftToe->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftToe, offset).row(1);
    newDirection += -0.2 * error * gradient;

    offset[2] = -0.03;
    error = (leftToe->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftToe, offset).row(1);
    newDirection += -0.2 * error * gradient;

    newPose += newDirection;
    biped->setPositions(newPose);
    biped->computeForwardKinematics(true, false, false);
  }
  return newPose;
}

SkeletonPtr Simulation::_createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}
