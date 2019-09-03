#ifndef FIT_LEGGED_ROBOT_HPP
#define FIT_LEGGED_ROBOT_HPP

#include <sferes/fit/fitness.hpp>
#include <Eigen/Core>

// #include "parameters.hpp"
#include "legged_robot.hpp"

namespace sf = sferes;

SFERES_FITNESS(FitLeggedRobot,sf::fit::Fitness){
    public:
    template<typename Indiv>
    void eval(Indiv& ind){

        ind.nn().simplify();

        this->_objs.resize(1);

        legged::Simulation simu(legged::Params::robot::actuator_type,legged::Params::simu::model_path());
        // _simu.reset();

#ifdef VISU
        std::cout << "Initiatlization of the visualisation" << std::endl;
        simu.init_visu();
#endif


        for(int i = 0; i < legged::Params::simu::nb_steps && !stop_eval;){

//            std::cout << "eval ... step " << i << std::endl;

            get_inputs(simu);

            if(pos_bd.size() < legged::Params::novelty::nb_pos
                    && i > 0
                    && (i%(int)std::round(legged::Params::simu::nb_steps/legged::Params::novelty::nb_pos)==0)){
                        if(!std::isnan(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3)(0)))
                            pos_bd.push_back(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3));
                        else pos_bd.push_back(Eigen::VectorXd::Zero(3));
                    }

            step_check(ind.nn(),simu);

            move_check(simu);

            simu.update(i);

            i++;
        }
        if(!std::isnan(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3)(0)))
            pos_bd.push_back(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3));
        else pos_bd.push_back(Eigen::VectorXd::Zero(3));
        // std::cout << "POS BD :" << std::endl;
        // for(auto pos : pos_bd)
        //     std::cout << pos << std::endl;
    }

    template<typename Indiv>
    float dist(const Indiv& ind) {
        double dist = 0;
        double delta;
        for(size_t i = 0; i < pos_bd.size(); i++){
            if(ind.fit().pos_bd[i].rows() < pos_bd[i].rows())
                return 0;
            for(int j = 0; j < pos_bd[i].rows(); j++){

                delta = pos_bd[i](j) - ind.fit().pos_bd[i](j);
                dist += delta*delta;
            }
        }
        return sqrt(dist);
    }

    //TODO functions to convert [lower,upper] <-> [0,1]

    void get_inputs(legged::Simulation& simu){
        Eigen::VectorXd feedback;
        if(legged::Params::robot::fb_type == legged::feedback_type::POSITION)
            feedback = simu._controller->get_model()->getPositions();
        else if(legged::Params::robot::fb_type == legged::feedback_type::VELOCITIES)
            feedback = simu._controller->get_model()->getVelocities();
        else if(legged::Params::robot::fb_type == legged::feedback_type::ACCELERATION)
            feedback = simu._controller->get_model()->getAccelerations();
        else if(legged::Params::robot::fb_type == legged::feedback_type::FORCES)
            feedback = simu._controller->get_model()->getForces();

        inputs.resize(feedback.rows());
        Eigen::VectorXd lower_limits;
        Eigen::VectorXd upper_limits;
        double lower, upper;
        int k = 0;
        for(int i = 0; i < inputs.size();){
            lower_limits = simu._controller->get_model()->getJoint(k)->getPositionLowerLimits();
            upper_limits = simu._controller->get_model()->getJoint(k)->getPositionUpperLimits();
            for(int j = 0; j < lower_limits.rows(); j++){
                lower = lower_limits(j);
                upper = upper_limits(j);
                if(std::isinf(lower_limits(j)))
                    lower = -M_PI;
                if(std::isinf(upper_limits(j)))
                    upper = M_PI;
                inputs[i] = (feedback(i) - lower)/(upper - lower);
               // std::cout << "feedback : " << feedback(i) << " ";
               // std::cout << "input : " << inputs[i] << " ";
               // std::cout << "limits : lower " << lower_limits(j) << " upper " << upper_limits(j) << std::endl;

                i++;
            }
            k++;
        }
    }

    template<typename NN>
    void step_check(NN &nn, legged::Simulation& simu){
        nn.step(inputs);
        int index;
        Eigen::VectorXd lower_limits;
        Eigen::VectorXd upper_limits;
        double lower, upper;
        outf.resize(nn.get_outf().size());
        int i = 0;
        for(const std::string &dofname : simu._controller->get_dofs_names()){
            lower = simu._controller->get_model()->getDof(dofname)->getVelocityLowerLimit();
            upper = simu._controller->get_model()->getDof(dofname)->getVelocityUpperLimit();
            // std::cout << "upper :" << upper << " lower : " <<  lower << std::endl;
            if(std::isinf(lower))
                lower = -7;
            if(std::isinf(upper))
                upper = 7;
            outf[i] = nn.get_outf()[i]*(upper - lower) + lower;
            i++;
        }
        // for(double val : outf)
        //    std::cout << val << " ";
        // std::cout << std::endl;
    }

    void move_check(legged::Simulation& simu){
        simu._controller->setCommands(outf);        
    }

    std::vector<Eigen::VectorXd> pos_bd; //behavior descriptor base on the 3D position of the pelvis.

    bool stop_eval = false;

    std::vector<float> outf, inputs;
};

#endif //FIT_LEGGED_ROBOT_HPP
