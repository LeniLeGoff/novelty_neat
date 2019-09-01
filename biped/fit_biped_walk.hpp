#ifndef FIT_BIPED_WALK_HPP
#define FIT_BIPED_WALK_HPP

#include <sferes/fit/fitness.hpp>
#include <Eigen/Core>

// #include "parameters.hpp"
#include "biped.hpp"

namespace sf = sferes;

SFERES_FITNESS(FitBipedWalk,sf::fit::Fitness){
    public:
    FitBipedWalk(){
        // _simu.reset(new biped::Simulation(biped::Params::biped::actuator_type,biped::Params::simu::model_path()));
    }

    template<typename Indiv>
    void eval(Indiv& ind){

        ind.nn().simplify();

        this->_objs.resize(1);

        biped::Simulation simu(biped::Params::biped::actuator_type,biped::Params::simu::model_path());
        // _simu.reset();

#ifdef VISU
        std::cout << "Initiatlization of the visualisation" << std::endl;
        simu.init_visu();
#endif


        for(int i = 0; i < biped::Params::simu::nb_steps && !stop_eval;){

//            std::cout << "eval ... step " << i << std::endl;

            get_inputs(simu);

            if(pos_bd.size() < biped::Params::novelty::nb_pos
                    && i > 0
                    && (i%(int)std::round(biped::Params::simu::nb_steps/biped::Params::novelty::nb_pos)==0)){
                        if(!std::isnan(simu._controller->get_biped()->getBodyNode("h_pelvis")->getWorldTransform().matrix().col(3)(0)))
                            pos_bd.push_back(simu._controller->get_biped()->getBodyNode("h_pelvis")->getWorldTransform().matrix().col(3));
                        else pos_bd.push_back(Eigen::VectorXd::Zero(3));
                    }

            step_check(ind.nn(),simu);

            move_check(simu);

            simu.update(i);

            i++;
        }
        if(!std::isnan(simu._controller->get_biped()->getBodyNode("h_pelvis")->getWorldTransform().matrix().col(3)(0)))
            pos_bd.push_back(simu._controller->get_biped()->getBodyNode("h_pelvis")->getWorldTransform().matrix().col(3));
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

    void get_inputs(biped::Simulation& simu){
        Eigen::VectorXd feedback;
        if(biped::Params::biped::fb_type == biped::feedback_type::POSITION)
            feedback = simu._controller->get_biped()->getPositions();
        else if(biped::Params::biped::fb_type == biped::feedback_type::VELOCITIES)
            feedback = simu._controller->get_biped()->getVelocities();
        else if(biped::Params::biped::fb_type == biped::feedback_type::ACCELERATION)
            feedback = simu._controller->get_biped()->getAccelerations();
        else if(biped::Params::biped::fb_type == biped::feedback_type::FORCES)
            feedback = simu._controller->get_biped()->getForces();

        inputs.resize(feedback.rows());
        Eigen::VectorXd lower_limits;
        Eigen::VectorXd upper_limits;
        double lower, upper;
        int k = 0;
        for(int i = 0; i < inputs.size();){
            lower_limits = simu._controller->get_biped()->getJoint(k)->getPositionLowerLimits();
            upper_limits = simu._controller->get_biped()->getJoint(k)->getPositionUpperLimits();
            for(int j = 0; j < lower_limits.rows(); j++){
                lower = lower_limits(j);
                upper = upper_limits(j);
                if(std::isinf(lower_limits(j)))
                    lower = -M_PI;
                if(std::isinf(upper_limits(j)))
                    upper = M_PI;
                inputs[i] = (feedback(i) - lower)/(upper - lower);
//                std::cout << "feedback : " << feedback(i) << " ";
//                std::cout << "input : " << inputs[i] << " ";
//                std::cout << "limits : lower " << lower_limits(j) << " upper " << upper_limits(j) << std::endl;

                i++;
            }
            k++;
        }
    }

    template<typename NN>
    void step_check(NN &nn, biped::Simulation& simu){
        nn.step(inputs);
        int index;
        Eigen::VectorXd lower_limits;
        Eigen::VectorXd upper_limits;
        double lower, upper;
        outf.resize(nn.get_outf().size());
        int i = 0;
        for(const std::string &jname : biped::Controller::joint_names){
            lower_limits = simu._controller->get_biped()->getJoint(jname)->getVelocityLowerLimits();
            upper_limits = simu._controller->get_biped()->getJoint(jname)->getVelocityUpperLimits();
            for(int j = 0; j < lower_limits.rows(); j++){
                lower = lower_limits(j);
                upper = upper_limits(j);
                if(std::isinf(lower_limits(j)))
                    lower = -1;
                if(std::isinf(upper_limits(j)))
                    upper = 1;
                outf[i] = nn.get_outf()[i]*(upper - lower) + lower;
                i++;
            }
        }
//        for(double val : outf)
//            std::cout << val << " ";
//        std::cout << std::endl;
    }

    void move_check(biped::Simulation& simu){
        for(int i = 0; i < outf.size(); i++){
            simu._controller->setCommands(biped::Controller::dof_names[i],outf[i]);
        }
    }

    std::vector<Eigen::VectorXd> pos_bd; //behavior descriptor base on the 3D position of the pelvis.

    bool stop_eval = false;

    // std::shared_ptr<biped::Simulation> _simu;
    std::vector<float> outf, inputs;
};

#endif //FIT_BIPED_WALK_HPP
