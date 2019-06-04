#ifndef FIT_MAZE_NAVIGATION_HPP
#define FIT_MAZE_NAVIGATION_HPP

#include <fastsim/simu_fastsim.hpp>

#include "parameters.hpp"

namespace sf = sferes;
namespace fs = fastsim;

typedef sf::simu::Fastsim<Params> simu_t;

//namespace sferes {


SFERES_FITNESS(FitMazeNavigation,sf::fit::Fitness){
    public:

    FitMazeNavigation() :
        nb_coll(0), time(0), speed(0), lin_speed(0), success1(0), success3(0),stop_eval(false) { }


    template<typename Indiv>
    void eval(Indiv& ind){
        ind.nn().simplify();

        nb_coll=0;
        speed=0;
        lin_speed=0;
        stop_eval=false;
#ifdef VERBOSE
        std::cout<<"Eval ..."<<std::endl;
#endif


        simu_t simu;
        assert(simu.map()!=NULL);

        // init

        init_simu(simu);
        ind.nn().init();

        //#ifdef SAVETRAJ
        //        std::ostringstream straj;
        //        straj<<"# map size "<<simu.map()->get_real_w()<<" "<<simu.map()->get_real_h()<<std::endl;
        //        straj<<"# "<<Params::simu::map_name()<<std::endl;
        //#endif


        time=0;

        success1=0;
        success2=0;
        success3=0;
        size_t i;
        // *** Main Loop ***
        for (i = 0; i < Params::simu::nb_steps && !stop_eval;)
        {

            // Number of steps the robot is evaluated
            time++;

            // Update robot info & caracs
            simu.refresh();
#ifdef VISU
            if (1) {
#elif defined(NO_VISU)
            if (0) {
#else
            if (this->mode() == sf::fit::mode::view){
#endif
                simu.refresh_view();
            }


            // Get inputs
            get_inputs(simu);

            // Step  neural network -- outf is the output vector.
            step_check(ind.nn());

            // move the robot and check for collision and if is still
            move_check(simu);

            //#ifdef SAVETRAJ
            //            straj<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<" "<<simu.robot().get_pos().theta()<<std::endl;
            //#endif


            if ((i>0)&&(i%(int)(Params::simu::nb_steps/Params::novelty::nb_pos)==0))
                pos_bd.push_back(simu.robot().get_pos());

            // loop forever if we are in the visualization mode
            if (this->mode() != sf::fit::mode::view)
                i++;

        }

        for (unsigned int j=pos_bd.size();j<Params::novelty::nb_pos;j++)
            pos_bd.push_back(simu.robot().get_pos());


        end_pos=simu.robot().get_pos();



#ifndef NOEXIT
        if ((simu.robot().get_pos().get_x()>=simu.map()->get_real_w()*Params::fitness::min_x1)
                &&(simu.robot().get_pos().get_x()<=simu.map()->get_real_w()*Params::fitness::max_x1)
                &&(simu.robot().get_pos().get_y()>=simu.map()->get_real_h()*Params::fitness::min_y1)
                &&(simu.robot().get_pos().get_y()<=simu.map()->get_real_h()*Params::fitness::max_y1)) {
            //std::cout<<"The robot has found the goal:"<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<std::endl;
            success1=1;
//            success1_so_far+=1; // WARNING: not significant if individuals are reevaluated (which is the case for NSGA-2, for instance)
        }
        else {
            //std::cout<<"Goal not found"<<std::endl;
        }
        if ((simu.robot().get_pos().get_x()>=simu.map()->get_real_w()*Params::fitness::min_x2)
                &&(simu.robot().get_pos().get_x()<=simu.map()->get_real_w()*Params::fitness::max_x2)
                &&(simu.robot().get_pos().get_y()>=simu.map()->get_real_h()*Params::fitness::min_y2)
                &&(simu.robot().get_pos().get_y()<=simu.map()->get_real_h()*Params::fitness::max_y2)) {
            //std::cout<<"The robot has found the goal:"<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<std::endl;
            success2=1;
//            success2_so_far+=1; // WARNING: not significant if individuals are reevaluated (which is the case for NSGA-2, for instance)
        }
        else {
            //std::cout<<"Goal not found"<<std::endl;
        }
        if ((simu.robot().get_pos().get_x()>=simu.map()->get_real_w()*Params::fitness::min_x3)
                &&(simu.robot().get_pos().get_x()<=simu.map()->get_real_w()*Params::fitness::max_x3)
                &&(simu.robot().get_pos().get_y()>=simu.map()->get_real_h()*Params::fitness::min_y3)
                &&(simu.robot().get_pos().get_y()<=simu.map()->get_real_h()*Params::fitness::max_y3)) {
            //std::cout<<"The robot has found the goal:"<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<std::endl;
            success3=1;
//            success3_so_far+=1;  // WARNING: not significant if individuals are reevaluated (which is the case for NSGA-2, for instance)
        }
        else {
            //std::cout<<"Goal not found"<<std::endl;
        }
#endif



        //std::cout<<"End_pos | "<<end_pos.get_x()<<" "<<end_pos.get_y()<<" | "<<end_pos.get_x()/simu.map()->get_real_w()<<" "<<end_pos.get_y()/simu.map()->get_real_h()<<std::endl;


#ifdef VERBOSE
        static int nbeval=0;
        std::cout<<"fit="<<this->_objs[0]<<" nbeval="<<nbeval<<std::endl;
        nbeval++;
#endif

        //#ifdef SAVETRAJ
        //        traj=straj.str();
        //#endif
    }
//    template<typename Indiv>
//    float dist(Indiv& ind){
//        delta=pos_bd[k].get_x()-_pop[j]->fit().pos_bd[k].get_x();
//        delta*=delta;
//        hd+=delta;
//        delta=_pop[i]->fit().pos_bd[k].get_y()-_pop[j]->fit().pos_bd[k].get_y();
//        delta*=delta;
//        hd+=delta;
//    }

    template<typename Simu>
    void init_simu(Simu& simu)
    {
        this->_objs.resize(1); //pure novelty

        //Visualisation mode
#ifdef VISU
        simu.init_view(true);
#elif !defined(NO_VISU)
        if(this->mode() == sf::fit::mode::view)
            simu.init_view(true);
#endif

        simu.init();

        // Adding robot sensors (no need to add effectors):
        // 3 lasers range sensors
        //right
        simu.robot().add_laser(fs::Laser(M_PI / 4.0, 8.f*simu.robot().get_radius()*2.f));
        // left
        simu.robot().add_laser(fs::Laser(-M_PI / 4.0, 8.f*simu.robot().get_radius()*2.f));
        //middle
        simu.robot().add_laser(fs::Laser(0.0f, 8.f*simu.robot().get_radius()*2.f));

        old_pos=simu.robot().get_pos();
        inputs.resize(Params::dnn::nb_inputs);

        std::cout << "Map size : " << simu.map()->get_real_w() << std::endl;

        //#ifdef TOWARDSCORNER
        //      simu.robot().set_pos(fs::Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.1, -3.*M_PI/4.0));
        //#elif TURNED
        //      simu.robot().set_pos(fs::Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.1, -M_PI/4.0));
        //#elif LOWERLEFT
        //      simu.robot().set_pos(fs::Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.9, -M_PI/2.0));
        ////#else
        simu.robot().set_pos(fs::Posture(Params::simu::init_pos_x, Params::simu::init_pos_y, Params::simu::init_pos_theta));
        //#endif
        simu.robot().move(0,0,simu.map());

    }

    // *** Get sensors inputs
    template<typename Simu>
    void get_inputs(Simu &simu)
    {
        // Update of the sensors
        size_t nb_lasers = simu.robot().get_lasers().size();

        // *** set inputs ***

        // inputs from sensors
        for (size_t j = 0; j < nb_lasers; ++j)
        {
            float d = simu.robot().get_lasers()[j].get_dist();
            float range = simu.robot().get_lasers()[j].get_range();
            inputs[j] = (d == -1 ? 0 : 1 - d / range); //Normalized value
        }

        inputs[nb_lasers]=1;

    }


    // *** Step Neural Network and various checks
    template<typename NN>
    void step_check(NN &nn)
    {
        nn.step(inputs);
        outf.resize(nn.get_outf().size());
        assert(nn.get_outf().size() == 2);

        for(size_t j = 0; j < nn.get_outf().size(); j++)
            if(std::isnan(nn.get_outf()[j]))
                outf[j] = 0.0;
            else
                outf[j]=4*(2*nn.get_outf()[j]-1); // to put nn values in the interval [-4;4] instead of [0;1]

        //std::cout<<"Outf: "<<nn.get_outf()[0]<<" "<<nn.get_outf()[1]<<std::endl;

    }

    // *** Move and check if robot is colliding, or still
    template<typename Simu>
    void move_check(Simu &simu)
    {
        // *** move robot ***
        simu.move_robot(outf[0], outf[1]);

        // *** To save simulation time, we stop evaluation if the robot is stuck for more than 100 time steps ***
        if ((old_pos.dist_to(simu.robot().get_pos())<0.0001)&&
                (fabs(old_pos.theta()-simu.robot().get_pos().theta())<0.0001)) {
            stand_still++;
            if (stand_still>100) {
                stop_eval=true;
#ifdef VERBOSE
                std::cout<<"Still robot, we stop the eval..."<<std::endl;
#endif
                // We add collisions to be fair and avoid side effects
                if (simu.robot().get_collision())
                    nb_coll+=Params::simu::nb_steps-time;
            }
        }
        else {
            if (simu.robot().get_collision()) {
                nb_coll++;
            }
        }

        old_pos=simu.robot().get_pos();
    }


    float width, height, fit;
    int nb_coll, time;
    float speed, lin_speed;
    unsigned int stand_still;
    fs::Posture old_pos,end_pos;

    std::vector<fs::Posture> pos_bd; // behavior descriptor based on the position


//    static int success1_so_far;
//    static int success2_so_far;
//    static int success3_so_far;

    int success1, success2, success3;
//    int get_success1_so_far(void) {return success1_so_far;}
//    int get_success2_so_far(void) {return success2_so_far;}
//    int get_success3_so_far(void) {return success3_so_far;}
    bool stop_eval;                                  // Stops the evaluation
    std::vector<float> outf, inputs;

//    std::string traj;


};


//} //sferes
#endif //FIT_MAZE_NAVIGATION_HPP
