#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <sferes/stc.hpp>
#include <nn2/gen_dnn_ff.hpp>
#include <sferes/gen/evo_float.hpp>

namespace sf = sferes;

struct Params{
    struct simu {
      static constexpr float dt = 0.01;
      static constexpr int nb_steps = 1000;
//      SFERES_STRING(map_name, "/home/le_goff/git/sferes2/modules/fastsim/maze.pbm");
//      static constexpr float init_pos_x = 40;
//      static constexpr float init_pos_y = 360;
//      static constexpr float init_pos_theta = M_PI/4.0;


//      SFERES_STRING(map_name, "/home/leni/git/novelty_neat/multi.pbm");
//      static constexpr float init_pos_x = 300;
//      static constexpr float init_pos_y = 560;
//      static constexpr float init_pos_theta = M_PI/4.0;

     /* SFERES_STRING(map_name, "/home/le_goff/git/novelty_neat/multi2.pbm");
      static constexpr float init_pos_x = 300;
      static constexpr float init_pos_y = 540;
      static constexpr float init_pos_theta = M_PI/4.0;*/


/*      SFERES_STRING(map_name, "/home/le_goff/git/novelty_neat/subset.pbm");
      static constexpr float init_pos_x = 300;
      static constexpr float init_pos_y = 215;
      static constexpr float init_pos_theta = M_PI/4.0;
*/
      /*      SFERES_STRING(map_name, "/home/le_goff/git/novelty_neat/subset2.pbm");
            static constexpr float init_pos_x = 300;
            static constexpr float init_pos_y = 215;
            static constexpr float init_pos_theta = M_PI/4.0;*/
      SFERES_STRING(map_name, "/home/le_goff/git/novelty_neat/zigzag.pbm");
      static constexpr float init_pos_x = 60;
      static constexpr float init_pos_y = 570;
      static constexpr float init_pos_theta = M_PI/4.0;
    };
    struct evo_float {
        static constexpr float mutation_rate = 0.1f;
        static constexpr float cross_rate = 0.0f;
        static constexpr sf::gen::evo_float::mutation_t mutation_type = sf::gen::evo_float::polynomial;
        static constexpr sf::gen::evo_float::cross_over_t cross_over_type = sf::gen::evo_float::no_cross_over;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 15.0f;
    };
    struct parameters {
        // maximum value of parameters
        static constexpr float min = -5.0f;
        // minimum value
        static constexpr float max = 5.0f;
    };
    struct dnn {
        static constexpr size_t nb_inputs	= 4;
        static constexpr size_t nb_outputs	= 2;
        static constexpr size_t min_nb_neurons	= 0;
        static constexpr size_t max_nb_neurons	= 30;
        static constexpr size_t min_nb_conns	= 8;
        static constexpr size_t max_nb_conns	= 250;

        static float m_rate_add_conn;
        static float m_rate_del_conn;
        static float m_rate_change_conn;
        static float m_rate_add_neuron;
        static float m_rate_del_neuron;

        static constexpr int io_param_evolving = true;
        static constexpr sf::gen::dnn::init_t init = sf::gen::dnn::ff;
    };
    struct static_nn{
#if defined(NB_HIDDEN_0)
        static constexpr size_t nb_hidden = 0;
#elif defined(NB_HIDDEN_2)
        static constexpr size_t nb_hidden = 2;
#elif defined(NB_HIDDEN_4)
        static constexpr size_t nb_hidden = 4;
#elif defined(NB_HIDDEN_8)
        static constexpr size_t nb_hidden = 8;
#elif defined(NB_HIDDEN_16)
        static constexpr size_t nb_hidden = 16;
#else
        static constexpr size_t nb_hidden = 5;
#endif
    };
    struct ea
    {
        /*SFERES_CONST size_t res_x = 256;
      SFERES_CONST size_t res_y = 256;*/

        static constexpr size_t behav_dim = 2;
        SFERES_ARRAY(size_t, behav_shape, 256, 256);

    };
    struct pop
    {
        // size of a batch
        static constexpr size_t size = 400;

        static constexpr size_t nb_gen = 10001;
        static constexpr float initial_aleat = 2.0f;
        static constexpr size_t dump_period = 50;
    };
    struct novelty {
        static constexpr float rho_min_init = 1.0;
        static constexpr size_t k = 15;
//        static constexpr size_t stalled_tresh = 2500;
//        static constexpr size_t adding_tresh = 4;
        static constexpr unsigned int max_archive_size = 50000; //Max archive size

//        static constexpr float add_to_archive_prob = 0;
        static constexpr int nb_pos = 2 ;
    };
    struct fitness{
        /*hard maze
        static constexpr float min_x1=0.85;
        static constexpr float max_x1=0.95;
        static constexpr float min_y1=0.85;
        static constexpr float max_y1=0.95;

        static constexpr float min_x2=0.85;
        static constexpr float max_x2=0.95;
        static constexpr float min_y2=0.15;
        static constexpr float max_y2=0.25;

        static constexpr float min_x3=0.15;
        static constexpr float max_x3=0.25;
        static constexpr float min_y3=0.15;
        static constexpr float max_y3=0.25;*/


        /*multi
        static constexpr float min_x1=0.45;
        static constexpr float max_x1=0.55;
        static constexpr float min_y1=0.;
        static constexpr float max_y1=0.1;

        static constexpr float min_x2=0.85;
        static constexpr float max_x2=0.95;
        static constexpr float min_y2=0.7;
        static constexpr float max_y2=0.8;

        static constexpr float min_x3=0.1;
        static constexpr float max_x3=0.2;
        static constexpr float min_y3=0.1;
        static constexpr float max_y3=0.2;*/

        //multi2
/*        static constexpr float min_x1=0.45;
        static constexpr float max_x1=0.55;
        static constexpr float min_y1=0.5;
        static constexpr float max_y1=0.6;

        static constexpr float min_x2=0.45;
        static constexpr float max_x2=0.55;
        static constexpr float min_y2=0.8;
        static constexpr float max_y2=0.9;

        static constexpr float min_x3=0.35;
        static constexpr float max_x3=0.45;
        static constexpr float min_y3=0.35;
        static constexpr float max_y3=0.45;
*/
        //subset
        /*static constexpr float min_x1=1.;
        static constexpr float max_x1=0.9;
        static constexpr float min_y1=0.23;
        static constexpr float max_y1=0.33;

        static constexpr float min_x2=0.1;
        static constexpr float max_x2=0.2;
        static constexpr float min_y2=0.1;
        static constexpr float max_y2=0.2;

        static constexpr float min_x3=0.35;
        static constexpr float max_x3=0.45;
        static constexpr float min_y3=0.95;
        static constexpr float max_y3=0.85;*/


        //zigzag
        static constexpr float min_x1=0.;
        static constexpr float max_x1=0.1;
        static constexpr float min_y1=0.9;
        static constexpr float max_y1=1.;

        static constexpr float min_x2=0.45;
        static constexpr float max_x2=0.55;
        static constexpr float min_y2=0.;
        static constexpr float max_y2=0.1;

        static constexpr float min_x3=0.9;
        static constexpr float max_x3=1.;
        static constexpr float min_y3=0.9;
        static constexpr float max_y3=1.;
    };
};


#endif //PARAMETERS_HPP
