#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <sferes/stc.hpp>

namespace sf = sferes;

struct Params{
    struct simu {
      SFERES_STRING(map_name, "../../../modules/fastsim/maze.pbm");
      static constexpr float dt = 0.01;
      static constexpr int nb_steps = 1000;
    };
    struct evo_float {
        static constexpr float mutation_rate = 0.1f;
        static constexpr float cross_rate = 0.1f;
        static constexpr sf::gen::evo_float::mutation_t mutation_type = sf::gen::evo_float::polynomial;
        static constexpr sf::gen::evo_float::cross_over_t cross_over_type = sf::gen::evo_float::sbx;
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

        static constexpr float m_rate_add_conn	= 0.1f;
        static constexpr float m_rate_del_conn	= 0.01f;
        static constexpr float m_rate_change_conn = 0.1f;
        static constexpr float m_rate_add_neuron  = 0.1f;
        static constexpr float m_rate_del_neuron  = 0.01f;

        static constexpr int io_param_evolving = true;
        static constexpr sf::gen::dnn::init_t init = sf::gen::dnn::ff;
    };
    struct static_nn{
        static constexpr size_t nb_hidden = 5;
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
        static constexpr float max_y3=0.25;
    };
};

#endif //PARAMETERS_HPP
