#ifndef BIPED_PARAMETERS_HPP
#define BIPED_PARAMETERS_HPP

#include <sferes/stc.hpp>
#include <dart/dart.hpp>

namespace sf = sferes;
namespace dart_dyn = dart::dynamics;

namespace biped {


typedef enum feedback_type{
    POSITION,
    VELOCITIES,
    ACCELERATION,
    FORCES
}feedback_type;

struct Params{
    struct biped{
        static constexpr feedback_type fb_type = POSITION;
        static constexpr dart_dyn::Joint::ActuatorType actuactor_type = dart_dyn::Joint::VELOCITY;
    };
    struct simu{
        static constexpr int nb_steps = 5000;
        SFERES_STRING(model_path,"/home/leni/git/novelty_neat/biped/biped.skel");
    };
    struct evo_float {
        static constexpr float mutation_rate = 0.1f;
        static constexpr float cross_rate = 0.0f;
        static constexpr sf::gen::evo_float::mutation_t mutation_type = sf::gen::evo_float::polynomial;
        static constexpr sf::gen::evo_float::cross_over_t cross_over_type = sf::gen::evo_float::no_cross_over;
        static constexpr float eta_m = 15.0f;
        static constexpr float eta_c = 15.0f;
    };
    struct dnn {
        static constexpr size_t nb_inputs	= 20;
        static constexpr size_t nb_outputs	= 14;
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
    struct parameters {
        // maximum value of parameters
        static constexpr float min = -5.0f;
        // minimum value
        static constexpr float max = 5.0f;
    };
    struct pop
    {
        // size of a batch
        static constexpr size_t size = 4000;
        static constexpr size_t nb_gen = 10001;
        static constexpr float initial_aleat = 1.0f;
        static constexpr size_t dump_period = 50;
    };
    struct novelty {
        static constexpr int nb_pos = 2;
        static constexpr float rho_min_init = 1.0;
        static constexpr size_t k = 8;
        static constexpr size_t stalled_tresh = 2500;
        static constexpr size_t adding_tresh = 4;
        static constexpr float add_to_archive_prob = 0;
    };
};

}//biped
#endif //BIPED_PARAMETERS_HPP
