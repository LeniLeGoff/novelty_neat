#ifndef LEGGED_PARAMETERS_HPP
#define LEGGED_PARAMETERS_HPP

#include <sferes/stc.hpp>
#include <sferes/gen/evo_float.hpp>
#include <nn2/gen_dnn.hpp>
#include <dart/dart.hpp>

namespace sf = sferes;
namespace dart_dyn = dart::dynamics;

namespace legged {


typedef enum feedback_type{
    POSITION,
    VELOCITIES,
    ACCELERATION,
    FORCES
}feedback_type;

struct Params{
    struct robot{
        static constexpr feedback_type fb_type = POSITION;
        static constexpr dart_dyn::Joint::ActuatorType actuator_type = dart_dyn::Joint::SERVO;
    };
    struct simu{
        static constexpr int nb_steps = 3000;
#if defined(THREE_LEGS_2DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/tripod_2dof.urdf");
#elif defined(THREE_LEGS_3DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/tripod_3dof.urdf");
#elif defined(THREE_LEGS_4DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/tripod_4dof.urdf");
#elif defined(FOuR_LEGS_2DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/quadpod_2dof.urdf");
#elif defined(FOUR_LEGS_3DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/quadpod_3dof.urdf");
#elif defined(FOUR_LEGS_4DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/quadpod_4dof.urdf");
#elif defined(SIX_LEGS_2DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/hexapod_2dof.urdf");
#elif defined(SIX_LEGS_4DOF)
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/hexapod_4dof.urdf");
#else
        SFERES_STRING(model_path,"/git/sferes2/exp/novelty_neat/legged_robot/tripod_3dof.urdf");
#endif
        static constexpr bool self_collision = true;
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
#if defined(THREE_LEGS_2DOF)
        static constexpr size_t nb_inputs	= 12;
        static constexpr size_t nb_outputs	= 6;
#elif defined(THREE_LEGS_3DOF)
        static constexpr size_t nb_inputs   = 15;
        static constexpr size_t nb_outputs  = 9;
#elif defined(THREE_LEGS_4DOF)
        static constexpr size_t nb_inputs   = 18;
        static constexpr size_t nb_outputs  = 12;
#elif defined(FOUR_LEGS_2DOF)
        static constexpr size_t nb_inputs   = 14;
        static constexpr size_t nb_outputs  = 8;
#elif defined(FOUR_LEGS_3DOF)
        static constexpr size_t nb_inputs   = 18;
        static constexpr size_t nb_outputs  = 12;
#elif defined(FOUR_LEGS_4DOF)
        static constexpr size_t nb_inputs   = 22;
        static constexpr size_t nb_outputs  = 16;
#elif defined(SIX_LEGS_2DOF)
        static constexpr size_t nb_inputs   = 18;
        static constexpr size_t nb_outputs  = 12;
#elif defined(SIX_LEGS_4DOF)
        static constexpr size_t nb_inputs   = 30;
        static constexpr size_t nb_outputs  = 24;
#else         
        static constexpr size_t nb_inputs   = 24;
        static constexpr size_t nb_outputs  = 18;
#endif

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
        SFERES_STRING(log_dir,"/data");
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
        static constexpr size_t size = 400;
        static constexpr size_t nb_gen = 4001;
        static constexpr float initial_aleat = 1.0f;
        static constexpr size_t dump_period = 50;
    };
    struct novelty {
        static constexpr int nb_pos = 2;
        static constexpr float rho_min_init = 1.0;
        static constexpr size_t k = 15;
        static constexpr size_t stalled_tresh = 2500;
        static constexpr size_t adding_tresh = 4;
        static constexpr float add_to_archive_prob = 0;
    };
};

}//legged
#endif //LEGGED_PARAMETERS_HPP
