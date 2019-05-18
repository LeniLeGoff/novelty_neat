/*
 * Created by L.K Le Goff
 */

#include <iostream>

#ifndef VISU
#include <sferes/eval/parallel.hpp>
#else
#include <sferes/eval/eval.hpp>
#endif

#include <sferes/fit/fitness.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/phen/parameters.hpp>
//#include <sferes/modif/novelty.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <sferes/ea/nsga2.hpp>
#include <sferes/stat/pareto_front.hpp>

#include <nn2/gen_dnn_ff.hpp>
#include <nn2/phen_dnn.hpp>

#include "parameters.hpp"
#include "fit_maze_navigation.hpp"
#include "stat_bd.hpp"
#include "stat_success.hpp"
//#include "stat_traj.hpp"

#include "phen_static_nn.hpp"
#include "gen_static_nn.hpp"
#include "phen_static_rnn.hpp"
#include "gen_static_rnn.hpp"
#include "modifier_novelty.hpp"

//namespace sferes {

//template<>
//int FitMazeNavigation<Params>::success1_so_far = 0;

//template<>
//int FitMazeNavigation<Params>::success2_so_far = 0;

//template<>
//int FitMazeNavigation<Params>::success3_so_far = 0;
//}

namespace sf = sferes;
namespace fs = fastsim;

int main(int argc, char** argv){
    typedef sf::phen::Parameters<sf::gen::EvoFloat<1, Params>, sf::fit::FitDummy<>, Params> weight_t;
    typedef sf::phen::Parameters<sf::gen::EvoFloat<1, Params>, sf::fit::FitDummy<>, Params> bias_t;
    typedef sf::phen::Parameters<sf::gen::EvoFloat<4, Params>, sf::fit::FitDummy<>, Params> node_label_t;

    typedef nn::PfWSum<weight_t> pf_t;
    typedef nn::AfSigmoidBias<bias_t> af_t;
    typedef nn::Neuron<pf_t, af_t> neuron_t;
    typedef nn::Connection<weight_t> connection_t;

#ifdef NEAT
    typedef sf::gen::Dnn<neuron_t, connection_t, Params> gen_t;
    typedef sf::phen::Dnn<gen_t, FitMazeNavigation<Params>, Params> phen_t;
#elif RNN
    typedef sf::gen::StaticRNN<neuron_t, connection_t, Params> gen_t;
    typedef sf::phen::StaticRNN<gen_t, FitMazeNavigation<Params>, Params> phen_t;
#else
    typedef sf::gen::StaticNN<neuron_t, connection_t, Params> gen_t;
    typedef sf::phen::StaticNN<gen_t, FitMazeNavigation<Params>, Params> phen_t;
#endif

#ifndef VISU
    typedef sf::eval::Parallel<Params> eval_t;
#else
    typedef sf::eval::Eval<Params> eval_t;
#endif

#ifdef NOVELTY
    typedef sf::modif::BehaviorNov<Params> modifier_t;
#else
    typedef sf::modif::Dummy<Params> modifier_t;
#endif
    // STATS
    typedef boost::fusion::vector<
            sf::stat::ParetoFront<phen_t, Params>,
            sf::stat::BD<phen_t, Params>,
            sf::stat::Success<phen_t, Params>
            //      sf::stat::Traj<phen_t, Params>
            >  stat_t;

    typedef sf::ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;


    ea_t ea;
    sf::run_ea(argc,argv,ea);
}
