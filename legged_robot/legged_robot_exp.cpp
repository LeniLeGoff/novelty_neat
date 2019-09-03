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
#include <sferes/modif/novelty.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <sferes/ea/nsga2.hpp>
#include <sferes/stat/pareto_front.hpp>

#include <nn2/gen_dnn_ff.hpp>
#include <nn2/phen_dnn.hpp>

#include "fit_legged_robot.hpp"
#include "parameters.hpp"
#include "../phen_static_nn.hpp"
#include "../phen_static_rnn.hpp"
#include "stat_bd.hpp"


namespace sf = sferes;

int main(int argc, char** argv){
    typedef sf::phen::Parameters<sf::gen::EvoFloat<1, legged::Params>, sf::fit::FitDummy<>, legged::Params> weight_t;
    typedef sf::phen::Parameters<sf::gen::EvoFloat<1, legged::Params>, sf::fit::FitDummy<>, legged::Params> bias_t;
    typedef sf::phen::Parameters<sf::gen::EvoFloat<4, legged::Params>, sf::fit::FitDummy<>, legged::Params> node_label_t;

    typedef nn::PfWSum<weight_t> pf_t;
    typedef nn::AfSigmoidBias<bias_t> af_t;
    typedef nn::Neuron<pf_t, af_t> neuron_t;
    typedef nn::Connection<weight_t> connection_t;


#ifdef NEAT
    typedef sf::gen::Dnn<neuron_t, connection_t, legged::Params> gen_t;
    typedef sf::phen::Dnn<gen_t, FitLeggedRobot<legged::Params>, legged::Params> phen_t;
#elif RNN
    typedef sf::gen::StaticRNN<neuron_t, connection_t, legged::Params> gen_t;
    typedef sf::phen::StaticRNN<gen_t, FitLeggedRobot<legged::Params>, legged::Params> phen_t;
#else
    typedef sf::gen::StaticNN<neuron_t, connection_t, legged::Params> gen_t;
    typedef sf::phen::StaticNN<gen_t, FitLeggedRobot<legged::Params>, legged::Params> phen_t;
#endif

#ifndef VISU
   typedef sf::eval::Parallel<legged::Params> eval_t;
#else
    typedef sf::eval::Eval<legged::Params> eval_t;
#endif

#ifdef NOVELTY
    typedef sf::modif::Novelty<phen_t,legged::Params> modifier_t;
#else
    typedef sf::modif::Dummy<legged::Params> modifier_t;
#endif

    // STATS
    typedef boost::fusion::vector<
            sf::stat::ParetoFront<phen_t, legged::Params>,
            sf::stat::BD<phen_t,legged::Params>>  stat_t;
            //      sf::stat::Traj<phen_t, Params>

    typedef sf::ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, legged::Params> ea_t;



    ea_t ea;
    ea.set_res_dir(legged::Params::ea::log_dir());
    sf::run_ea(argc,argv,ea);
}