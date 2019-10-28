/*
 * Created by L.K Le Goff
 */

#include <iostream>
#include <ctime>
#include <chrono>
//#include <ratio>
#include <boost/algorithm/string.hpp>

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

#ifndef RANK
#include <sferes/ea/nsga2.hpp>
#else
#include <sferes/ea/rank_simple.hpp>
#endif

#include <sferes/stat/pareto_front.hpp>

#include <nn2/gen_dnn_ff.hpp>
#include <nn2/phen_dnn.hpp>

#include "fit_legged_robot.hpp"
#include "parameters.hpp"
#include "../phen_static_nn.hpp"
#include "../phen_static_rnn.hpp"
#include "stat_bd.hpp"
#include "stat_neat.hpp"

#define EA_

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
#ifndef RANK
            sf::stat::ParetoFront<phen_t, legged::Params>,
#endif
            sf::stat::BD<phen_t,legged::Params>
#ifdef NEAT
            ,sf::stat::Neat<phen_t,legged::Params>
#endif
            >  stat_t;
            //      sf::stat::Traj<phen_t, Params>

#ifndef RANK
    typedef sf::ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, legged::Params> ea_t;
#else
    typedef sf::ea::RankSimple<phen_t, eval_t, stat_t, modifier_t, legged::Params> ea_t;
#endif

    ea_t ea;

    //set logs directory
    typedef std::chrono::duration<double,std::milli> milli_sec;
    std::chrono::time_point<std::chrono::high_resolution_clock,milli_sec> milli = std::chrono::time_point_cast<milli_sec>(std::chrono::high_resolution_clock::now());
    double time_milli = milli.time_since_epoch().count();
    time_milli = time_milli/(10000.f);
    time_milli = time_milli - static_cast<int>(time_milli);
    time_milli = std::trunc(time_milli*10000.f);
    std::time_t present_time = std::time(nullptr);
    std::tm* date = std::localtime(&present_time);
    std::stringstream stream;
    stream << date->tm_mday 
        << "_" << date->tm_mon
        << "_" << date->tm_hour 
        << "-" << date->tm_min 
        << "-" << date->tm_sec
        << "-" << time_milli;
    std::string command_line = std::string(argv[0]);
    std::vector<std::string> strs;
    boost::split(strs,command_line,boost::is_any_of("/"));
    std::string folder = std::string("/") + strs.back() 
                + std::string("_") + stream.str();
    if(!boost::filesystem::exists(legged::Params::ea::log_dir() + folder))
        boost::filesystem::create_directory(legged::Params::ea::log_dir() + folder);
    ea.set_res_dir(legged::Params::ea::log_dir() + folder);
    
    sf::run_ea(argc,argv,ea);
}
