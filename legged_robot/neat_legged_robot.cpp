#include <multineat/Population.h>
#include "legged_robot.hpp"
#include <sferes/misc/rand.hpp>
#include <boost/algorithm/string.hpp>

#ifdef VISU
#define NO_PARALLEL
#else
#include <sferes/parallel.hpp>
namespace sf_tbb = sferes::parallel;
#endif

void init_params(NEAT::Parameters& params)
{
	params.PopulationSize = legged::Params::pop::size;
    params.DynamicCompatibility = true;
    params.CompatTreshold = 2.0;
    params.YoungAgeTreshold = 15;
    params.SpeciesMaxStagnation = 100;
    params.OldAgeTreshold = 35;
    params.MinSpecies = 5;
    params.MaxSpecies = 10;
    params.RouletteWheelSelection = false;

    params.MutateRemLinkProb = 0.02;
    params.RecurrentProb = 0.5;
    params.RecurrentLoopProb = 0.5;
    params.OverallMutationRate = 0.15;
    params.MutateAddLinkProb = 0.1;
    params.MutateAddNeuronProb = 0.1;
    params.MutateWeightsProb = 0.1;
    params.MaxWeight = 8.0;
    params.WeightMutationMaxPower = 0.2;
    params.WeightReplacementMaxPower = 1.0;

    params.MutateActivationAProb = 0.0;
    params.ActivationAMutationMaxPower = 0.5;	
    params.MinActivationA = 0.05;
    params.MaxActivationA = 6.0;

    params.MutateNeuronActivationTypeProb = 0.03;

    params.ActivationFunction_SignedSigmoid_Prob = 0.0;
    params.ActivationFunction_UnsignedSigmoid_Prob = 0.0;
    params.ActivationFunction_Tanh_Prob = 1.0;
    params.ActivationFunction_TanhCubic_Prob = 0.0;
    params.ActivationFunction_SignedStep_Prob = 1.0;
    params.ActivationFunction_UnsignedStep_Prob = 0.0;
    params.ActivationFunction_SignedGauss_Prob = 1.0;
    params.ActivationFunction_UnsignedGauss_Prob = 0.0;
    params.ActivationFunction_Abs_Prob = 0.0;
    params.ActivationFunction_SignedSine_Prob = 1.0;
    params.ActivationFunction_UnsignedSine_Prob = 0.0;
    params.ActivationFunction_Linear_Prob = 1.0;
}

std::vector<double> step_nn(NEAT::NeuralNetwork& nn, legged::Simulation& simu,const std::vector<double>& inputs){
	
    nn.Input(inputs);
    nn.ActivateUseInternalBias();
    // nn.ActivateUseInternalBias();
    
    std::vector<double> output = nn.Output();

    double lower, upper;
    
    int i = 0;
    for(const std::string &dofname : simu._controller->get_dofs_names()){
        lower = -3;//simu._controller->get_model()->getDof(dofname)->getVelocityLowerLimit();
        upper = 3;//simu._controller->get_model()->getDof(dofname)->getVelocityUpperLimit();
        // std::cout << "upper :" << upper << " lower : " <<  lower << std::endl;
        if(std::isinf(lower))
            lower = -7;
        if(std::isinf(upper))
            upper = 7;
        output[i] = output[i]*(upper - lower) + lower;
        i++;
    }
    return output;
}

bool get_inputs(legged::Simulation& simu, std::vector<double>& inputs){
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
           // std::cerr << "feedback : " << feedback(i) << " ";
           // std::cerr << "input : " << inputs[i] << " ";
           // std::cerr << "limits : lower " << lower_limits(j) << " upper " << upper_limits(j) << std::endl;

            i++;
        }
        k++;
    }
    for(double input : inputs)
       if(std::isnan(input))
        return false;
    return true;
}


     


class Novelty {
public:
  typedef std::vector<Eigen::VectorXd> pop_t;

  Novelty() : _rho_min(legged::Params::novelty::rho_min_init), _not_added(0) {}


  	void distance_f(Eigen::MatrixXf &distances, const pop_t& pop, const pop_t& archive){
  		for (size_t i = 0; i < pop.size(); ++i)
            for (size_t j = 0; j < archive.size(); ++j)
                distances(i, j) = (pop[i] - archive[j]).norm();
  	}

	std::vector<double> apply(const pop_t &pop) {
	    const size_t k = legged::Params::novelty::k;
	    // merge the population and the archive in a single archive
	    pop_t archive = _archive;
	    archive.insert(archive.end(), pop.begin(), pop.end());

	    // we compute all the distances from pop(i) to archive(j) and store them
	
	    Eigen::MatrixXf distances(pop.size(), archive.size());
	    distance_f(distances, pop, archive);
	    // parallel::init();
	    // parallel::p_for(parallel::range_t(0, pop.size()), f);

	    // compute the sparseness of each individual of the population
	    // and potentially add some of them to the archive
	    std::vector<double> novelty_scores;
	    int added = 0;
	    for (size_t i = 0; i < pop.size(); ++i) {
	      	Eigen::VectorXf vd = distances.row(i);

	    	double n = 0.0;
	    	std::partial_sort(vd.data(),
	                        vd.data() + k,
	                        vd.data() + vd.size());

	    	novelty_scores.push_back(vd.head<k>().sum() / k);
	    	
	      	// add to the archive
	    	if (n > _rho_min
	        	|| sferes::misc::rand<float>() < legged::Params::novelty::add_to_archive_prob) {
	        	_archive.push_back(pop[i]);
	        	_not_added = 0;
	        	++added;
	    	} else {
	        	++_not_added;
	      	}
	    } // end for all individuals

	    // update rho_min
	    if (_not_added > legged::Params::novelty::stalled_tresh) { //2500
	      _rho_min *= 0.95;
	      _not_added = 0;
	    }
	    if (_archive.size() > legged::Params::novelty::k
	        && added > legged::Params::novelty::adding_tresh)//4
	      _rho_min *= 1.05f;

	  	return novelty_scores;
	  }
	  const pop_t& archive() const { return _archive; }
protected:
  	pop_t _archive;
  	pop_t pop;
  	float _rho_min;
  	size_t _not_added;
};

void make_res_dir(std::string& res_dir, const std::string &exp_name) {
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
    std::vector<std::string> strs;
    boost::split(strs,exp_name,boost::is_any_of("/"));
    std::string folder = std::string("/") + strs.back() 
                + std::string("_") + stream.str();
    if(!boost::filesystem::exists(legged::Params::ea::log_dir() + folder))
        boost::filesystem::create_directory(legged::Params::ea::log_dir() + folder);
    res_dir = legged::Params::ea::log_dir() + folder;

}

void stat_bd(const std::string& res_dir,const Novelty::pop_t pop_pos){
    static int bd_id=0;
    std::ostringstream ofbd_name;
    ofbd_name << res_dir << "/bd_" << std::setfill('0') << std::setw(6) << bd_id << ".log";
    bd_id++;

    std::ofstream outbd(ofbd_name.str());
    if (!outbd.is_open()) {
        std::cerr<<"Can't open file to save behavior descriptors: "<<ofbd_name.str()<<std::endl;
        exit(1);
    }

    for (size_t i = 0; i < pop_pos.size(); ++i)
    {
    
        outbd<<pop_pos[i](0) <<" ";
        outbd<<pop_pos[i](1) <<" ";
        outbd<<pop_pos[i](2) <<" ";
        outbd<<std::endl;
    }
    outbd.close();
}

void stat_neat(const std::string& res_dir, NEAT::Population& pop){

    static int file_id=0;
    std::ostringstream of_name;
    of_name<<res_dir<<"/neat_"<<std::setfill('0')<<std::setw(6)<<file_id<<".dat";
    file_id++;

    std::ofstream outf(of_name.str());
    if (!outf.is_open()) {
        std::cerr << " Can't open file to save neat data: " << of_name.str() << std::endl;
        exit(1);
    }

    for (size_t i = 0; i < legged::Params::pop::size; ++i)
    {
        outf << pop.AccessGenomeByIndex(i).m_NeuronGenes.size() << " ";
    }
    outf << std::endl;
    for (size_t i = 0; i < legged::Params::pop::size; ++i)
    {
        outf << pop.AccessGenomeByIndex(i).m_LinkGenes.size() << " ";
    }
    outf << std::endl;
    outf.close();

}

int main(int argc, char** argv)
{
    NEAT::Parameters params;
    init_params(params);

    NEAT::Genome neat_genome(0, legged::Params::dnn::nb_inputs, 0, legged::Params::dnn::nb_outputs, false, NEAT::SIGNED_SIGMOID, NEAT::SIGNED_SIGMOID, 0, params, 0);
    NEAT::Population population(neat_genome, params, true, 1.0, std::random_device{}());

    std::string res_dir;
    make_res_dir(res_dir,std::string(argv[0]));

    Novelty novelty;
    std::vector<double> novelty_scores;

#ifndef NO_PARALLEL
    sf_tbb::init();
#endif

    for(int g = 0; g < legged::Params::pop::nb_gen ; g++){
    	Novelty::pop_t pop_pos(legged::Params::pop::size); //Store the final positions of the robot.
#ifndef NO_PARALLEL
    	sf_tbb::p_for(sf_tbb::range_t(0,legged::Params::pop::size),[&](sf_tbb::range_t& r){
    		for(int k = r.begin(); k != r.end(); ++k){
#else
	    for(int k = 0; k < legged::Params::pop::size; ++k){
#endif
	    	legged::Simulation simu(legged::Params::robot::actuator_type,legged::Params::simu::model_path());

#ifdef VISU
			std::cout << "Initiatlization of the visualisation" << std::endl;
		    simu.init_visu();
#endif

			std::vector<Eigen::VectorXd> pos_bd;
	        NEAT::NeuralNetwork nn;
	    	population.AccessGenomeByIndex(k).BuildPhenotype(nn);
			for(int i = 0; i < legged::Params::simu::nb_steps /*&& !stop_eval*/;){

				std::vector<Eigen::VectorXd> pos_bd;

	    		std::vector<double> inputs;
		        if(!get_inputs(simu,inputs)){
		            std::cerr << "Stop simulation because of aberrant behavior" << std::endl;
		            pos_bd.clear();
		            for(int j = 0; j < legged::Params::novelty::nb_pos; j++)
		                pos_bd.push_back(Eigen::VectorXd::Zero(3));
		            return 1;
		        }


		        if(pos_bd.size() < legged::Params::novelty::nb_pos
		                && i > 0
		                && (i%(int)std::round(legged::Params::simu::nb_steps/legged::Params::novelty::nb_pos)==0)){
		                    if(!std::isnan(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3)(0)))
		                        pos_bd.push_back(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3));
		                    else pos_bd.push_back(Eigen::VectorXd::Zero(3));
		        }
		        
		        std::vector<double> outputs = step_nn(nn,simu,inputs);
		        simu._controller->setCommands(outputs);

		        simu.update(i);
		        // for(const auto& o: outputs)
		        // 	std::cout << o << ",";
		        // std::cout << std::endl;
		        i++;
		    }

	        if(!std::isnan(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3)(0)))
	            pos_bd.push_back(simu._controller->get_model()->getBodyNode("base_link")->getWorldTransform().matrix().col(3));
	        else pos_bd.push_back(Eigen::VectorXd::Zero(3));
	        pop_pos[k] = pos_bd.back();
#ifndef NO_PARALLEL
	    }
    	});
#else   
		}
#endif
		
		novelty_scores = novelty.apply(pop_pos);
		for(int k = 0; k < legged::Params::pop::size; k++)
	    	population.AccessGenomeByIndex(k).SetFitness(novelty_scores[k]);
	    stat_bd(res_dir,pop_pos);
	    stat_neat(res_dir,population);
	   	
	   	std::cout << "Generation - " << g << "finished" << std::endl;
	    population.Epoch();

	}	

    return 0;
}

