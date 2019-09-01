#include "biped.hpp"
#include <boost/random.hpp>

int main(int argc, char** argv){

	if(argc < 5){
		std::cout << "usages" << std::endl;
		std::cout << "	- specify number of steps" << std::endl;
		std::cout << "	- self collision (0|1)" << std::endl;
		std::cout << "	- enable spd (0|1)" << std::endl;
		std::cout << "	- enable ankle (0|1)" << std::endl;
		return 1;
	}
	boost::random::mt19937  gen;
	boost::random::uniform_real_distribution<> rand_nb(-20,20);
	int nb_step = std::stoi(argv[1]);

	biped::Simulation sim(biped::Params::biped::actuator_type,
		biped::Params::simu::model_path());

	sim.enable_self_collision(std::stoi(argv[2]));
	sim.enable_spd(std::stoi(argv[3]));
	sim.enable_ankle_strat(std::stoi(argv[4]));



	sim.init_visu();

	for(int i = 0; i < nb_step; i++){
		sim.update(i);
		for(int i = 0; i < 14; i++){
			float cmd = rand_nb(gen);
			std::cout << biped::Controller::dof_names[i] << " " << cmd << std::endl;
	    	sim._controller->setCommands(biped::Controller::dof_names[i],cmd);
		}
	}


	

	return 0;
}