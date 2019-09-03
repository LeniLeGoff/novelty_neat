#include "legged_robot.hpp"
#include <boost/random.hpp>

int main(int argc, char** argv){

	if(argc < 3){
		std::cout << "usages" << std::endl;
		std::cout << "  - model path" << std::endl;
		std::cout << "	- specify number of steps" << std::endl;
		std::cout << "	- self collision (0|1)" << std::endl;
	}

	boost::random::mt19937 gen;
	boost::random::uniform_real_distribution<> dist(-5,5);

	int nb_step = std::stoi(argv[2]);

	legged::Simulation sim(legged::Params::robot::actuator_type,
		argv[1]);

	sim.enable_self_collision(std::stoi(argv[3]));

	sim.init_visu();

	
	for(int i = 0; i < nb_step; i++){
		std::cout << "Step ---- " << i << std::endl;
		std::cout << "Positions :" << std::endl;
		std::cout << sim._controller->get_model()->getPositions() << std::endl;
		sim.update(i);
		std::vector<double> cmds;
		std::cout << "Commands" << std::endl;
		for(int k = 0; k < sim._controller->get_dofs_names().size(); k++){
			cmds.push_back(dist(gen));
			std::cout << cmds.back() << std::endl;
		}

		sim._controller->setCommands(cmds);
	}

	return 0;
}