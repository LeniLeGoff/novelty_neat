#include <iostream>
#include <sferes/stat/state.hpp>

int main(int argc, char** argv)
{

	if(argc < 2){
		std::cout << "usage : " << std::endl;
		std::cout << "\t- serialized gen to load" << std::endl;
		std::cout << "\t- output file" << std::endl;
		return 1;
	}

	return 0;
}