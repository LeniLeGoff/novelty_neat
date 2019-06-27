#ifndef STAT_NEAT_HPP
#define STAT_NEAT_HPP

#include <sferes/stat/stat.hpp>
//#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

namespace sferes
{
namespace stat
{


/** \brief Stastistic used to save the behavior descriptor of the robot into
    * files for later analysis.
    */
SFERES_STAT(Neat, Stat)
{
    public:
    template<typename E>
    void refresh(const E& ea)
    {

        static int file_id=0;
        std::ostringstream of_name;
        of_name<<ea.res_dir()<<"/neat_"<<std::setfill('0')<<std::setw(6)<<file_id<<".dat";
        file_id++;

        std::ofstream outf(of_name.str());
        if (!outf.is_open()) {
            std::cerr << " Can't open file to save neat data: " << of_name.str() << std::endl;
            exit(1);
        }

        for (size_t i = 0; i < ea.pop().size(); ++i)
        {
            outf << ea.pop()[i]->gen().get_nb_neurons() << " ";
            outf << std::endl;
        }

        for (size_t i = 0; i < ea.pop().size(); ++i)
        {
            outf << ea.pop()[i]->gen().get_nb_connections() << " ";
            outf << std::endl;
        }
        outf.close();
    }



};
}
}


#endif // STAT_NEAT_HPP
