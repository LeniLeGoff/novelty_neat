#ifndef STAT_CHILD_POP_HPP
#define STAT_CHILD_POP_HPP

#include <sferes/stat/stat.hpp>

namespace sferes
{
namespace stat
{


/** \brief Stastistic used to save the behavior descriptor of the robot into
    * files for later analysis.
    */
SFERES_STAT(ChildPop, Stat)
{
    public:
    template<typename E>
    void refresh(const E& ea)
    {

        static int id=0;
        std::ostringstream file_name;
        file_name<<ea.res_dir()<<"/bd_"<<std::setfill('0')<<std::setw(6)<<id<<".log";
        id++;

        std::ofstream output(file_name.str());
        if (!output.is_open()) {
            std::cerr<<"Can't open file to save behavior descriptors: "<<file_name.str()<<std::endl;
            exit(1);
        }

        for (size_t i = 0; i < ea.child_pop().size(); ++i)
        {
            for (int j=0;j<ea.child_pop()[i]->fit().pos_bd.size();j++) {
                output<<ea.child_pop()[i]->fit().pos_bd[j](0)<<" ";
                output<<ea.child_pop()[i]->fit().pos_bd[j](1) <<" ";
                output<<ea.child_pop()[i]->fit().pos_bd[j](2) <<" ";
            }
            output<<std::endl;
        }


        output.close();
    }

};
}
}
#endif
