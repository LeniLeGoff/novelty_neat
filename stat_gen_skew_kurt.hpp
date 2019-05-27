#ifndef STAT_GEN_SKEWNESS_KURTOSIS_HPP
#define STAT_GEN_SKEWNESS_KURTOSIS_HPP

#include <sferes/stat/stat.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace sferes
{
namespace stat
{


/** \brief Stastistic used to save the behavior descriptor of the robot into
    * files for later analysis.
    */
SFERES_STAT(BDSkewnessKurtosis, Stat)
{
    public:
    template<typename E>
    void refresh(const E& ea)
    {
        this->_create_log_file(ea, "gen_skew_kurt.dat");

        (*this->_log_file) << ea.gen() << " " << ea.nb_evals();
        for(size_t i = 0; i < ea.pop()[0]->fit().pos_bd.size(); i++)
            (*this->_log_file) << " " << kurtosis(ea,i);
        for(size_t i = 0; i < ea.pop()[0]->fit().pos_bd.size(); i++)
            (*this->_log_file) << " " << skewness(ea,i);

        (*this->_log_file) << std::endl;

    }

    template<typename E>
    Eigen::Vector3d mean(const E& ea, int k){
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();;
        Eigen::Vector3d v;

        for(size_t i = 0; i < ea.pop().size(); i++){
            v(0) = ea.pop()[i]->fit().pos_bd[k].x();
            v(1) = ea.pop()[i]->fit().pos_bd[k].y();
            v(2) = ea.pop()[i]->fit().pos_bd[k].theta();

            sum += v;
        }
        return sum/(double)ea.pop().size();
    }

    template<typename E>
    Eigen::Matrix3d covariance(const E& ea, int k){
        Eigen::Matrix3d sum = Eigen::Matrix3d::Zero();;
        Eigen::Vector3d v;
        Eigen::Vector3d m = mean(ea,k);
        for(size_t i = 0; i < ea.pop().size(); i++){
            v(0) = ea.pop()[i]->fit().pos_bd[k].x();
            v(1) = ea.pop()[i]->fit().pos_bd[k].y();
            v(2) = ea.pop()[i]->fit().pos_bd[k].theta();

            sum += (v - m)*(v - m).transpose();
        }
        return sum/((double)ea.pop().size()-1);
    }

    template<typename E>
    double skewness(const E& ea, int k){
        double sum = 0;
        Eigen::Vector3d v1,v2;
        Eigen::Vector3d m = mean(ea,k);
        Eigen::Matrix3d covar = covariance(ea,k);
        for(size_t i = 0; i < ea.pop().size(); i++){
            v1(0) = ea.pop()[i]->fit().pos_bd[k].x();
            v1(1) = ea.pop()[i]->fit().pos_bd[k].y();
            v1(2) = ea.pop()[i]->fit().pos_bd[k].theta();

            for(size_t j = 0; j < ea.pop().size(); j++){
                v2(0) = ea.pop()[j]->fit().pos_bd[k].x();
                v2(1) = ea.pop()[j]->fit().pos_bd[k].y();
                v2(2) = ea.pop()[j]->fit().pos_bd[k].theta();
                double val = (v1 - m).transpose()*covar.inverse()*(v2 - m);
                sum += val*val*val;
            }
        }
        return sum/(ea.pop().size()*ea.pop().size());
    }

    template<typename E>
    double kurtosis(const E& ea, int k){
        double sum = 0;
        Eigen::Vector3d v;
        Eigen::Vector3d m = mean(ea,k);
        Eigen::Matrix3d covar = covariance(ea,k);
        for(size_t i = 0; i < ea.pop().size(); i++){
            v(0) = ea.pop()[i]->fit().pos_bd[k].x();
            v(1) = ea.pop()[i]->fit().pos_bd[k].y();
            v(2) = ea.pop()[i]->fit().pos_bd[k].theta();
            double val = (v - m).transpose()*covar.inverse()*(v-m);
            sum += val*val;
        }
        return sum/ea.pop().size();
    }


};
}
}
#endif //STAT_GEN_SKEWNESS_KURTOSIS_HPP
