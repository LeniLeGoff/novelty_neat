#ifndef STAT_GEN_SKEWNESS_KURTOSIS_HPP
#define STAT_GEN_SKEWNESS_KURTOSIS_HPP

#include <sferes/stat/stat.hpp>
#include <Eigen/Dense>

namespace sferes
{
namespace stat
{


/** \brief Stastistic used to save the behavior descriptor of the robot into
    * files for later analysis.
    */
SFERES_STAT(GenSkewnessKurtosis, Stat)
{
    public:

    typedef typename Phen::nn_t::graph_t graph_t;

    template<typename E>
    void refresh(const E& ea)
    {
        this->_create_log_file(ea, "gen_skew_kurt.dat");

        (*this->_log_file) << ea.gen() << " " << ea.nb_evals()
                           << " " << kurtosis(ea)
                           << " " << skewness(ea) << std::endl;

    }

    template<typename E>
    Eigen::VectorXd mean(const E& ea){
        int nb_edges = boost::num_edges(ea.pop()[0]->gen().get_graph());
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(nb_edges);
        Eigen::VectorXd v(nb_edges);

        for(size_t i = 0; i < ea.pop().size(); i++){
            int j = 0;
            BGL_FORALL_EDGES_T(e,ea.pop()[i]->gen().get_graph(),graph_t){
                v(j) = ea.pop()[i]->gen().get_graph()[e].get_weight().data()[0];
                j++;
            }
            sum += v;
        }
        return sum/(double)ea.pop().size();
    }

    template<typename E>
    Eigen::MatrixXd covariance(const E& ea){
        int nb_edges = boost::num_edges(ea.pop()[0]->gen().get_graph());
        Eigen::MatrixXd sum = Eigen::MatrixXd::Zero(nb_edges,nb_edges);
        Eigen::VectorXd v(nb_edges);
        Eigen::VectorXd m = mean(ea);
        for(size_t i = 0; i < ea.pop().size(); i++){
            int j = 0;
            BGL_FORALL_EDGES_T(e,ea.pop()[i]->gen().get_graph(),graph_t){
                v(j) = ea.pop()[i]->gen().get_graph()[e].get_weight().data()[0];
                j++;
            }
            sum += (v - m)*(v - m).transpose();
        }
        return sum/((double)ea.pop().size()-1);
    }

    template<typename E>
    double skewness(const E& ea){
        int nb_edges = boost::num_edges(ea.pop()[0]->gen().get_graph());
        double sum = 0;
        Eigen::VectorXd v1(nb_edges),v2(nb_edges);
        Eigen::VectorXd m = mean(ea);
        Eigen::MatrixXd covar = covariance(ea);
        for(size_t i = 0; i < ea.pop().size(); i++){
            int k = 0;
            BGL_FORALL_EDGES_T(e,ea.pop()[i]->gen().get_graph(),graph_t){
                v1(k) = ea.pop()[i]->gen().get_graph()[e].get_weight().data()[0];
                k++;
            }
            for(size_t j = 0; j < ea.pop().size(); j++){
                int k = 0;
                BGL_FORALL_EDGES_T(e,ea.pop()[i]->gen().get_graph(),graph_t){
                    v2(k) = ea.pop()[i]->gen().get_graph()[e].get_weight().data()[0];
                    k++;
                }
                double val = (v1 - m).transpose()*covar.inverse()*(v2 - m);
                sum += val*val*val;
            }
        }
        return sum/(ea.pop().size()*ea.pop().size());
    }

    template<typename E>
    double kurtosis(const E& ea){
        int nb_edges = boost::num_edges(ea.pop()[0]->gen().get_graph());
        double sum = 0;
        Eigen::VectorXd v(nb_edges);
        Eigen::VectorXd m = mean(ea);
        Eigen::MatrixXd covar = covariance(ea);
        for(size_t i = 0; i < ea.pop().size(); i++){
            int k = 0;
            BGL_FORALL_EDGES_T(e,ea.pop()[i]->gen().get_graph(),graph_t){
                v(k) = ea.pop()[i]->gen().get_graph()[e].get_weight().data()[0];
                k++;
            }
            double val = (v - m).transpose()*covar.inverse()*(v-m);
            sum += val*val;
        }
        return sum/ea.pop().size();
    }


};
}
}
#endif //STAT_GEN_SKEWNESS_KURTOSIS_HPP
