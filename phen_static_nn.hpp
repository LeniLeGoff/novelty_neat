#ifndef PHEN_STATIC_NN_HPP
#define PHEN_STATIC_NN_HPP


#include <map>
#include <sferes/phen/indiv.hpp>
#include "gen_static_nn.hpp"

namespace sferes {
namespace phen {

SFERES_INDIV(StaticNN,Indiv){
  public:
    typedef typename Gen::nn_t nn_t;

    void develop(){
        BGL_FORALL_VERTICES_T(v, this->gen().get_graph(),
                              typename nn_t::graph_t) {
          this->gen().get_graph()[v].get_afparams().develop();
          this->gen().get_graph()[v].get_pfparams().develop();
        }
        BGL_FORALL_EDGES_T(e, this->gen().get_graph(),
                           typename nn_t::graph_t) {
          this->gen().get_graph()[e].get_weight().develop();
        }
        this->gen().init();
    }

    void show(std::ostream& os){
        this->gen().write(os);
    }

    nn_t& nn(){
        return this->gen();
    }

    const nn_t& nn() const{
        return this->gen();
    }
};

}

}

#endif //PHEN_STATIC_NN_HPP
