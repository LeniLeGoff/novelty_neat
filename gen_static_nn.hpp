#ifndef GEN_STATIC_NN_HPP
#define GEN_STATIC_NN_HPP


#include <bitset>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/utility.hpp>
#include <sferes/dbg/dbg.hpp>
#include <sferes/misc.hpp>

#include "mlp.hpp"
//#include <nn2/trait.hpp>


//namespace nn {

//template<>
//struct trait<sf::gen::EvoFloat<1,Params>>{
//typedef sf::gen::EvoFloat<1,Params> e_float_t;

//    static e_float_t zero(){
//        e_float_t ef;
//        for(int i = 0; i < 1; i++)
//            ef.data(i,0.0f);
//        return ef;
//    }


//};

//}

namespace sferes {
namespace gen {

template<typename N, typename C, typename Params>
class StaticNN : public nn::Mlp<N,C> {
public:
    typedef nn::Mlp<N, C> mlp_t;
    typedef N neuron_t;
    typedef C conn_t;
    typedef typename mlp_t::graph_t graph_t;
    typedef typename mlp_t::vertex_desc_t vertex_desc_t;
    typedef typename mlp_t::weight_t weight_t;


    StaticNN() :
        nn::Mlp<N,C>(Params::dnn::nb_inputs-1,Params::static_nn::nb_hidden,Params::dnn::nb_outputs)
    {}

    void random(){
        BGL_FORALL_EDGES_T(e,this->_g,graph_t)
            this->_g[e].get_weight().random();

    }

    void mutate(){
        BGL_FORALL_EDGES_T(e, this->_g, graph_t)
            this->_g[e].get_weight().mutate();
    }

    void cross(const StaticNN& o, StaticNN& c1, StaticNN& c2){
        c1 = *this;
        c2 = o;
    }

    template<typename Archive>
    void save(Archive& a, const unsigned v) const {
      dbg::trace("nn", DBG_HERE);
      std::vector<int> inputs;
      std::vector<int> outputs;
      std::vector<typename neuron_t::af_t::params_t> afparams;
      std::vector<typename neuron_t::pf_t::params_t> pfparams;
      std::map<vertex_desc_t, int> nmap;
      std::vector<std::pair<int, int> > conns;
      std::vector<weight_t> weights;

      BGL_FORALL_VERTICES_T(v, this->_g, graph_t) {
        if (this->is_input(v))
          inputs.push_back(afparams.size());
        if (this->is_output(v))
          outputs.push_back(afparams.size());
        nmap[v] = afparams.size();
        afparams.push_back(this->_g[v].get_afparams());
        pfparams.push_back(this->_g[v].get_pfparams());
      }
      BGL_FORALL_EDGES_T(e, this->_g, graph_t) {
        conns.push_back(std::make_pair(nmap[source(e, this->_g)],
                                       nmap[target(e, this->_g)]));
        weights.push_back(this->_g[e].get_weight());
      }
      assert(pfparams.size() == afparams.size());
      assert(weights.size() == conns.size());

      a & BOOST_SERIALIZATION_NVP(afparams);
      a & BOOST_SERIALIZATION_NVP(pfparams);
      a & BOOST_SERIALIZATION_NVP(weights);
      a & BOOST_SERIALIZATION_NVP(conns);
      a & BOOST_SERIALIZATION_NVP(inputs);
      a & BOOST_SERIALIZATION_NVP(outputs);
    }
    template<typename Archive>
    void load(Archive& a, const unsigned v) {
      dbg::trace("nn", DBG_HERE);
      std::vector<int> inputs;
      std::vector<int> outputs;
      std::vector<typename neuron_t::af_t::params_t> afparams;
      std::vector<typename neuron_t::pf_t::params_t> pfparams;
      std::map<size_t, vertex_desc_t> nmap;
      std::vector<std::pair<int, int> > conns;
      std::vector<weight_t> weights;

      a & BOOST_SERIALIZATION_NVP(afparams);
      a & BOOST_SERIALIZATION_NVP(pfparams);
      a & BOOST_SERIALIZATION_NVP(weights);
      a & BOOST_SERIALIZATION_NVP(conns);
      a & BOOST_SERIALIZATION_NVP(inputs);
      a & BOOST_SERIALIZATION_NVP(outputs);

      assert(pfparams.size() == afparams.size());

      assert(weights.size() == conns.size());
      this->set_nb_inputs(inputs.size());
      this->set_nb_outputs(outputs.size());
      for (size_t i = 0; i < this->get_nb_inputs(); ++i)
        nmap[inputs[i]] = this->get_input(i);
      for (size_t i = 0; i < this->get_nb_outputs(); ++i)
        nmap[outputs[i]] = this->get_output(i);

      for (size_t i = 0; i < afparams.size(); ++i)
        if (std::find(inputs.begin(), inputs.end(), i) == inputs.end()
            && std::find(outputs.begin(), outputs.end(), i) == outputs.end())
          nmap[i] = this->add_neuron("n", pfparams[i], afparams[i]);
        else {
          this->_g[nmap[i]].set_pfparams(pfparams[i]);
          this->_g[nmap[i]].set_afparams(afparams[i]);
        }


      //assert(nmap.size() == num_vertices(this->_g));
      for (size_t i = 0; i < conns.size(); ++i)
        this->add_connection(nmap[conns[i].first], nmap[conns[i].second], weights[i]);
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER();


};

}
}

#endif //GEN_STATIC_NN_HPP
