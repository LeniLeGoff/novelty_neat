
#ifndef SIMPLE_EA_HPP_
#define SIMPLE_EA_HPP_

#include <algorithm>
#include <limits>

#include <boost/foreach.hpp>

#include <sferes/stc.hpp>
#include <sferes/parallel.hpp>
#include <sferes/ea/ea.hpp>
#include <sferes/fit/fitness.hpp>
#include <sferes/ea/dom_sort.hpp>
#include <sferes/ea/common.hpp>
#include <sferes/ea/crowd.hpp>

namespace sferes {
  namespace ea {
    template<typename Phen, typename Eval, typename Stat, typename FitModifier, typename Crowd, typename Params,
             typename Exact = stc::Itself>
    class RankCrowdSimple : public Ea <Phen, Eval, Stat, FitModifier, Params,
    typename stc::FindExact<RankCrowdSimple<Phen, Eval, Stat, FitModifier, Params, Exact>, Exact>::ret > {
    public:
      typedef boost::shared_ptr<crowd::Indiv<Phen> > indiv_t;
      typedef typename std::vector<indiv_t> pop_t;
      typedef typename pop_t::iterator it_t;
      typedef typename std::vector<std::vector<indiv_t> > front_t;
      SFERES_EA_FRIEND(RankCrowdSimple);

      void random_pop() {
        parallel::init();

        _parent_pop.resize(Params::pop::size);
        assert(Params::pop::size % 4 == 0);

        pop_t init_pop((size_t)(Params::pop::size * Params::pop::initial_aleat));
        parallel::p_for(parallel::range_t(0, init_pop.size()),
                        random<crowd::Indiv<Phen> >(init_pop));
        _eval_subpop(init_pop);
        _apply_modifier(init_pop);
        front_t fronts;
        _rank_crowd(init_pop, fronts);
      }

      // a step
      void epoch() {
        // this->_pop.clear();
        // _pareto_front.clear();
        _selection (_parent_pop, _child_pop);
        parallel::p_for(parallel::range_t(0, _child_pop.size()),
                        mutate<crowd::Indiv<Phen> >(_child_pop));
#ifndef EA_EVAL_ALL
        _eval_subpop(_child_pop);
        _merge(_parent_pop, _child_pop, _mixed_pop);
#else
        _merge(_parent_pop, _child_pop, _mixed_pop);
        _eval_subpop(_mixed_pop);
#endif
        _apply_modifier(_mixed_pop);
#ifndef NDEBUG
        BOOST_FOREACH(indiv_t& ind, _mixed_pop)
        for (size_t i = 0; i < ind->fit().objs().size(); ++i) {
          assert(!std::isnan(ind->fit().objs()[i]));
        }
#endif
        front_t fronts;
        _rank_crowd(_mixed_pop,fronts);
        _mixed_pop.clear();
        _child_pop.clear();

        _convert_pop(_parent_pop, this->_pop);

        assert(_parent_pop.size() == Params::pop::size);
        assert(_pareto_front.size() <= Params::pop::size * 2);
        assert(_mixed_pop.size() == 0);
        //	assert(_child_pop.size() == 0);
        assert(this->_pop.size() == Params::pop::size);
      }
      const std::vector<boost::shared_ptr<Phen> >& pareto_front() const {
        return _pareto_front;
      }
      const pop_t& mixed_pop() const {
        return _mixed_pop;
      }
      const pop_t& parent_pop() const {
        return _parent_pop;
      }
      const pop_t& child_pop() const {
        return _child_pop;
      }

    protected:
      std::vector<boost::shared_ptr<Phen> > _pareto_front;
      pop_t _parent_pop;
      pop_t _child_pop;
      pop_t _mixed_pop;

      // for resuming
      void _set_pop(const std::vector<boost::shared_ptr<Phen> >& pop) {
        assert(!pop.empty());
        _parent_pop.resize(pop.size());
        for (size_t i = 0; i < pop.size(); ++i)
          _parent_pop[i] = boost::shared_ptr<crowd::Indiv<Phen> >(new crowd::Indiv<Phen>(*pop[i]));
      }
      void _update_pareto_front(const front_t& fronts) {
        _convert_pop(fronts.front(), _pareto_front);
      }

      void _convert_pop(const pop_t& pop1,
                        std::vector<boost::shared_ptr<Phen> > & pop2) {
        pop2.resize(pop1.size());
        for (size_t i = 0; i < pop1.size(); ++i)
          pop2[i] = pop1[i];
      }

      void _eval_subpop(pop_t& pop) {
        this->_eval_pop(pop, 0, pop.size());
      }

      void _apply_modifier(pop_t& pop) {
        _convert_pop(pop, this->_pop);
        this->apply_modifier();
      }


      //
      void _merge(const pop_t& pop1, const pop_t& pop2, pop_t& pop3) {
        assert(pop1.size());
        assert(pop2.size());
        pop3.clear();
        pop3.insert(pop3.end(), pop1.begin(), pop1.end());
        pop3.insert(pop3.end(), pop2.begin(), pop2.end());
        assert(pop3.size() == pop1.size() + pop2.size());
      }

      // --- tournament selection ---
      void _selection(pop_t& old_pop, pop_t& new_pop) {
        new_pop.resize(old_pop.size());
        std::vector<size_t> a1, a2;
        misc::rand_ind(a1, old_pop.size());
        misc::rand_ind(a2, old_pop.size());
        // todo : this loop could be parallelized
        for (size_t i = 0; i < old_pop.size(); i += 4) {
          const indiv_t& p1 = _tournament(old_pop[a1[i]], old_pop[a1[i + 1]]);
          const indiv_t& p2 = _tournament(old_pop[a1[i + 2]], old_pop[a1[i + 3]]);
          const indiv_t& p3 = _tournament(old_pop[a2[i]], old_pop[a2[i + 1]]);
          const indiv_t& p4 = _tournament(old_pop[a2[i + 2]], old_pop[a2[i + 3]]);
          assert(i + 3 < new_pop.size());
          p1->cross(p2, new_pop[i], new_pop[i + 1]);
          p3->cross(p4, new_pop[i + 2], new_pop[i + 3]);
        }
      }

      const indiv_t& _tournament(const indiv_t& i1, const indiv_t& i2) {
        // if (i1->rank() < i2->rank())
        //   return i1;
        // else if (i2->rank() > i1->rank())
        //   return i2;
        // else if (misc::flip_coin())
        //   return i1;
        // else
        //   return i2;

        int flag = fit::dominate_flag(i1, i2);
        if (flag == 1)
          return i1;
        if (flag == -1)
          return i2;
        if (i1->crowd() > i2->crowd())
          return i1;
        if (i1->crowd() < i2->crowd())
          return i2;
        if (misc::flip_coin())
          return i1;
        else
          return i2;
      }

      // --- rank & crowd ---

      void _rank_crowd(pop_t& pop, front_t& fronts) {
        std::vector<size_t> ranks;
#ifndef NDEBUG
        BOOST_FOREACH(indiv_t& ind, pop)
        for (size_t i = 0; i < ind->fit().objs().size(); ++i) {
          assert(!std::isnan(ind->fit().objs()[i]));
        }
#endif
        dom_sort(pop, fronts, ranks);
        _update_pareto_front(fronts);
        parallel::p_for(parallel::range_t(0, fronts.size()),
                        crowd::assign_crowd<indiv_t >(fronts));

        for (size_t i = 0; i < ranks.size(); ++i)
          pop[i]->set_rank(ranks[i]);
        parallel::sort(pop.begin(), pop.end(), crowd::compare_ranks());
      }

      void _assign_rank(pop_t& pop) {
        int rank = 0;
        fit::compare_pareto comp;
        assert(pop.size());
        std::sort(pop.begin(), pop.end(), comp);
        pop[0]->set_rank(0);
        for (unsigned i = 1; i < pop.size(); ++i) {
          assert(comp(pop[i-1], pop[i]) || comp.eq(pop[i -1], pop[i]));
          if (comp(pop[i-1], pop[i]))
            ++rank;
          pop[i]->set_rank(rank);
        }
      }

    };

    template<typename Phen, typename Eval, typename Stat, typename FitModifier, typename Params,
             typename Exact = stc::Itself>
    class SimpleEA : public RankCrowdSimple <Phen, Eval, Stat, FitModifier,
      crowd::assign_crowd<boost::shared_ptr<crowd::Indiv<Phen> > >, Params,
    typename stc::FindExact<SimpleEA<Phen, Eval, Stat, FitModifier, Params, Exact>, Exact>::ret >
    {};
  }
}
#endif
