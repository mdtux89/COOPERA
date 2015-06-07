// -*- mode:C++; tab-width():4; c-basic-offset:4; indent-tabs-mode:nil -*-



/*!
 * \file SimulationGraph.h
 *
 * \author Shashank Pathak
 * \date 2014
 *
 * Some useful typedefs related with Boost graph, we use.
 */


#ifndef MYGRAPH_H
#define MYGRAPH_H

#include <Configuration.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/unordered_map.hpp>

#include <boost_serialize_unordered_map.hpp>
#include <UnorderedMapImpl.hpp>
#include <SerializeTuple.hpp>

//namespace repair
//{

    // Define edge properties of the graph and "install" them

    enum edge_action_id_t { edge_action_id };
    enum edge_count_t { edge_count };

    namespace boost {
      BOOST_INSTALL_PROPERTY(edge, action_id);
      BOOST_INSTALL_PROPERTY(edge, count);
    }

    // Define vertex properties and "install" them

    enum vertex_state_id_t { vertex_state_id };
    enum vertex_count_t {vertex_count };

    namespace boost {
      BOOST_INSTALL_PROPERTY(vertex, state_id);
      BOOST_INSTALL_PROPERTY(vertex, count);
    }

    typedef float Weight;
    typedef boost::property<edge_action_id_t, int> actId;
    typedef boost::property<boost::edge_weight_t, Weight, actId> actWeight;
    typedef boost::property<edge_count_t,int,actWeight> EdgeProps;

    typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::bidirectionalS,
        // Vertex properties
        boost::property<vertex_state_id_t, int,
            boost::property<vertex_count_t, int > >,
        // Edge properties
         EdgeProps >
      SimulationGraph;

    // Maps for vertex properties
    typedef boost::property_map<SimulationGraph,
          vertex_state_id_t>::type VertInternalIdMap;
    typedef boost::property_map<SimulationGraph,
          vertex_count_t>::type VertVisitsMap;

    // Maps for edges properties
    typedef boost::property_map<SimulationGraph,
            edge_action_id_t>::type EdgeActionIdMap;
    typedef boost::property_map<SimulationGraph,
            boost::edge_weight_t>::type EdgeWeightMap;
    typedef boost::property_map<SimulationGraph,
            edge_count_t>::type EdgeVisitsMap;

    // Edge and vertex descriptors and iterators
    typedef boost::graph_traits < SimulationGraph >::vertex_descriptor Vertex;
    typedef boost::graph_traits < SimulationGraph >::vertex_iterator Viter;
    typedef boost::graph_traits < SimulationGraph >::edge_descriptor Edge;
    typedef boost::graph_traits < SimulationGraph >::edge_iterator Eiter;
    typedef boost::graph_traits < SimulationGraph >::out_edge_iterator oe_iter;
    typedef boost::graph_traits < SimulationGraph >::in_edge_iterator ie_iter;


    // Predicate Function for use for filtering in map
    template <class ValPropertyMap, class Filtertype>
    struct tag_equals_predicate
    {
      tag_equals_predicate(const Filtertype& x, ValPropertyMap value)
        : m_val(x), m_value_map(value) { }

      template <class VertexOrEdge>
      bool operator()(const VertexOrEdge& e) const {
        return boost::get(m_val,e) == m_value_map[e];
      }
      Filtertype m_val;
      ValPropertyMap m_value_map;
    };
    // helper creation function
    template <class ValPropertyMap, class Filtertype>
    inline tag_equals_predicate<ValPropertyMap,Filtertype>
    tag_equals(Filtertype& val, ValPropertyMap value) {
      return tag_equals_predicate<ValPropertyMap,Filtertype>(val, value);
    }

    template <class Name, class Category>
    class myVertexWriter {
    public:
        myVertexWriter(Name _name, Category _cat) : name(_name), cat(_cat) {}
         template <class VertexOrEdge>
         void operator()(std::ostream& out, const VertexOrEdge& v) const {

             //LATER:if(tag_equals(boost::get(cat,v),cat))
             switch (boost::get(cat,v))
             {
             case 0:
                 out << "[label=\" Init \"]";
                 out << "[style=filled] ";
                 break;
             case 1:
                 out << "[label=\" Safe \"]";
                 out << "[style=filled] ";
                 break;
             case 2:
                 out << "[label=\" Fall \"]";
                 out << "[style=filled] ";
                 break;
             case 3:
                 out << "[label=\" OverHeating \"]";
                 out << "[style=filled] ";
                 break;
             case 4:
                 out << "[label=\" AngleError \"]";
                 out << "[style=filled] ";
                 break;
             case 5:
                 out << "[label=\" OverLoad \"]";
                 out << "[style=filled] ";
                 break;
             default:
                 out << "[label=\"" << boost::get(name,v) << "\"]";
                 //out << "[label=\"" << " " << "\"]";
                 out << "[style=filled] ";

             }

         }
    private:
         Name name;
         Category cat;
    };

    template <class Name, class Filter, class Filter2>
    class myEdgeWriter {
    public:

        myEdgeWriter(Name _name,Filter _filter,Filter2 _filter2) : name(_name),filter(_filter),filter2(_filter2) {}
         template <class VertexOrEdge>
         void operator()(std::ostream& out, const VertexOrEdge& v) const {

                if(boost::get(filter2,v) != -1)
                {
                    out << "[label=\"";
                    double p = boost::get(name,v);
                    if (p != -2) {
                        out << "Act" << p;
                    }
                    out << "\"]";
                    if(boost::get(filter,v) == true)
                    {
                       out << "[color=red] ";
                    }
                    else
                    {
                       out << "[color=blue] ";
                    }

                }
                else
                {
                    out << "[label=\"" << boost::get(name,v) << "\"]";
                    out << "[color=white] ";
                }

         }
    private:
         Name name;
         Filter filter;
         Filter2 filter2;
    };

    /* Maps for constructing DTMC */
    // Experinence = S,A,R,S',flag-for-safety
    typedef typename boost::tuples::tuple<int,int,double,int,bool> experience;

    typedef boost::tuples::tuple<int,int> IntIntTuple;
    typedef boost::tuples::tuple<int,int,int> IntIntIntTuple;

    struct ihash
        : std::unary_function<IntIntTuple, std::size_t>
    {
        std::size_t operator()(IntIntTuple const& e) const
        {
            std::size_t seed = 0;
            boost::hash_combine( seed, e.get<0>() );
            boost::hash_combine( seed, e.get<1>() );

            return seed;
        }
    };

    struct ihashTriple
        : std::unary_function<IntIntIntTuple, std::size_t>
    {
        std::size_t operator()(IntIntIntTuple const& e) const
        {
            std::size_t seed = 0;
            boost::hash_combine( seed, e.get<0>() );
            boost::hash_combine( seed, e.get<1>() );
            boost::hash_combine( seed, e.get<2>() );
            return seed;
        }
    };

    struct iequal_to_triple
        : std::binary_function<IntIntIntTuple, IntIntIntTuple, bool>
    {
        bool operator()(IntIntIntTuple const& x, IntIntIntTuple const& y) const
        {

            return (x.get<0>()==y.get<0>() &&
                    x.get<1>()==y.get<1>() &&
                    x.get<2>()==y.get<2>());
        }
    };

    struct iequal_to
        : std::binary_function<IntIntTuple, IntIntTuple, bool>
    {
        bool operator()(IntIntTuple const& x, IntIntTuple const& y) const
        {

            return (x.get<0>()==y.get<0>() &&
                    x.get<1>()==y.get<1>());
        }
    };


    typedef boost::tuples::tuple<int,int,std::vector<IntIntTuple> > pathParameters;

    typedef boost::tuples::tuple<int,
                                int,
                                int,
                                bool> TransitTuple;/* <s,a,s',unsafe> and TransitMap stores count of these */
    struct ihashT
        : std::unary_function<TransitTuple, std::size_t>
    {
        std::size_t operator()(TransitTuple const& e) const
        {
            std::size_t seed = 0;
            boost::hash_combine( seed, e.get<0>() );
            boost::hash_combine( seed, e.get<1>() );
            boost::hash_combine( seed, e.get<2>() );
            boost::hash_combine( seed, e.get<3>() );

            return seed;
        }
    };

    struct iequal_toT
        : std::binary_function<TransitTuple, TransitTuple, bool>
    {
        bool operator()(TransitTuple const& x, TransitTuple const& y) const
        {
            return (x.get<0>()==y.get<0>() &&
                    x.get<1>()==y.get<1>() &&
                    x.get<2>()==y.get<2>() &&
                    x.get<3>()==y.get<3>());
        }
    };

    typedef boost::tuples::tuple<int> InitTuple;

    struct ihashInit
        : std::unary_function<InitTuple, std::size_t>
    {
        std::size_t operator()(InitTuple const& e) const
        {
            std::size_t seed = 0;
            boost::hash_combine( seed, e.get<0>() );

            return seed;
        }
    };

    struct iequal_to_init
        : std::binary_function<InitTuple, InitTuple, bool>
    {
        bool operator()(InitTuple const& x, InitTuple const& y) const
        {
            return (x.get<0>()==y.get<0>());
        }
    };

    // Define a map to go from vertex to state id
    typedef boost::unordered_map< InitTuple, int, ihashInit, iequal_to_init > Int2IntMap;

    // Define a map from Vertex to distance from good/bad state
    typedef boost::unordered_map< Vertex, int > Vert2IntMap;

    // Define a map to go from state id to vertex in the graph
    typedef boost::unordered_map< InitTuple, Vertex, ihashInit, iequal_to_init > Int2VerMap;

    // Define a map to go from (s,s') pair to its edge in the graph
    typedef boost::unordered_map< IntIntIntTuple, Edge, ihashTriple, iequal_to_triple > StatePair2EdgeMap;

    typedef boost::unordered_map< IntIntTuple, vector<int>*, ihash, iequal_to > StateAction2StateMap;

    typedef boost::unordered_map< IntIntTuple, int, ihash, iequal_to > StateActionVisits;

    // Loading and saving maps
    template<typename T>
    void SaveMap(const std::string& name, T mp)
    {
        std::ofstream fl(name.c_str(),std::ios_base::binary);
        boost::archive::text_oarchive oa(fl);
        boost::serialization::save(oa,mp,0);
    }

    template<typename T>
    void LoadMap(std::string& name, T mp)
    {
        std::ifstream fl(name.c_str(),std::ios_base::binary);
        boost::archive::text_iarchive oa(fl);
        boost::serialization::load(oa,mp,0);
    }

    /* End of maps */

//}


#endif // MYGRAPH_H
