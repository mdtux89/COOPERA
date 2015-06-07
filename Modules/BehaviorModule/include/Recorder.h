/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef RECORDER_H
#define RECORDER_H

#define NORMAL_STATE 0
#define TARGET_STATE 1
#define FALL_RESET 2
#define FEEDBACK_RESET 3
//#define OVERHEATING_RESET 4
#define OVERLOAD_RESET 5

#include <GraphDefs.h>
#include <Sampler.h>
#include <boost/serialization/vector.hpp>
#include <SerializeTuple.hpp>

/**
 * The data structure used to maintain the learning graph
 */
class Recorder {
protected:
    enum { wInit, wok, wfall, /*woverheating,*/ wfeedbackError, woverload };
    SimulationGraph     m_graph;
    VertInternalIdMap   stateIdMap;
    VertVisitsMap       nodeVisitsMap;

    UniformSampler uniformSampler;
    
    EdgeActionIdMap actionIdMap;
    EdgeWeightMap weightMap;
    EdgeVisitsMap edgeVisitsMap;

    std::pair<Edge, bool> addEdgePair;
    Vertex w0, wOk, wFall, /*wOverheating,*/ wFeedbackError, wOverload;

    Int2IntMap          vertex2state;
    Int2VerMap          state2vertex;
    StatePair2EdgeMap   statePair2edge;
    StateAction2StateMap stateAction2state;
    StateActionVisits stateActionVisits;

private:
    int nRealSamples;
    int vertexCounter;          // this counts vertices we have explored and added to the recorder

    void InsertEdgeImp(Vertex from, Vertex to, int edgeId)
    {
        IntIntTuple stateAction = boost::tuples::make_tuple(from,edgeId);
        IntIntIntTuple statePair = boost::tuples::make_tuple(vertex2state[from],vertex2state[to],edgeId);

        int counter = 0;
        if (stateActionVisits.find(stateAction) == stateActionVisits.end()) {
            counter = 1;
        }
        else {
            counter = stateActionVisits[stateAction] + 1;
        }
        stateActionVisits[stateAction] = counter;

        if(statePair2edge.find(statePair) == statePair2edge.end()) { // not exists
            // Actual edge insertion
            addEdgePair = boost::add_edge(from,to,m_graph);

            boost::put(actionIdMap,addEdgePair.first,edgeId);
            boost::put(weightMap,addEdgePair.first,1);
            boost::put(edgeVisitsMap,addEdgePair.first,1);

            // Populate corresponding map, assuming the nodes already exist
            statePair2edge[statePair] = addEdgePair.first;
            if(stateAction2state[stateAction] == NULL) {
                stateAction2state[stateAction] = new vector<int>;
            }
            stateAction2state[stateAction]->push_back(vertex2state[to]);
        }
        else { // already exists
            int counter = boost::get(edgeVisitsMap,statePair2edge[statePair]) + 1;
            boost::put(edgeVisitsMap,statePair2edge[statePair],counter);
        }
    }

    void InsertVertexImp(Vertex& inserted,int stateId)
    {
        if(state2vertex.find(stateId) == state2vertex.end()) { //not exists
            inserted = boost::add_vertex(vertexCounter, m_graph);
            boost::put(stateIdMap,inserted,vertexCounter);
            boost::put(nodeVisitsMap,inserted,1);
            vertexCounter++;
            // set the maps
            state2vertex[stateId] = inserted;
            vertex2state[inserted] = stateId;
        }
        else { // exists
            int counter = boost::get(nodeVisitsMap,state2vertex[stateId]) + 1;
            boost::put(nodeVisitsMap,state2vertex[stateId],counter);
        }
    }

public:

    Recorder() {
    }

    Recorder(int nRealSamples) {
        this->nRealSamples = nRealSamples;
        m_graph = SimulationGraph(0);
    }

    Recorder(const Recorder &rec) {
        this->state2vertex = rec.state2vertex;
        this->vertex2state = rec.vertex2state;
        this->stateAction2state = rec.stateAction2state;
        this->stateActionVisits = rec.stateActionVisits;
        this->statePair2edge = rec.statePair2edge;
        this->m_graph = rec.m_graph;
    }

    Recorder& operator= (const Recorder &rec) {\
        this->state2vertex = rec.state2vertex;
        this->vertex2state = rec.vertex2state;
        this->stateAction2state = rec.stateAction2state;
        this->stateActionVisits = rec.stateActionVisits;
        this->statePair2edge = rec.statePair2edge;
        this->m_graph = rec.m_graph;
        return *this;
    }

    void Initialize()
    {
        // Define maps for properties on the graph
        m_graph.clear();

        stateIdMap      = boost::get(vertex_state_id,m_graph);
        nodeVisitsMap   = boost::get(vertex_count,m_graph);

        actionIdMap     = boost::get(edge_action_id,m_graph);
        weightMap       = boost::get(boost::edge_weight,m_graph);
        edgeVisitsMap   = boost::get(edge_count,m_graph);

        // Insert 6 invariant nodes, i.e init, safe-sink and unsafe sink
        vertexCounter = 0;

        InsertVertexImp(w0,wInit);
        InsertVertexImp(wOk,wok);
        InsertVertexImp(wFall,wfall);
        //InsertVertexImp(wOverheating,woverheating);
        InsertVertexImp(wFeedbackError,wfeedbackError);
        InsertVertexImp(wOverload,woverload);

        //Add self-loops to these sinks
        addEdgePair = add_edge(wOk,wOk,m_graph);
        boost::put(actionIdMap,addEdgePair.first,-2);
        boost::put(weightMap,addEdgePair.first,1);
        boost::put(edgeVisitsMap,addEdgePair.first,1);

        addEdgePair = add_edge(wFall,wFall,m_graph);
        boost::put(actionIdMap,addEdgePair.first,-2);
        boost::put(weightMap,addEdgePair.first,1);
        boost::put(edgeVisitsMap,addEdgePair.first,1);

        /*addEdgePair = add_edge(wOverheating,wOverheating,m_graph);
        boost::put(actionIdMap,addEdgePair.first,-2);
        boost::put(weightMap,addEdgePair.first,1);
        boost::put(edgeVisitsMap,addEdgePair.first,1);*/

        addEdgePair = add_edge(wFeedbackError,wFeedbackError,m_graph);
        boost::put(actionIdMap,addEdgePair.first,-2);
        boost::put(weightMap,addEdgePair.first,1);
        boost::put(edgeVisitsMap,addEdgePair.first,1);

        addEdgePair = add_edge(wOverload,wOverload,m_graph);
        boost::put(actionIdMap,addEdgePair.first,-2);
        boost::put(weightMap,addEdgePair.first,1);
        boost::put(edgeVisitsMap,addEdgePair.first,1);
    }

    void dump() {
        this->SaveTheGraph("graph");

        Viter _v,_vend;
        boost::tuples::tie(_v, _vend) = vertices(m_graph);
        vector<tuple<int, int, int, int, int>> edges;
        vector<tuple<int, int>> vertices;
        vector<tuple<int, int, int>> sAVisits;

        for (; _v != _vend; ++_v) {

            int sId = vertex2state[boost::get(stateIdMap, *_v)];
            int v = boost::get(nodeVisitsMap, *_v);
            vertices.push_back(make_tuple(sId,v));

            std::pair<oe_iter, oe_iter> outEdges = boost::out_edges(*_v, m_graph);
            oe_iter itrEd = outEdges.first;
            for(; itrEd!= outEdges.second; ++itrEd) {
                int from = boost::source(*itrEd,m_graph);
                int to = boost::target(*itrEd,m_graph);
                int aId = boost::get(actionIdMap,*itrEd);
                IntIntTuple stateAction = boost::make_tuple(from,aId);
                int saV = stateActionVisits[stateAction];
                sAVisits.push_back(make_tuple(from,aId,saV));
                int w = boost::get(weightMap,*itrEd);
                int v = boost::get(edgeVisitsMap,*itrEd);
                edges.push_back(make_tuple(from, to, aId, w, v));
            }
        }

        std::ofstream ofs("edges.txt");
        boost::archive::text_oarchive oe(ofs);
        oe << edges;
        ofs.close();

        ofs.open("vertices.txt");
        boost::archive::text_oarchive ov(ofs);
        ov << vertices;
        ofs.close();

        ofs.open("visits.txt");
        boost::archive::text_oarchive ovi(ofs);
        ovi << sAVisits;
        ofs.close();
    }

    void retrieve() {
        this->Initialize();

        vector<tuple<int, int, int, int, int>> edges;
        vector<tuple<int, int>> vertices;
        vector<tuple<int, int, int>> sAVisits;

        std::ifstream ifs("edges.txt");
        if (!ifs.is_open() || ifs.bad()) {
            return;
        }
        boost::archive::text_iarchive ie(ifs);
        ie >> edges;
        ifs.close();

        ifs.open("vertices.txt");
        if (!ifs.is_open() || ifs.bad()) {
            return;
        }
        boost::archive::text_iarchive iv(ifs);
        iv >> vertices;
        ifs.close();

        ifs.open("visits.txt");
        if (!ifs.is_open() || ifs.bad()) {
            return;
        }
        boost::archive::text_iarchive ivi(ifs);
        ivi >> sAVisits;
        ifs.close();


        for (int i = 0; i < vertices.size(); i++) {
            int sId = std::get<0>(vertices[i]);
            int visits = std::get<1>(vertices[i]);
            Vertex v;
            InsertVertexImp(v,sId);
            boost::put(nodeVisitsMap,state2vertex[sId],visits);
        }

        for (int i = 0; i < edges.size(); i++) {
            int from = std::get<0>(edges[i]);
            if (from == wFall || from == wFeedbackError || /*from == wOverheating ||*/ from == wOverload || from == wOk) {
                continue;
            }
            int to = std::get<1>(edges[i]);
            int aId = std::get<2>(edges[i]);
            int w = std::get<3>(edges[i]);
            int v = std::get<4>(edges[i]);
            InsertEdgeImp(from,to,aId);
            IntIntIntTuple statePair = boost::tuples::make_tuple(vertex2state[from],vertex2state[to],aId);
            boost::put(weightMap,statePair2edge[statePair],w);
            boost::put(edgeVisitsMap,statePair2edge[statePair],v);
        }

        for (int i = 0; i < sAVisits.size(); i++) {
            int from = std::get<0>(sAVisits[i]);
            int act = std::get<1>(sAVisits[i]);
            int visits = std::get<2>(sAVisits[i]);
            stateActionVisits[boost::make_tuple(from,act)] = visits;
        }
    }

    // Utility used when reconstructing the graph after joint releasing.
    // It returns the info of all the edges in the graph.
    vector<tuple<int, int, int>> getEdges() {
         Viter _v,_vend;
         boost::tuples::tie(_v, _vend) = vertices(m_graph);
         vector<tuple<int, int, int>> edges;
         for (; _v != _vend; ++_v) {
             std::pair<oe_iter, oe_iter> outEdges = boost::out_edges(*_v, m_graph);
             oe_iter itrEd = outEdges.first;
             for(; itrEd!= outEdges.second; ++itrEd) {
                 int from = vertex2state[boost::source(*itrEd,m_graph)];
                 int to = vertex2state[boost::target(*itrEd,m_graph)];
                 int aId = boost::get(actionIdMap,*itrEd);
                 edges.push_back(make_tuple(from, to, aId));
             }
         }
        return edges;
    }

    list<int> shortestPath (int from, int to) {
        typedef boost::property_map < SimulationGraph, boost::vertex_index_t >::type IndexMap;
        typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
        typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;

        std::vector<Vertex> predecessors(boost::num_vertices(m_graph));
        std::vector<Weight> distances(boost::num_vertices(m_graph));
        IndexMap indexMap = boost::get(boost::vertex_index, m_graph);
        PredecessorMap predecessorMap(&predecessors[0], indexMap);
        DistanceMap distanceMap(&distances[0], indexMap);

        boost::dijkstra_shortest_paths(m_graph, state2vertex[from],
                                       boost::distance_map(distanceMap).predecessor_map(predecessorMap));

        Vertex v = state2vertex[to]; // We want to start at the destination and work our way back to the source
        list<int> path;
        for(Vertex u = predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
            u != v; // Keep tracking the path until we get to the source
            v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
        {
          std::pair<SimulationGraph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
          SimulationGraph::edge_descriptor edge = edgePair.first;
          path.push_front( boost::get(actionIdMap, edge) );
        }
        return path;
    }

    bool InsertNode(int stateId)
    {
        Vertex t;
        InsertVertexImp(t,stateId);
        return true;
    }

    bool isExploredStateAction(int stateId, int actionId) {
        IntIntTuple stateAction = boost::tuples::make_tuple(state2vertex[stateId],actionId);
        if (stateActionVisits.find(stateAction) == stateActionVisits.end()) {
            cout << "Apply in state " << state2vertex[stateId] << endl;
            return false;
        }
        int counter = stateActionVisits[stateAction];
        if (counter <= nRealSamples) cout << "Apply in state " << state2vertex[stateId] << " (" << counter << ")" << endl;
        else cout << "Simulate in state " << state2vertex[stateId] << endl;
        return (counter > nRealSamples);
    }

    int getNextProbState(int stateId, int actionId) {
        IntIntTuple stateAction = boost::tuples::make_tuple(state2vertex[stateId],actionId);
        vector<double> p;
        for (int i = 0; i < stateAction2state[stateAction]->size(); i++) {
            int j = boost::get(edgeVisitsMap,boost::edge(state2vertex[stateId],
                                                  state2vertex[(*stateAction2state[stateAction])[i]],m_graph).first);
            double pr = double(stateActionVisits[stateAction] / j);
            p.push_back(pr);
        }

        double sample   = uniformSampler.sample();
        double currentSum   = 0;
        int i = 1;
        for (; i < p.size(); i++) {
            if (currentSum > sample) {
                break;
            }
            currentSum += p[i];
        }
        i--;

        return (*stateAction2state[stateAction])[i];
    }

    int check(int from, int target) {
        int next = state2vertex[getNextProbState(from,-2)];
        /*if (next == wOverheating) {
            cout << "OVERHEATING (simulated)" << endl;
            return OVERHEATING_RESET;
        }*/
        if (next == wFeedbackError) {
            cout << "FEEDBACK ERROR (simulated)" << endl;
            return FEEDBACK_RESET;
        }
        else if (next == wOverload) {
            cout << "OVERLOAD (simulated)" << endl;
            return OVERLOAD_RESET;
        }
        else if (next == wFall) {
            return FALL_RESET;
        }
        else if (from == target) {
            cout << "TARGET (simulated)" << endl;
            return TARGET_STATE;
        }
        else {
            return NORMAL_STATE;
        }

    }

    bool InsertEdge(int stateFrom, int stateTo,  int actionId = 0)
    {
        // Normal case
        InsertEdgeImp(state2vertex[stateFrom],state2vertex[stateTo],actionId);

    }

    bool InsertEdgeFromInit(int stateTo,  int actionId = 0)
    {
        InsertEdgeImp(w0,state2vertex[stateTo],actionId);// initial state cant be unsafe
    }

    bool InsertEdgeToSafeSink(int stateFrom,  int actionId = 0)
    {
        InsertEdgeImp(state2vertex[stateFrom],wOk,actionId); // safe sink state cant be unsafe
    }

    bool InsertEdgeToFallSink(int stateFrom,  int actionId = 0)
    {
        InsertEdgeImp(state2vertex[stateFrom],wFall,actionId); // unsafe sink state is always unsafe
    }

    /*bool InsertEdgeToOverHeatingSink(int stateFrom,  int actionId = 0)
    {
        InsertEdgeImp(state2vertex[stateFrom],wOverheating,actionId); // unsafe sink state is always unsafe
    }*/

    bool InsertEdgeToOverLoadSink(int stateFrom,  int actionId = 0)
    {
        InsertEdgeImp(state2vertex[stateFrom],wOverload,actionId); // unsafe sink state is always unsafe
    }

    bool InsertEdgeToFeedbackErrorSink(int stateFrom,  int actionId = 0)
    {
        InsertEdgeImp(state2vertex[stateFrom],wFeedbackError,actionId); // unsafe sink state is always unsafe
    }

    Vertex GetVertex(int state)
    {
        return state2vertex[state];
    }

    void SaveTheGraph(std::string withName)
    {
        std::string totalName(withName);
        totalName = totalName + ".dot";

        std::ofstream graphVizFile(totalName.c_str());


        myVertexWriter<VertInternalIdMap,VertInternalIdMap> vW(stateIdMap,stateIdMap);
        myEdgeWriter<EdgeActionIdMap,EdgeWeightMap,EdgeActionIdMap > eW(actionIdMap,weightMap,actionIdMap);

        boost::write_graphviz(graphVizFile, m_graph,
                              vW,//boost::default_writer(),
                              eW,//boost::make_label_writer(boost::get(edge_propensity,G)),
                              boost::default_writer(),
                              boost::get(vertex_state_id,m_graph));

    }

    SimulationGraph& Graph()
    {
        return m_graph;
    }
};

#endif // RECORDER_H
