/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef SARSA_H
#define SARSA_H

#include <Configuration.h>
#include <Exploration.h>
#include <Sampler.h>
#include <Pose.h>
#include <Recorder.h>

/**
 * Sarsa RL learner experiment: it tries to learn an optimal policy through SARSA algorithm.
 * The RL task is defined through a Json configuration file.
 */
class LearnerThread
{

protected:

    map<int,double*> *m_Q;
    double m_alpha;
    double m_gamma;

    int m_episodes;
    int m_length;
    int simulatedActions;

    bool m_isFirst;
    bool m_isRecordedState;
    bool m_reset;
    bool m_target;
    bool load;
    int oldInit;

    Evolution* evolution;
    Pose* pose;
    State thisState, nextState, targetState, lastRealState;
    Action thisAction, nextAction;
    Recorder graph;

    vector<int> locked;
    vector<int> overheating;
    vector<int> feedbackerror;
    vector<int> overload;

    double thisReward, cumulativeReward;
    double proximity;       // wrt obstacle(s)
    double totalTimeTaken;
    double originalTime;
    double timeSinceRestart;

    Policy *explorer;
    UniformSampler uniformSampler;

    std::string rewardFileName;

    yarp::os::Port outputQtablePort;        // a port to send learned q-table
    yarp::os::Port inputQtablePort;         // a port to receive repaired q-table

public:

    LearnerThread(Evolution* evolution, Pose* pose)
    {
        this->graph = Recorder(Config::instance()->root["Experiment"]["NRealSamples"].asInt());
        this->load = false;
        this->evolution = evolution;
        this->pose = pose;
        rewardFileName = Config::instance()->root["Reward"]["FileName"].asString();
        if (!inputQtablePort.open("/gridWorld/learner/qTable:i"))
        {
            std::cout <<": unable to open port to get input q-table \n";
        }

        if (!outputQtablePort.open("/gridWorld/learner/qTable:o"))
        {
            std::cout <<": unable to open port to output q-table \n";
        }
    }

    void InitializeLearner();

    void InitializePolicy();

    void SetAlpha(double val)
    {
        m_alpha = val;
    }
    void SetGamma(double val)
    {
        m_gamma = val;
    }
    void SetEpsilon(double val)
    {
        explorer->SetEpsilon(val);
    }

    int GetEpisodeCount()
    {
        return m_episodes;
    }

    map<int,double*>* getQPtr()
    {
        return m_Q;
    }

    bool threadInit();

    void GetAction(State &s, Action &a);

    void GetReward(int stStatus);

    void UpdateQ();

    void run(bool learn=true);

    void threadRelease();

    inline double norm(const yarp::sig::Vector &v)
    {
        return sqrt(yarp::math::dot(v,v));
    }    


    // Dump the learning on disk
    void saveLearning() {
        graph.dump(); // save the graph

        this->printQtable(); // save the Q-table

        State::printId2ValuesMap(); // save the map id-values

        // Update all the configuration parameters in order to be able to resume the learning
        Config::instance()->root["Experiment"]["EpisodesCompleted"] = (m_episodes + 1);
        Config::instance()->root["Experiment"]["SimulatedActions"] = simulatedActions;
        Config::instance()->root["Experiment"]["NSamples"] = uniformSampler.n_samples;
        Config::instance()->root["Experiment"]["StatesCounter"] = State::counter;
        ofstream overheatingFile("overheating.txt", ios::app);
        for (int i = 0; i < overheating.size(); i++) {
            overheatingFile << std::to_string(overheating[i]) << ",";
        }
        overheatingFile.close();
        ofstream overloadFile("overload.txt", ios::app);
        for (int i = 0; i < overload.size(); i++) {
            overloadFile << std::to_string(overload[i]) << ",";
        }
        overloadFile.close();
        ofstream feedbackerrorFile("feedbackerror.txt", ios::app);
        for (int i = 0; i < feedbackerror.size(); i++) {
            feedbackerrorFile << std::to_string(feedbackerror[i]) << ",";
        }
        feedbackerrorFile.close();
        Config::instance()->root["Domain"]["Locked"] = Json::Value(Json::arrayValue);
        for (int i = 0; i < locked.size(); i++) {
            Config::instance()->root["Domain"]["Locked"][i] = locked[i];
        }
        overheating.clear();
        feedbackerror.clear();
        overload.clear();
        Config::instance()->update();
    }

    // Resume the learning preciously saved
    void loadLearning() {
        // the number of episodes tells me if a previous dumping exists, and also tells
        // how much the decaying parameters must decay
        m_episodes = Config::instance()->root["Experiment"]["EpisodesCompleted"].asInt();

        // If we are resuming..
        if(m_episodes > 0 ) {
            graph.retrieve();
            readQtable();
            State::readId2ValuesMap();
            State::counter = Config::instance()->root["Experiment"]["StatesCounter"].asInt();
        }

        // Otherwise just initialize the graph
        else {
            graph.Initialize();
        }

        //Things to do in any case: set the locked joints and initialize parameters
        for (int i = 0; i < Config::instance()->root["Domain"]["Locked"].size(); i++) {
            locked.push_back(Config::instance()->root["Domain"]["Locked"][i].asInt());
        }
        Discretizer::setLocked(locked);
        m_alpha = Config::instance()->root["Learn"]["Alpha"]["init"].asDouble();
        m_gamma = Config::instance()->root["Learn"]["Gamma"]["init"].asDouble();
        explorer->SetEpsilon(Config::instance()->root["Explore"]["Epsilon"]["init"].asDouble());
        explorer->SetBeta(Config::instance()->root["Explore"]["Beta"]["init"].asDouble());
        simulatedActions = Config::instance()->root["Experiment"]["SimulatedActions"].asInt();
        uniformSampler.n_samples = Config::instance()->root["Experiment"]["NSamples"].asInt();

        for (int i = 0; i < uniformSampler.n_samples; i++) {
            uniformSampler.sample(false);
        }
        int alpha = m_episodes / Config::instance()->root["Learn"]["Alpha"]["decayAfter"].asInt();
        for (int i = 0; i < alpha; i++){
            if (m_alpha <= Config::instance()->root["Learn"]["Alpha"]["decayTill"].asDouble()) {
                break;
            }
            m_alpha *= (1.0 - Config::instance()->root["Learn"]["Alpha"]["decayBy"].asDouble());
        }


        int gamma = m_episodes / Config::instance()->root["Learn"]["Gamma"]["decayAfter"].asInt();
        for (int i = 0; i < gamma; i++){
            if (m_gamma <= Config::instance()->root["Learn"]["Gamma"]["decayTill"].asDouble()) {
                break;
            }
            m_gamma *= (1.0 - Config::instance()->root["Learn"]["Gamma"]["decayBy"].asDouble());
        }

        int epsilon = m_episodes / Config::instance()->root["Explore"]["Epsilon"]["decayAfter"].asInt();
        for (int i = 0; i < epsilon; i++) {
            if(explorer->GetEpsilon() <= Config::instance()->root["Explore"]["Epsilon"]["decayTill"].asDouble()) {
                break;
            }
            explorer->SetEpsilon(explorer->GetEpsilon() * (1.0 - Config::instance()->root["Explore"]
                                 ["Epsilon"]["decayBy"].asDouble()));
        }

        int beta = m_episodes / Config::instance()->root["Explore"]["Beta"]["decayAfter"].asInt();
        for (int i = 0; i < beta; i++) {
            if(explorer->GetBeta() <= Config::instance()->root["Explore"]["Beta"]["decayTill"].asDouble()) {
                break;
            }
            explorer->SetBeta(explorer->GetBeta() * (1.0 - Config::instance()->root["Explore"]
                              ["Beta"]["decayBy"].asDouble()));
        }
    }

    void readQtable() {
        Action a;
        std::ifstream qFile("../resources/data/qT.dat");
        if(!qFile.good())
        {
            std::cout << "Failed to read q file.\n";
            return;
        }

        m_Q = new map<int,double*>;
        int id;
        while(qFile >> id)
        {
            (*m_Q)[id] = new double[a.n_actions];
            for(int j=0; j<a.n_actions; ++j)
            {
                qFile >> (*m_Q)[id][j];
            }
        }
        std::cout << "Q-values read from file" << endl;
    }

    void printQtable()
    {
        std::ofstream qFile("../resources/data/qT.dat");
        map<int,double*>::iterator iter;
        for(iter = m_Q->begin(); iter != m_Q->end(); ++iter)
        {
            qFile << iter->first << " ";
            for(int j=0; j<thisAction.n_actions; ++j)
                qFile << iter->second[j] << " ";
            qFile << "\n";
        }
        qFile.close();

    }

};

#endif // SARSA_H
