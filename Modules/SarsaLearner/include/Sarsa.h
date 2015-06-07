/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef SARSA_H
#define SARSA_H

#include <Configuration.h>
#include <StatesActions.h>
#include <Exploration.h>
#include <Sampler.h>

/**
 * Use Sarsa algorithm to build the Q-table used in modules SarsaPolicy and MotorsController.
 * Those values are used to learn an optimal policy for the robot to follow.
 */
class LearnerThread
{
protected:

    map<int,double*> *m_Q;
    double m_alpha;
    double m_gamma;

    int m_episodes;
    int m_length;

    bool m_isFirst;
    bool m_isRecordedState;
    bool m_reset;
    bool m_learnPhase;

    State thisState, nextState, targetState;
    vector<State> resetStates;
    vector<vector<int>> relevantSets;
    Action thisAction, nextAction;
    int pixelSize;

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

    LearnerThread()
    {
        rewardFileName = Config::instance()->root["Reward"]["FileName"].asString();
        ofstream rewFile;
        rewFile.open(rewardFileName.c_str(),ios_base::app);
        rewFile << "Episode,Reward" << "\n";
        rewFile.close();
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



    void SetAlpha(double val)
    {
        m_alpha = val;
    }
    void SetGamma(double val)
    {
        m_gamma = val;
    }
    void SetLearnPhase(bool val)
    {
        m_learnPhase = val;
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

    void GetReward(State &st);

    void UpdateQ();

    bool DoAction(State &st, Action &act);

    void run(bool learn=true);

    void threadRelease();

    inline double norm(const yarp::sig::Vector &v)
    {
        return sqrt(yarp::math::dot(v,v));
    }    

    bool SendQtable()
    {
        Json::Value qFcnf = Config::instance()->root["Output"];
        if(qFcnf["QFileOut"].asString() == "")
        {
            yarp::os::Bottle b;
            b.clear();
            State s; Action a;
            map<int,double*>::iterator iter;
            for(iter = m_Q->begin(); iter != m_Q->end(); ++iter)
            {
                for(int j=0; j<a.m_all_actions; ++j)
                {
                    b.addDouble(iter->second[j]);
                }
            }

            bool ret    = outputQtablePort.write(b);

            return ret;

        }
        else
        {
            State s; Action a;
            std::stringstream qFileName;
            qFileName << qFcnf["QFileOut"].asString() << ".qfile";

            std::ofstream qFile(qFileName.str().c_str());

            map<int,double*>::iterator iter;
            for(iter = m_Q->begin(); iter != m_Q->end(); ++iter)
            {
                for(int j=0; j<a.m_all_actions; ++j)
                {
                    qFile << iter->second[j] << "  " ;
                }
                qFile << " \n";
            }

            qFile.close();

            return true;

        }

    }

    void printQtable()
    {
        std::ofstream qFile("../resources/data/qT.dat");
        map<int,double*>::iterator iter;
        for(iter = m_Q->begin(); iter != m_Q->end(); ++iter)
        {
            qFile << iter->first << " ";
            for(int j=0; j<thisAction.m_all_actions; ++j)
                qFile << iter->second[j] << " ";
            qFile << "\n";
        }
        qFile.close();

    }

};

#endif // SARSA_H
