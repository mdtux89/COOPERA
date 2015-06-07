/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef EXPLORATION_H
#define EXPLORATION_H
#define EXPLORATION_RANDOM_SEED 13

#include <Configuration.h>
#include <Evolution.h>
#include <algorithm>
#include <vector>
#include <limits>
#include <string>
#include <list>
#include <cmath>
#include <iostream>
using namespace std;

namespace PolicyFollowed
{
    enum { softmax, greedy , greedyNondeterministic};
}

/**
 * Defines the possible exploitation strategies for Sarsa RL experiments.
 * The options are Greedy, Non-deterministic Greedy and Softmax.
 */
class Policy
{
protected:

    double m_epsilon,m_beta;    // This is epsilon now
    map<int,double*> *m_QPtr;       // Pointer to Q-table data
    vector<int> locked;

    UniformSampler* uniformSampler;

    /**
     * Return an action using the greedy policy
     * @param st the input state
     * @param ac the output action
     */
    void GreedyDeterministicAction(State& st, Action& ac)
    {
        if((*m_QPtr)[st.state_id]==NULL){
            (*m_QPtr)[st.state_id] = new double[ac.n_actions];
            for(int i=0 ; i<ac.n_actions; i++){
                (*m_QPtr)[st.state_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
            }
        }

        double maxQ  = (*m_QPtr)[st.state_id][0];

        for(int i=1 ; i<ac.n_actions; i++)
        {
            double thisVal = (*m_QPtr)[st.state_id][i];
            if(thisVal > maxQ)
            {
                maxQ = thisVal;
            }
        }

        if(maxQ < EPSILON && maxQ > -EPSILON)
        {
            RandomUniformAction(ac);//GET_UNIFORM_ACTION;
        }
        else
        {

            for(int i=0 ; i<ac.n_actions; i++)
            {
                double diff = (maxQ - (*m_QPtr)[st.state_id][i]);
                if(diff > -EPSILON && diff < EPSILON && !Action::isLocked(i, locked))
                {
                   ac = Action(i);
                   return;
                }
            }

            for(int i=0 ; i<ac.n_actions; i++) {
                if(!Action::isLocked(i, locked)) {
                    ac = Action(i);
                    return;
                }
            }
        }

    }

    /**
     * Select a random action
     * @param ac the selected action
     */
    inline void RandomUniformAction(Action& ac)
    {
        int id = 0;
        do {
            id = uniformSampler->sample() * ac.n_actions;
        }
        while(Action::isLocked(id, locked));
        ac = Action(id);//GET_UNIFORM_ACTION;
    }

    /**
     * Return an action using the non-deterministic greedy policy
     * @param st the input state
     * @param ac the output action
     */
    void GreedyNonDeterministicAction(State& st, Action& ac)
    {
        if((*m_QPtr)[st.state_id]==NULL){
            (*m_QPtr)[st.state_id] = new double[ac.n_actions];
            for(int i=0 ; i<ac.n_actions; i++){
                (*m_QPtr)[st.state_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
            }
        }

        double maxQ  = (*m_QPtr)[st.state_id][0] ;

        for(int i=1 ; i<ac.n_actions; i++)
        {
            double thisVal = (*m_QPtr)[st.state_id][i];
            if(thisVal > maxQ)
            {
                maxQ = thisVal;
            }
        }

        if(maxQ < EPSILON && maxQ > -EPSILON)
        {
            RandomUniformAction(ac);//GET_UNIFORM_ACTION;
        }
        else
        {

            std::vector<int> maximalQActions;
            maximalQActions.reserve(ac.n_actions);
            int countOfMaximalActions=0;

            for(int i=0 ; i<ac.n_actions; i++)
            {
                double diff = (maxQ - (*m_QPtr)[st.state_id][i]);
                if(diff > -EPSILON && diff < EPSILON)
                {
                   maximalQActions.push_back(i);
                   countOfMaximalActions++;
                }
            }

            int id = 0;
            do {
                id = (int) ( (countOfMaximalActions)* uniformSampler->sample() );
            }while (Action::isLocked(id, locked));
            ac = Action(maximalQActions[id]) ;
        }
    }

    /**
     * Return an action using the softmax policy
     * @param st the input state
     * @param ac the output action
     */
    void SoftmaxAction(State& st, Action& ac)
    {
        if((*m_QPtr)[st.state_id]==NULL){
            (*m_QPtr)[st.state_id] = new double[ac.n_actions];
            for(int i=0 ; i<ac.n_actions; i++){
                (*m_QPtr)[st.state_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
            }
        }

        double propensities[ac.n_actions];
        double sumOfPropensity  =   0;

        for(int i=0 ; i<ac.n_actions; i++)
        {
            propensities[i] = exp((*m_QPtr)[st.state_id][i]/m_beta);
            sumOfPropensity    += propensities[i];
        }

        for(int i=0 ; i<ac.n_actions; i++)
        {
            propensities[i] /= sumOfPropensity;
        }

        double sample   = uniformSampler->sample();
        double currentSum   = propensities[0];

        int i=1 ;
        for(; i<ac.n_actions; i++)
        {
            if(currentSum > sample)
            {
                if(!Action::isLocked(i - 1, locked)) {
                    break;
                }
            }
            currentSum += propensities[i];
        }

        do {
            i--;
        }while(Action::isLocked(i, locked) && i > 0);

        ac = Action(i);
    }


public:

    Policy(UniformSampler* uniformSampler)
    {
       this->uniformSampler = uniformSampler;
    }

    /**
     * Initialize the policy with the Q-values
     * @param QVal the Q-values
     */
    void initPolicy(map<int,double*> *QVal)
    {
        m_QPtr = QVal;
    }

    /**
     * Get a new action to explore
     * @param st the input state
     * @param ac the output action
     */
    void explore(State& st, Action& ac, vector<int> locked)
    {
        this->locked = locked;
        double smpl = uniformSampler->sample();
        switch(Config::instance()->root["Learn"]["PolicyFollowed"].asInt())
        {
        case PolicyFollowed::greedy:
            GreedyDeterministicAction(st,ac);
            break;
        case PolicyFollowed::greedyNondeterministic:
            if(smpl < m_epsilon) RandomUniformAction(ac);
            else GreedyNonDeterministicAction(st,ac);
            break;
        case PolicyFollowed::softmax:
            SoftmaxAction(st,ac);
            break;
        default:
            GreedyDeterministicAction(st,ac);
        }
    }

    double GetUniformRand()
    {
        return uniformSampler->sample();
    }

    double GetPropensity(double QVal)
    {
        return exp(QVal/100.0);
    }

    void SetEpsilon(double val)
    {
        m_epsilon = val;
    }

    void SetBeta(double val)
    {
        m_beta = val;
    }

    double GetEpsilon()
    {
        return m_epsilon;
    }

    double GetBeta()
    {
        return m_beta;
    }


};


#endif // EXPLORATION_H
