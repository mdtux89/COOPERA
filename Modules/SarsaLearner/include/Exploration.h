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
#include <StatesActions.h>
#include <algorithm>

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
    vector<int> vect;

    typedef boost::minstd_rand base_generator_type;
    base_generator_type genUniRand01;
    boost::uniform_real<> randReal01;
    boost::variate_generator<base_generator_type&, boost::uniform_real<> >* getRand01;

    /**
     * Return an action using the greedy policy
     * @param st the input state
     * @param ac the output action
     */
    void GreedyDeterministicAction(State& st, Action& ac)
    {
        if((*m_QPtr)[st.m_id]==NULL){
            (*m_QPtr)[st.m_id] = new double[ac.m_all_actions];
            for(int i=0 ; i<ac.m_all_actions; i++){
                (*m_QPtr)[st.m_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
            }
        }

        double maxQ  = (*m_QPtr)[st.m_id][vect[0]];

        for(int i=1 ; i<vect.size(); i++)
        {
            double thisVal = (*m_QPtr)[st.m_id][vect[i]];
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

            for(int i=0 ; i<vect.size(); i++)
            {
                double diff = (maxQ - (*m_QPtr)[st.m_id][vect[i]]);
                if(diff > -EPSILON && diff < EPSILON)
                {
                   ac = Action(vect[i]);
                   return;
                }
            }

            ac = Action(vect[0]);
            return;
        }

    }

    /**
     * Select a random action
     * @param ac the selected action
     */
    inline void RandomUniformAction(Action& ac)
    {
        ac = Action(vect[(*getRand01)() * vect.size()]);//GET_UNIFORM_ACTION;
    }

    /**
     * Return an action using the non-deterministic greedy policy
     * @param st the input state
     * @param ac the output action
     */
    void GreedyNonDeterministicAction(State& st, Action& ac)
    {
        if((*m_QPtr)[st.m_id]==NULL){
            (*m_QPtr)[st.m_id] = new double[ac.m_all_actions];
            for(int i=0 ; i<ac.m_all_actions; i++){
                (*m_QPtr)[st.m_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
            }
        }

        double maxQ  = (*m_QPtr)[st.m_id][vect[0]] ;

        for(int i=1 ; i<vect.size(); i++)
        {
            double thisVal = (*m_QPtr)[st.m_id][vect[i]];
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
            maximalQActions.reserve(vect.size());
            int countOfMaximalActions=0;

            for(int i=0 ; i<vect.size(); i++)
            {
                double diff = (maxQ - (*m_QPtr)[st.m_id][vect[i]]);
                if(diff > -EPSILON && diff < EPSILON)
                {
                   maximalQActions.push_back(vect[i]);
                   countOfMaximalActions++;
                }
            }

            ac = Action(maximalQActions[ (int) ( (countOfMaximalActions)*(*getRand01)() ) ]) ;
        }
    }

    /**
     * Return an action using the softmax policy
     * @param st the input state
     * @param ac the output action
     */
    void SoftmaxAction(State& st, Action& ac)
    {
        if((*m_QPtr)[st.m_id]==NULL){
            (*m_QPtr)[st.m_id] = new double[ac.m_all_actions];
            for(int i=0 ; i<ac.m_all_actions; i++){
                (*m_QPtr)[st.m_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
            }
        }

        double propensities[vect.size()];
        double sumOfPropensity  =   0;

        for(int i=0 ; i<vect.size(); i++)
        {
            propensities[i] = exp((*m_QPtr)[st.m_id][vect[i]]/m_beta);
            sumOfPropensity    += propensities[i];
        }

        for(int i=0 ; i<vect.size(); i++)
        {
            propensities[i] /= sumOfPropensity;
        }

        double sample   = (*getRand01)();
        double currentSum   = propensities[0];

        int i=1 ;
        for(; i<vect.size(); i++)
        {
            if(currentSum > sample)
            {
                break;
            }
            currentSum += propensities[i];
        }
        ac = Action(vect[i-1]);
    }


public:

    Policy() : genUniRand01(EXPLORATION_RANDOM_SEED),randReal01(0,1)
    {
        //srand48(1);
        getRand01 = new boost::variate_generator<base_generator_type&, boost::uniform_real<> >(genUniRand01,randReal01);
        std::cout<<"First nbr from explore: "<< (*getRand01)()<<"\n";
    }
    virtual ~Policy()
    {
        delete getRand01;
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
     * @param relevants the set of relevant actions
     */
    void explore(State& st, Action& ac, vector<int> relevants)
    {
        vect.clear();
        for(int i=0;i<ac.m_all_actions;i++){
            if(find(st.previousActions.begin(),st.previousActions.end(),i)==st.previousActions.end() &&
                    find(relevants.begin(),relevants.end(),i)!=relevants.end()){
                vect.push_back(i);
            }
        }
        double smpl = (*getRand01)();
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
        return (*getRand01)();
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
