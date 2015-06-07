/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include "Sarsa.h"
using namespace std;

/**
 * Read the parameters and initializes the learner
 */
void LearnerThread::InitializeLearner()
{

    m_alpha = Config::instance()->root["Learn"]["Alpha"]["init"].asDouble();
    m_gamma = Config::instance()->root["Learn"]["Gamma"]["init"].asDouble();
    explorer->SetEpsilon(Config::instance()->root["Explore"]["Epsilon"]["init"].asDouble());
    explorer->SetBeta(Config::instance()->root["Explore"]["Beta"]["init"].asDouble());

    cout << "No Q-table in database. Assiging all values to  "<<
            Config::instance()->root["Experiment"]["InitialQValue"].asDouble() <<" .\n";

    map<int,double*>::iterator iter;
    for(iter = m_Q->begin(); iter!=m_Q->end(); ++iter)
    {
        delete [] iter->second;
    }
    m_Q->erase(m_Q->begin(),m_Q->end());

    originalTime = 0;
    m_isRecordedState = false;

    if(m_isRecordedState)
        timeSinceRestart = yarp::os::Time::now();

    explorer->initPolicy(m_Q);

    cout << "Have initialized Q table for "
         << thisState.m_states << " states and "
         << thisAction.m_all_actions << " actions. \n";

    m_episodes = 0;
    m_length = 0;
    cumulativeReward = 0;

    cout<<"Learner initialized!\n";
}

/**
 * Initializes the policy to be used and set the initial states and actions
 * @return true if initialization is properly accomplished
 */
bool LearnerThread::threadInit()
{
    m_Q = new map<int,double*>;

    //*m_runArm = false;
    m_learnPhase = true;

    explorer = new Policy();
    explorer->initPolicy(m_Q);



    m_episodes = 0;
    m_length = 0;
    m_isFirst = true;
    totalTimeTaken = 0;

    // Set initial states
    vector<int> v;
    for(int j=0;j<Config::instance()->root["Domain"]["Initial"].size(); j++){
        v.clear();
        for(int i=0; i<thisState.size; i++){
            v.push_back(Config::instance()->root["Domain"]["Initial"][to_string(j)][i].asInt());
        }
        resetStates.push_back(State(v));
    }

    //Set target state
    v.clear();
    for(int i=0;i<thisState.size;i++){
        v.push_back(Config::instance()->root["Domain"]["Target"][i].asInt());
    }
    targetState = State(v);

    //Choose a random initial state and a action set accordingly (the relevant ones for that initial state)
    relevantIndex = int(uniformSampler.sample() * (Config::instance()->root["Domain"]["Initial"].size()));
    thisState = resetStates[relevantIndex];
    for(int j=0; j<Config::instance()->root["Domain"]["RelevanceSets"].size(); j++){
        v.clear();
        for(int i=0; i<Config::instance()->root["Domain"]["RelevanceSets"][to_string(j)].size(); i++){
            v.push_back(Config::instance()->root["Domain"]["RelevanceSets"][to_string(j)][i].asInt());
        }
        if(Config::instance()->root["Domain"]["ReverseAction"].asBool()==true){
            v.push_back(Config::instance()->root["Domain"]["Actions"].size()); //for the reverse action
        }
        relevantSets.push_back(v);
    }

    //Choose a random action
    thisAction = Action(relevantSets[relevantIndex][int(uniformSampler.sample() * (relevantSets.size()))]);

    std::cout<<"Learner constructed for first time!\n";
    return true;

}

/**
 * Given the current state infers the action to choose using the learned Q-values
 * @param s the current state
 * @param a the chosen action
 */
void LearnerThread::GetAction(State& s, Action& a)
{
    explorer->explore(s,a,relevantSets[relevantIndex]);
}

/**
 * Computes the reward for the given state
 * @param st the given state
 */
void LearnerThread::GetReward(State& st)
{
    Json::Value rwdConf = Config::instance()->root["Reward"];

    thisReward = rwdConf["Action"].asDouble();
    proximity = targetState.distance(st);

    int stStatus = st.check();
    if( stStatus == STATE_OUT_OF_BOUND )
    {
            thisReward += rwdConf["Reset"].asDouble();
            m_reset = true;
    }
    else if (  stStatus == STATE_UNSAFE )
    {
        thisReward += rwdConf["Unsafe"].asDouble();
        m_reset = true;
    }
    else if(stStatus == STATE_TARGET)
    {
        thisReward = rwdConf["Goal"].asDouble();
        m_reset = true;
    }

    // Adjust by factor
    thisReward *= rwdConf["Factor"].asDouble();

    cumulativeReward += thisReward;

    cout<<"Reward calculated: "<<thisReward<<"\n";
}

/**
 * Update the Q-values using the Sarsa update rule
 */
void LearnerThread::UpdateQ()
{
    if((*m_Q)[thisState.m_id]==NULL){
        (*m_Q)[thisState.m_id] = new double[thisAction.m_all_actions];
        for(int i=0 ; i<thisAction.m_all_actions; i++){
            (*m_Q)[thisState.m_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
        }
    }
    if((*m_Q)[nextState.m_id]==NULL){
        (*m_Q)[nextState.m_id] = new double[nextAction.m_all_actions];
        for(int i=0 ; i<nextAction.m_all_actions; i++){
            (*m_Q)[nextState.m_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
        }
    }

    if(m_reset)
    {
        (*m_Q)[thisState.m_id][thisAction.id()] +=
                m_alpha*( thisReward - (*m_Q)[thisState.m_id][thisAction.id()]);

    }
    else
    {
        (*m_Q)[thisState.m_id][thisAction.id()] +=
                m_alpha*(thisReward + m_gamma*
                         ((*m_Q)[nextState.m_id][nextAction.id()] - (*m_Q)[thisState.m_id][thisAction.id()]));
    }

    cout<<"Q-value at s="<<thisState.m_id<<
               " a="<<thisAction.id()<<" updated to " <<
          (*m_Q)[thisState.m_id][thisAction.id()] << ".\n";
}

/**
 * Check the new state and reset when necessary
 * @param st the previous state
 * @param act the action to employ to generate the new state
 * @return true if new state properly managed
 */
bool LearnerThread::DoAction(State& st, Action& act)
{
    State tmp;
    act.apply(st,tmp);

    switch(tmp.check())
    {
    case STATE_NORMAL:
        m_reset = false;
        nextState = tmp;
        return true;
    case STATE_INIT:
        m_reset = false;
        nextState = tmp;
        return true;
    case STATE_UNSAFE:
        m_reset = true;
        nextState = tmp;
        break;
    case STATE_TARGET:
        m_reset = true;
        nextState = tmp;
        break;
    case STATE_OUT_OF_BOUND:
        m_reset = true;
        nextState = tmp;
        break;
    default:
        m_reset = false;
        return false;
    }

    return false;

}

/**
 * Run one iteration of Sarsa
 */
void LearnerThread::run(bool learn)
{
    if(m_isFirst)
    {
        m_isFirst = false;
        GetAction(thisState,thisAction);
    }

    DoAction(thisState, thisAction);

    // Obtain action for this state
    GetAction(nextState,nextAction);

    // Update time-taken
    totalTimeTaken += 1;

    GetReward(nextState);

    cout << "---";
    cout << endl;
    list<int>::const_iterator iterator2;
    for (iterator2 = nextState.previousActions.begin(); iterator2 != nextState.previousActions.end(); ++iterator2) {
        cout << *iterator2 << " ";
    }
    cout << endl;
    cout << "---";
    cout << endl;

    // Update the Q value if in learn phase
    if(m_learnPhase)
    {
        // Assuming both this-state and next-state as well as this-action
        // and next-action are known by now
        UpdateQ();
    }
    if(m_length > Config::instance()->root["State"]["MaxLengthFactor"].asDouble() * sqrt(thisState.m_states) )
    {
        m_reset = true;
    }

    if(m_reset)
    {
        cout << "*** world reset *** \n Episode#: "
             <<m_episodes<<" \n";

        m_isFirst = true;
        relevantIndex = int(uniformSampler.sample() * Config::instance()->root["Domain"]["Initial"].size());
        thisState = resetStates[relevantIndex];
        totalTimeTaken = 0;
        m_length = 0;
        ++m_episodes;

        if(m_episodes % (Config::instance()->root["Experiment"]["AverageFor"].asInt()) == 0)
        {
            printQtable();
            ofstream rewFile;
            rewFile.open(rewardFileName.c_str(),ios_base::app);
            rewFile << m_episodes<<","<< cumulativeReward/(Config::instance()->root["Experiment"]["AverageFor"].asInt()) << "\n";
            rewFile.close();
            cumulativeReward = 0;
        }
        if(learn)
        {
            if(m_episodes % Config::instance()->root["Learn"]["Alpha"]["decayAfter"].asInt() == 0
                    && m_alpha > Config::instance()->root["Learn"]["Alpha"]["decayTill"].asDouble())
            {
                m_alpha *= (1.0 - Config::instance()->root["Learn"]["Alpha"]["decayBy"].asDouble());

            }
            if(m_episodes % Config::instance()->root["Learn"]["Gamma"]["decayAfter"].asInt() == 0
                    && m_gamma > Config::instance()->root["Learn"]["Gamma"]["decayTill"].asDouble())
            {
                m_gamma *= (1.0 - Config::instance()->root["Learn"]["Gamma"]["decayBy"].asDouble());

            }
            if(m_episodes % Config::instance()->root["Explore"]["Epsilon"]["decayAfter"].asInt() == 0
                    && explorer->GetEpsilon() > Config::instance()->root["Explore"]["Epsilon"]["decayTill"].asDouble())
            {
                explorer->SetEpsilon(explorer->GetEpsilon() * (1.0 - Config::instance()->root["Explore"]["Epsilon"]["decayBy"].asDouble()));
            }
            if(m_episodes % Config::instance()->root["Explore"]["Beta"]["decayAfter"].asInt() == 0
                    && explorer->GetBeta() > Config::instance()->root["Beta"]["Epsilon"]["decayTill"].asDouble())
            {
                explorer->SetBeta(explorer->GetBeta() * (1.0 - Config::instance()->root["Explore"]["Beta"]["decayBy"].asDouble()));
            }
        }

    }
    else
    {
        ++m_length;
        thisState = nextState;
        thisAction = nextAction;
    }


}

/**
 * Delete the Q-table and release the thread
 */
void LearnerThread::threadRelease()
{
    cout<<"Releasing learner thread...\n";

    delete explorer;

    map<int,double*>::iterator iter;
    for(iter = m_Q->begin(); iter!=m_Q->end(); ++iter)
    {
        delete [] iter->second;
    }
    m_Q->erase(m_Q->begin(),m_Q->end());
    cout<<"Done with releasing learner thread.\n";

}
