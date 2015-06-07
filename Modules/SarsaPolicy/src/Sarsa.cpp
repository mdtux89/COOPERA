/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include "Sarsa.h"
#include "Experiment.h"
using namespace std;

/**
 * Read the parameters, open the proper resources and initializes the learner
 * @param qFileName where to read the Q-table
 * @return true if initialization is properly accomplished
 */
bool SarsaPolicyAlgo::InitializeLearner(const string qFileName)
{
    m_alpha = Config::instance()->root["Learn"]["Alpha"]["decayTill"].asDouble();
    m_gamma = Config::instance()->root["Learn"]["Gamma"]["decayTill"].asDouble();
    explorer->SetEpsilon(Config::instance()->root["Explore"]["Epsilon"]["decayTill"].asDouble());
    explorer->SetBeta(Config::instance()->root["Explore"]["Beta"]["decayTill"].asDouble());

    if (!inputQtablePort.open("/gridWorld/simulator/qTable:i"))
    {
        std::cout <<": unable to open port to get input q-table \n";
        return false;  // unable to open; let RFModule know so that it won't run

    }

    if (!outputQtablePort.open("/gridWorld/simulator/qTable:o"))
    {
        std::cout <<": unable to open port to output q-table \n";
        return false;  // unable to open; let RFModule know so that it won't run

    }

    if(qFileName == "")
    {
        std::cout << "Waiting to read Q-file from port ...\n";
        yarp::os::Bottle b;
        inputQtablePort.read(b,true);

        int c = 0;
        map<int,double*>::iterator iter;
        for(iter = m_Q->begin(); iter!=m_Q->end(); ++iter)
        {
            for(int j=0; j<thisAction.m_all_actions; ++j)
            {
                iter->second[j] = b.get(c).asDouble();
            }
        }
        std::cout << "Q-values read from port!\n";

    }
    else
    {
        cout << qFileName.c_str() << endl;
        ifstream qFile(qFileName.c_str());
        if(!qFile.good())
        {
            std::cout << "Failed to read q file.\n";
            return false;
        }

        m_Q = new map<int,double*>;
        for(int i=0; i<thisState.m_states;i++)
        {
            int id;
            qFile >> id;
            (*m_Q)[id] = new double[thisAction.m_all_actions];
            for(int j=0; j<thisAction.m_all_actions; ++j)
            {
                qFile >> (*m_Q)[id][j];
            }
        }
        std::cout << "Q-values read from file at:" <<
                     qFileName << ".\n";
    }


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
    return true;
}

/**
 * Initializes the policy to be used and set the initial states and actions
 * @return true if initialization is properly accomplished
 */
bool SarsaPolicyAlgo::threadInit()
{
    m_Q = new map<int,double*>;
    m_learnPhase = false;

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
void SarsaPolicyAlgo::GetAction(State& s, Action& a)
{
    explorer->getLearnedAction(s,a,relevantSets[relevantIndex]);
}

/**
 * Computes the reward for the given state
 * @param st the given state
 */
void SarsaPolicyAlgo::GetReward(State& st)
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
 * Check the new state and reset when necessary
 * @param st the previous state
 * @param act the action to employ to generate the new state
 * @return true if new state properly managed
 */
bool SarsaPolicyAlgo::DoAction(State& st, Action& act)
{
    State tmp;
    act.apply(st,tmp);

    switch(tmp.check())
    {
    case STATE_NORMAL:
        next = tmp.data;
        m_reset = false;
        nextState = tmp;
        return true;
    case STATE_INIT:
        next = tmp.data;
        m_reset = false;
        nextState = tmp;
        return true;
    case STATE_UNSAFE:
        m_reset = true;
        nextState = tmp;
        break;
    case STATE_TARGET:
        next = tmp.data;
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
void SarsaPolicyAlgo::run()
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
            ofstream rewFile;
            rewFile.open(rewardFileName.c_str(),ios_base::app);
            rewFile << m_episodes<<" "<< cumulativeReward/(Config::instance()->root["Experiment"]["AverageFor"].asInt()) << "\n";
            rewFile.close();
            cumulativeReward = 0;

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
void SarsaPolicyAlgo::threadRelease()
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




