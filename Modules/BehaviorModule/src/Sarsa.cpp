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
    if(!load) {
        map<int,double*>::iterator iter;
        for(iter = m_Q->begin(); iter!=m_Q->end(); ++iter)
        {
            delete [] iter->second;
        }
        m_Q->erase(m_Q->begin(),m_Q->end());
    }

    originalTime = 0;
    m_isRecordedState = false;

    if(m_isRecordedState)
        timeSinceRestart = yarp::os::Time::now();

    explorer->initPolicy(m_Q);

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

    explorer = new Policy(&uniformSampler);
    explorer->initPolicy(m_Q);

    m_length = 0;
    m_isFirst = true;
    totalTimeTaken = 0;

    //Set the initial state (via pose estimation)
    vector<int> val;
    evolution->poseEstimation(pose);

    loadLearning();

    for (int i = 0; i < Config::instance()->root["Domain"]["Init"][pose->pose].size(); i++) {
        int v = Discretizer::discretize(Config::instance()->root["Domain"]["Init"][pose->pose][i].asInt(),i);
        val.push_back(v);
    }
    cout << endl;
    thisState = State(val);
    oldInit = thisState.state_id;
    lastRealState = thisState;

    if(m_episodes == 0 ) {
        graph.InsertNode(thisState.state_id);
        graph.InsertEdgeFromInit(thisState.state_id,-2);
    }

    //Set target state
    val.clear();
    for (int i = 0; i < Config::instance()->root["Domain"]["Target"].size(); i++) {
        int v = Discretizer::discretize(Config::instance()->root["Domain"]["Target"][i].asInt(),i);
        val.push_back(v);
    }
    targetState = State(val);

    cout << "Init and target done" << endl;

    //Choose a random action
    int id = 0;
    do {
        id = int(uniformSampler.sample() * (thisAction.n_actions));
    } while(Action::isLocked(id,locked));
    thisAction = Action(id);

    std::cout<<"Learner constructed for first time!\n";
    return true;

}

/**
 * Given the current state infers the action to choose using the learned Q-values
 * @param s the current state
 * @param a the chosen action
 */
void LearnerThread::GetAction(State& s, Action& a) {
    explorer->explore(s,a,locked);
}

/**
 * Computes the reward for the given state
 * @param st the given state
 */
void LearnerThread::GetReward(int stStatus)
{
    Json::Value rwdConf = Config::instance()->root["Reward"];
    thisReward = rwdConf["Action"].asDouble();

    if( stStatus == FALL_RESET )
    {
            thisReward += rwdConf["Fall"].asDouble();
            m_reset = true;
            m_target = false;
    }
    else if( /*stStatus == OVERHEATING_RESET ||*/ stStatus == FEEDBACK_RESET || stStatus == OVERLOAD_RESET)
    {
            thisReward += rwdConf["HwReset"].asDouble();
            m_reset = true;
            m_target = false;
    }
    else if(stStatus == TARGET_STATE)
    {
        thisReward = rwdConf["Goal"].asDouble();
        m_reset = true;
        m_target = true;
    }
    else {
        m_reset = false;
        m_target = false;
    }

    // Adjust by factor
    thisReward *= rwdConf["Factor"].asDouble();

    cumulativeReward += thisReward;
}

/**
 * Update the Q-values using the Sarsa update rule
 */
void LearnerThread::UpdateQ()
{
    if((*m_Q)[thisState.state_id]==NULL){
        (*m_Q)[thisState.state_id] = new double[thisAction.n_actions];
        for(int i=0 ; i<thisAction.n_actions; i++){
            (*m_Q)[thisState.state_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
        }
    }
    if((*m_Q)[nextState.state_id]==NULL){
        (*m_Q)[nextState.state_id] = new double[nextAction.n_actions];
        for(int i=0 ; i<nextAction.n_actions; i++){
            (*m_Q)[nextState.state_id][i]=Config::instance()->root["Experiment"]["InitialQValue"].asDouble();
        }
    }

    if(m_reset)
    {
        (*m_Q)[thisState.state_id][thisAction.action_id] +=
                m_alpha*( thisReward - (*m_Q)[thisState.state_id][thisAction.action_id]);

    }
    else
    {
        (*m_Q)[thisState.state_id][thisAction.action_id] +=
                m_alpha*(thisReward + m_gamma*
                         ((*m_Q)[nextState.state_id][nextAction.action_id] - (*m_Q)[thisState.state_id][thisAction.action_id]));
    }

    /*cout<<"Q-value at s="<<thisState.state_id<<
               " a="<<thisAction.action_id<<" updated to " <<
          (*m_Q)[thisState.state_id][thisAction.action_id] << ".\n";*/
}

/**
 * Run one iteration of Sarsa
 * @param learn whether this is a learning run or not
 */
void LearnerThread::run(bool learn)
{
    int stStatus;
    if(m_isFirst)
    {
        m_isFirst = false;
        GetAction(thisState,thisAction);
    }

    // Apply (or simulate) the action, check and record everything on the graph
    // If the state is not explored (or we are not learning anymore) perform the action
    if (!learn || !graph.isExploredStateAction(thisState.state_id,thisAction.action_id)) {
        // If the previous action was simulated, then I need to take the robot in the right state before being
        // able to apply the current action
        bool recoveringFault;
        if (lastRealState.state_id != thisState.state_id) {
            list<int> path = graph.shortestPath(lastRealState.state_id, thisState.state_id);
            for (list<int>::iterator it = path.begin(); it != path.end(); ++it) {
                Action a = Action(*it);
                a.apply();
            }
            recoveringFault = evolution->stateEstimation();
            thisState = evolution->getState();
        }
        thisAction.apply();
        stStatus = evolution->check(targetState);
        if (recoveringFault == OVERHEATING_RESET || recoveringFault == FEEDBACK_RESET || recoveringFault == OVERLOAD_RESET)
            stStatus = recoveringFault;
        nextState = evolution->getState();
        lastRealState = nextState;
        graph.InsertNode(nextState.state_id);
        graph.InsertEdge(thisState.state_id,nextState.state_id,thisAction.action_id);
        if (stStatus == FALL_RESET) {
            graph.InsertEdgeToFallSink(nextState.state_id,-2);
        }
        else if (stStatus == OVERHEATING_RESET) {
            //graph.InsertEdgeToOverHeatingSink(nextState.state_id,-2);
            overheating.push_back(m_episodes);
        }
        else if (stStatus == FEEDBACK_RESET) {
            graph.InsertEdgeToFeedbackErrorSink(nextState.state_id,-2);
            feedbackerror.push_back(m_episodes);
        }
        else if (stStatus == OVERLOAD_RESET) {
            graph.InsertEdgeToOverLoadSink(nextState.state_id,-2);
            overload.push_back(m_episodes);
        }
        else {
            graph.InsertEdgeToSafeSink(nextState.state_id,-2);
        }
    }
    else {
       //simulate the action: get the most likely next state and update the graph
       nextState = State(graph.getNextProbState(thisState.state_id,thisAction.action_id));
       simulatedActions++;
       graph.InsertNode(nextState.state_id);
       graph.InsertEdge(thisState.state_id,nextState.state_id,thisAction.action_id);
       stStatus = graph.check(nextState.state_id, targetState.state_id);
       if (stStatus == OVERHEATING_RESET) {
           overheating.push_back(m_episodes);
       }
       else if (stStatus == FEEDBACK_RESET) {
           feedbackerror.push_back(m_episodes);
       }
       else if (stStatus == OVERLOAD_RESET) {
           overload.push_back(m_episodes);
       }
    }

    // Obtain action for this state
    GetAction(nextState,nextAction);

    // Update time-taken
    totalTimeTaken += 1;

    GetReward(stStatus);
    UpdateQ();

    if(m_length > Config::instance()->root["State"]["MaxLengthFactor"].asDouble() *
            Config::instance()->root["Domain"]["Actions"].size()) {
        m_reset = true;
        m_target = false;
    }

    if(m_reset) {
        cout << "*** world reset *** \n Episode#: " <<m_episodes<<" \n";

        if(m_episodes % (Config::instance()->root["Experiment"]["SaveAfter"].asInt()) == 0) {
            saveLearning();
        }

        vector<int> val;
        if (Info::poseEstimation) {
            Descent::descent(pose->indexToPose(int(uniformSampler.sample() * pose->nPoses)),m_target);
        }
        else {
            Descent::descent(Info::pose,m_target);
        }
        evolution->stateEstimation();
        evolution->poseEstimation(pose);

        for (int i = 0; i < Config::instance()->root["Domain"]["Init"][pose->pose].size(); i++) {
            int v = Discretizer::discretize(Config::instance()->root["Domain"]["Init"][pose->pose][i].asInt(),i);
            val.push_back(v);
        }
        thisState = State(val);
        lastRealState = thisState;
        graph.InsertNode(thisState.state_id);
        graph.InsertEdgeFromInit(thisState.state_id,-2);

        m_isFirst = true;
        totalTimeTaken = 0;
        m_length = 0;
        ++m_episodes;
        printQtable();

        if(m_episodes % (Config::instance()->root["Experiment"]["AverageFor"].asInt()) == 0) {
            ofstream rewFile;
            rewFile.open(rewardFileName.c_str(),ios_base::app);
            rewFile << m_episodes<<","<< cumulativeReward/(Config::instance()->root["Experiment"]["AverageFor"].asInt()) << "\n";
            rewFile.close();
            cumulativeReward = 0;
        }

        if(learn) {

            // Each releaseJointAfter episodes are completed, release a locked joint to refine the policy learned.
            // Redefine accordingly the initial state and the target state, reconstruct the graph and the Q-table
            if(m_episodes % Config::instance()->root["Learn"]["releaseJointAfter"].asInt() == 0) {
                if(locked.size() != 0) {
                    int index = int(uniformSampler.sample() * locked.size());
                    int idToRelease = locked[index];
                    locked.erase(locked.begin() + index);

                    cout << "Release joint " << idToRelease << endl;

                    Discretizer::setLocked(locked);
                    vector<int> val;
                    for (int i = 0; i < Config::instance()->root["Domain"]["Target"].size(); i++) {
                        int v = Discretizer::discretize(Config::instance()->root["Domain"]["Target"][i].asInt(),i);
                        val.push_back(v);
                    }
                    targetState = State(val);
                    val.clear();
                    for (int i = 0; i < Config::instance()->root["Domain"]["Init"][pose->pose].size(); i++) {
                        int v = Discretizer::discretize(Config::instance()->root["Domain"]["Init"][pose->pose][i].asInt(),i);
                        val.push_back(v);
                    }
                    thisState = State(val);
                    lastRealState = thisState;

                    // After releasing a joint, the graph as well as the Q-table are obsolete. They refer to states that are
                    // not part of the learning anymore. Therefore we need to delete all the old states and replacing them
                    // with the new states
                    int oldId;
                    vector<int> newIds;
                    vector<int> disc = Discretizer::getDiscretizations(idToRelease);
                    map<int,double*>::iterator iter;
                    map<int,vector<int>> idmap; // this is the structure that I will use to map the old state id with the
                                                // list of the ids of the new states
                    map<int,vector<double>> tmpMap;
                    Recorder r(Config::instance()->root["Experiment"]["NRealSamples"].asInt());

                    //fake mapping for init and sink states
                    vector<int> fake;
                    for (int i = 0; i < 6; i++) {
                        fake.clear();
                        fake.push_back(i);
                        idmap[i] = fake;
                    }

                    // Insert all the desired node: for each of the old nodes I will have N nodes, where N is the
                    // number of possible values that the new released joint can have
                    r.Initialize();
                    r.InsertNode(thisState.state_id);
                    fake.clear();
                    fake.push_back(thisState.state_id);
                    idmap[oldInit] = fake;
                    oldInit = thisState.state_id;
                    r.InsertEdgeFromInit(thisState.state_id,-2);
                    // Iterating through the Q-table I have the list of all states (and therefore all nodes in the graph)
                    for(iter = m_Q->begin(); iter!=m_Q->end(); ++iter) {
                        oldId = iter->first;
                        if (oldId == 6) continue;
                        newIds.clear();
                        vector<int> vals = State(oldId).values;
                        for(int i = 0; i < disc.size(); i++) {
                            vals[idToRelease] = disc[i];
                            newIds.push_back(State(vals).state_id);
                            r.InsertNode(newIds[i]);
                        }
                        idmap[oldId] = newIds;
                        // Since I am iterating through the Q-table I save the Q-values that I will need later
                        for (int i = 0; i < newIds.size(); i++) {
                            vector<double> tmp;
                            for (int j = 0; j < thisAction.n_actions; j++) {
                                tmp.push_back(iter->second[j]);
                            }
                            tmpMap[newIds[i]] = tmp;
                        }
                    }

                    // Now insert all the edges
                    vector<int> fromVec;
                    vector<int> toVec;
                    vector<tuple<int,int,int>> edges = graph.getEdges();
                    for (int i = 0; i < edges.size(); i++) {
                        int from = std::get<0>(edges[i]);
                        if (from >= 0 && from <= 3) {
                            continue;
                        }
                        fromVec = idmap[from];
                        int to = std::get<1>(edges[i]);
                        toVec = idmap[to];
                        int aId = std::get<2>(edges[i]);
                        for (int i = 0; i < fromVec.size(); i++) {
                            for (int j = 0; j < toVec.size(); j++) {
                                r.InsertEdge(fromVec[i],toVec[j],aId);
                            }
                        }
                    }

                    // Replace the graph
                    graph.SaveTheGraph("graphbefore");
                    graph = r;
                    graph.SaveTheGraph("graphafter");
                    int k = 0;
                    cin >> k;


                    // Reconstruct the Q-table deleting the old state and using the new states
                    for(iter = m_Q->begin(); iter!=m_Q->end(); ++iter) {
                        delete [] iter->second;
                    }
                    m_Q->erase(m_Q->begin(),m_Q->end());
                    map<int,vector<double>>::iterator iterTmp;
                    for(iterTmp = tmpMap.begin(); iterTmp!=tmpMap.end(); ++iterTmp) {
                        (*m_Q)[iterTmp->first] = new double[thisAction.n_actions];
                        for (int i = 0; i < thisAction.n_actions; i++) {
                            (*m_Q)[iterTmp->first][i] = iterTmp->second[i];
                        }
                    }
                }
            }

            //Update the sevearal parameters accordingly with the decay strategies
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
