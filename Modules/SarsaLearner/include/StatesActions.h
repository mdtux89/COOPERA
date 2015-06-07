/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef STATEACTION_H
#define STATEACTION_H

#include <Configuration.h>
#include <vector>
#include <limits>
#include <string>
#include <list>
#include <cmath>
#include <iostream>
using namespace std;

#define STATE_UNSAFE 0
#define STATE_NORMAL 1
#define STATE_TARGET 2
#define STATE_INIT 3
#define STATE_OUT_OF_BOUND 4

static int relevantIndex;

/**
 * Define one action step; each action is composed of a number of steps.
 * One step is identified by the motion part involved (left arm, right arm, etc..), the joint id, and the movement
 * extent
 */
class Step{
public:
    string part;
    int joint;
    int range;

    Step(string part, int joint, int range){
        this->part = part;
        this->joint = joint;
        this->range = range;
    }
};

/**
 * Describe the state datatype for Sarsa RL experiments.
 * Each state remembers the state that generated it, the actions used for reach it and the set of unsafe actions
 * that cannot be used. The description of each state is a vector of integer representing the encoders value.
 */
class State
{
public:
    int size;
    long int m_id, m_states;
    bool unsafe, outbound;
    vector<int> data;
    list<int> unsafeActions;
    list<int> previousActions;
    State* previousState;

    State()
    {
        previousState = NULL;
        size = Config::instance()->root["Domain"]["Initial"][to_string(relevantIndex)].size();
        m_states = 1;
        for(unsigned int i = 1; i <= (Config::instance()->root["Domain"]["Actions"].size()+1); ++i){ // compute factorial of actions
            m_states *= i;
        }
        m_id = 0;
        unsafe = outbound = false;
    }

    State(vector<int> data)
    {
        string str="";
        size =  Config::instance()->root["Domain"]["Initial"][to_string(relevantIndex)].size();
        for(int i=0;i<size;i++){
            string d = to_string(data[i]);
            str = str + d + " ";
        }
        unsafeActions.clear();
        for(int i=0; i<Config::instance()->root["Domain"]["Unsafe"][str].size();i++){
            unsafeActions.push_back(Config::instance()->root["Domain"]["Unsafe"][str][i].asInt());
        }
        m_id = getId(data);
        previousState = NULL;
        m_states = 1;
        for(unsigned int i = 1; i <= (Config::instance()->root["Domain"]["Actions"].size()+1); ++i){ // compute factorial of actions
            m_states *= i;
        }
        unsafe = outbound = false;
    }

    /**
     * Update the State data
     * @param data the State data
     * @param previousActions the list of previous action occured
     * @param action the new action
     */
    void update(vector<int> data, list<int> previousActions, int action){
        string str="";
        for(int i=0;i<data.size();i++){
            string d = to_string(data[i]);
            str = str + d + " ";
        }

        previousActions.push_back(action);
        this->previousActions = previousActions;
        unsafeActions.clear();
        for(int i=0; i<Config::instance()->root["Domain"]["Unsafe"][str].size();i++){
            unsafeActions.push_back(Config::instance()->root["Domain"]["Unsafe"][str][i].asInt());
        }
        m_id = getId(data);
        unsafe = outbound = false;
    }

    /**
     * Computes the distance between this State and another
     * @param s the other state
     * @return the distance computed
     */
    double distance(State s){
            int d=0;
            int tolerance = Config::instance()->root["Domain"]["Tolerance"].asInt();
            if(data.size()!=s.data.size()){
                return numeric_limits<int>::max();
            }
            for(int i=0;i<data.size();i++){
                if((data[i] - s.data[i])*(data[i] - s.data[i])<=(tolerance*tolerance)){
                    d+= 0;
                }
                else d += (data[i] - s.data[i])*(data[i] - s.data[i]);
            }
            return sqrt(d);
    }

    /**
     * Get the state id
     * @param data the State data
     * @return the State id
     */
    int getId(vector<int> data)
    {
        this->data = data;
        size_t seed = 0;
        for(int i=0; i< data.size();i++){
            boost::hash_combine( seed, data[i] );
        }
        return seed;
    }

    /**
     * Check the State status
     * @return the status
     */
    int check()
    {
        if(unsafe){
            return STATE_UNSAFE;
        }

        bool ret;
        if(data.size()==0) return -1;
        for(int i=0; i<Config::instance()->root["Domain"]["Initial"].size();i++){
            ret = true;
            for(int j=0; j<size; j++){
                ret &= (data[i] == Config::instance()->root["Domain"]["Initial"][to_string(i)][j].asInt());
            }
            if(ret){
                return STATE_INIT;
            }
        }

        ret = true;
        vector<int> target;
        for(int i=0;i<size;i++){
            target.push_back(Config::instance()->root["Domain"]["Target"][i].asInt());
        }
        ret &= (this->distance(State(target))==0);

        if(ret){
            return STATE_TARGET;
        }

        if(outbound){
            return STATE_OUT_OF_BOUND;
        }

        return STATE_NORMAL;

    }

};

/**
 * Describe the action datatype for Sarsa RL experiments.
 * Not every action can be used, only the relevant actions can be used
 */
struct Action
{
    int m_actions; /* Number of relevant actions available */
    int m_all_actions; /* Number of actions available */
    int m_id; /* Action id */

    Action()
    {
        m_all_actions = Config::instance()->root["Domain"]["Actions"].size();
        m_actions = Config::instance()->root["Domain"]["RelevanceSets"][to_string(relevantIndex)].size();
        if(Config::instance()->root["Domain"]["ReverseAction"].asBool()==true){
            m_all_actions++;
            m_actions++;
        }
    }
    Action(int id)
    {
        m_id = id;
        m_actions = Config::instance()->root["Domain"]["RelevanceSets"][to_string(relevantIndex)].size();
        m_all_actions = Config::instance()->root["Domain"]["Actions"].size();
        if(Config::instance()->root["Domain"]["ReverseAction"].asBool()==true){
            m_all_actions++;
            m_actions++;
        }
    }

    /**
     * Get the Action id
     * @return the id
     */
    int id()
    {
        return m_id;
    }

    /**
     * Check whether the action is safe using the unsafeActions data structure
     * @param s the state in which the action occurs
     * @return whether the action is safe or not
     */
    bool isSafe(State &s){
        list<int>::iterator it = find(s.unsafeActions.begin(),s.unsafeActions.end(),this->id());
        return (it==s.unsafeActions.end());
    }

    /**
     * Apply this action to a state
     * @param stBefore the input state
     * @param stAfter the output resulting state
     * @return the outcome of the new state check
     */
    bool apply(State &stBefore,State &stAfter){

      stAfter.previousState = new State(stBefore);

      vector<int> v = stBefore.data;
      v.resize(stBefore.size);

      int reverse = Config::instance()->root["Domain"]["ReverseId"].asInt();
      if(m_id!=reverse){
          for(int i=0;i<stBefore.size;i++){
              v[i] += Config::instance()->root["Domain"]["Actions"][to_string(m_id)][i].asInt();
          }
          stAfter.update(v,stBefore.previousActions,m_id);
      }
      else if(stBefore.previousState==NULL){
          stAfter = stBefore;
          stAfter.outbound = true;
          return false;
      }
      else{
          stAfter = *stBefore.previousState;
          if(stBefore.previousState->previousState==NULL){
              stAfter.previousState=NULL;
          }
      }

      if(!this->isSafe(stBefore)){
          stAfter.unsafe = true;
      }
      return ((stAfter.check() == STATE_NORMAL) || (stAfter.check() == STATE_INIT)) ;
   }
};

#endif
