/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef StateAction__H__
#define StateAction__H__

#include <sstream>
#include <string>
#include <vector>

#include "Hierarchy.h"
#include "Discretizer.h"

using namespace std;

/**
 * Data structure used to represent a state. It can be either built through an id or using the actual state variables.
 * A map between the two representations is stored.
 */
class State {
public:
    vector<int> values;
    int state_id;
    static map<int,vector<int>> id2values;
    static map<vector<int>,int> values2id;
    static int counter;

    State(){
        state_id = -1;
    }

    /**
     * Create the state from the encoder values
     */
    State (vector<int> values) {
        this->values = values;
        state_id = counter++;
        if (State::values2id.find(values) == State::values2id.end())  {
            State::id2values[state_id] = this->values;
            State::values2id[this->values] = state_id;
        }
        else {
            state_id = State::values2id[this->values];
            counter--;
        }
    }

    /**
     * Create the state from the id
     */
    State (int id) {
        state_id = id;
        values = State::id2values[id];
    }

    /**
     * Utility used when saving the learning status.
     * The map between the state id and the encoder values
     * are dumped on a file.
     */
    static void printId2ValuesMap() {
        ofstream id2vFile("id2values.txt");
        std::map<int, vector<int>>::iterator iter;
        for(iter = State::id2values.begin(); iter != State::id2values.end(); iter++) {
            id2vFile << iter->first;
            for (int i = 0; i < iter->second.size();i++) {
                id2vFile << " " << iter->second[i];
            }
            id2vFile << endl;
        }
        id2vFile.close();
    }

    /**
     * Utility used when loading the learning status.
     * The map between the state id and the encoder values
     * is restored.
     */
    static void readId2ValuesMap() {
        std::ifstream id2vFile("id2values.txt");
        vector<int> val;
        int id;
        int v;
        while(id2vFile >> id) {
            val.clear();
            int state_dim =  Config::instance()->root["Domain"]["Target"].size();
            for (int i = 0; i < state_dim; i++) {
                id2vFile >> v;
                val.push_back(v);
            }
            State::id2values[id] = val;
            State::values2id[val] = id;
        }
        id2vFile.close();
    }

};

/**
 * Data structure used to represent a SARSA action
 */
class Action {
public:
    int action_id;
    int n_actions;
    MacroBehavior* macro;

    /**
     * If the action used locked joint, it cannot be used
     */
    static bool isLocked(int id, vector<int> locked) {
        Discretizer::actionMapping(id);
        int actIndex = Discretizer::getAction();
        string actionFilename = Config::instance()->root["Domain"]["Actions"][actIndex].asString();
        ifstream inFile(("../resources/data/macros/" + actionFilename).c_str());
        string behaviorName, part, joint;
        while (inFile >> behaviorName) {
            ifstream behaviorFile(("../resources/data/behaviors/" + behaviorName).c_str());
            while (behaviorFile >> part >> joint) {
                int j = atoi(joint.c_str()) + (*Info::starts)[part];
                for (int i = 0; i < locked.size(); i++) {
                    if (locked[i] == j) {
                        return true;
                    }
                }
            }
            behaviorFile.close();
        }
        inFile.close();
        return false;
    }

    Action() {
        action_id = -1;
        int params = Config::instance()->root["Domain"]["Discretization"]["0"].size();
        n_actions = Config::instance()->root["Domain"]["Actions"].size() * params;
    }

    /**
     * Creates the action and its parameter from the id provided
     */
    Action(int id) {
        action_id = id;
        int params = Config::instance()->root["Domain"]["Discretization"]["0"].size();
        n_actions = Config::instance()->root["Domain"]["Actions"].size() * params;
        Discretizer::actionMapping(id);
        int actIndex = Discretizer::getAction();
        string actionFilename = Config::instance()->root["Domain"]["Actions"][actIndex].asString();
        ifstream inFile(("../resources/data/macros/" + actionFilename).c_str());
        if (!inFile.is_open() || inFile.bad()) {
            cout << "Input file error" << endl;
        }
        string behaviorName;
        vector<Behavior*> vec;
        while (inFile >> behaviorName) {
            Behavior* b = new Behavior(behaviorName);
            b->setParam(Discretizer::getPos());
            vec.push_back(b);
        }
        inFile.close();
        macro = new MacroBehavior(vec);
    }

    /**
     * Apply the action, causing the robot motors to move accordingly
     */
    void apply() {
        macro->go();
    }
};

#endif
