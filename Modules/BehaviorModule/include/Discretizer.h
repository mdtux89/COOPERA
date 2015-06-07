/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef DISCRETIZER_H
#define DISCRETIZER_H
#include "Configuration.h"

/**
 * Utilities for dealing with discretization of states and actions
 */
class Discretizer {
private:
    static int act, pos;
    static vector<bool> locked;

public:

    /**
     * See wich joint are locked in order to ignore them when comparing with the target state
     * @param lockedVec the vector of the locked joints
     */
    static void setLocked(vector<int> lockedVec) {
        locked.clear();
        int state_dim =  Config::instance()->root["Domain"]["Target"].size();
        for (int i = 0; i < state_dim; i++) {
            locked.push_back(false);
        }
        for (int i = 0; i < lockedVec.size(); i++) {
            locked[lockedVec[i]] = true;
        }
    }

    /**
     * Returns the several values that the joint (identified by index) can take
     * @param index the joint index
     * @return the allowed values for the joint
     */
    static vector<int> getDiscretizations(int index) {
        vector<int> disc;
        for (int i = 0; i < Config::instance()->root["Domain"]["Discretization"][boost::to_string(index)].size(); i++) {
            disc.push_back(Config::instance()->root["Domain"]["Discretization"][boost::to_string(index)][i].asInt());
        }
        return disc;
    }

    /**
     * Discretize a continuous value for the given joint
     * @param val the continuous value
     * @param index the joint index
     * @return the discretized value
     */
    static int discretize(double val, int index) {
        if (locked.size() <= index) {
            for (int i = 0; i <= index; i++) {
                locked.push_back(false);
            }
        }
        if (Config::instance()->root["Domain"]["Discretization"][boost::to_string(index)].size() == 0 || locked[index]) {
            return -1;
        }

        double min = 300;
        double d = 0;
        for (int i = 0; i < Config::instance()->root["Domain"]["Discretization"][boost::to_string(index)].size(); i++) {
            double diff = abs(val - Config::instance()->root["Domain"]["Discretization"][boost::to_string(index)][i].asInt());
            if (diff < min) {
                min = diff;
                d = Config::instance()->root["Domain"]["Discretization"][boost::to_string(index)][i].asInt();
            }
        }
        return d;
    }

    /**
     * Given the action id, compute the actual macro and the parameter value
     * @param id the action id
     */
    static void actionMapping(int id) {
        int nparam = Config::instance()->root["Domain"]["Discretization"]["0"].size();

        //Identify action
        act = (id / nparam);
        id -= (act * nparam);

        //Identify position parameter
        pos = id;
    }

    /**
     * Returns the action (actually the macro)
     * @return the action previously computed
     */
    static int getAction() {
        return act;
    }

    /**
     * Returns the parameter value
     * @return  the parameter value previously computed
     **/
    static int getPos() {
        return pos;
    }

    /**
     * Given the parameter and the joint concerned, computes the position value associated with that
     * parameter for that joint
     * @param param the parameter
     * @param joint the joint
     **/
    static int paramToPos(int param, int joint) {
        return Config::instance()->root["Domain"]["Discretization"][boost::to_string(joint)][param].asInt();
    }

};

#endif
