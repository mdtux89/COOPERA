/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Hierarchy__H__
#define Hierarchy__H__

#include <sstream>
#include <string>
#include <vector>
#include "Controller.h"
#include <map>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include "Configuration.h"
#include "Info.h"
#include "Discretizer.h"

#define TIMEOUT_ENCODERS 1
#define ENCODERMIN 0.001

#define MAXARM 210
#define MINARM 90
#define MAXSHOULDER 210
#define MINSHOULDER 90

using namespace std;
using namespace yarp::dev;

/**
 * Yarp thread used to define a single behavior.
 * It is the basic action unit used in the SARSA implementation.
 */
class Behavior : public Thread {
public:
    string name;
private:
    bool fail;
    int param;
    vector<int> startState;

    static void getEnc(IEncoders *enc, int j, double *val) {
        double v;
        enc->getEncoder(j,&v);
        *val = v;
    }

public:

    Behavior(string name) {
		this->name = name;
	}

    bool isFailed() {
        return fail;
    }

    void setParam(int param) {
        this->param = param;
	}

    /**
     * Run the behavior motion
     */
    void run() {
        //Read all the encoders (the motors that are not move will use these values as the new positions)
        map<int,vector<double>> encoders;
        vector<double> current;
        for(int i=0;i<Info::nParts;i++){
            IEncoders *enc;
            if (!Info::drivers[i].view(enc)) {
                cout << "Behavior cannot be performed" << endl;
                return;
            }
            for(int j=0;j<(*Info::nJoints)[i];j++){
                double v = 0;
                int k=0;
                for (; k < 100; k++) {
                     boost::thread api_caller(Behavior::getEnc, enc, j, &v);
                     api_caller.timed_join(boost::posix_time::seconds(TIMEOUT_ENCODERS));
                     if ( v > ENCODERMIN) {
                         current.push_back(v);
                         break;
                     }
                }
                if(k==100) {
                    cout << "Behavior cannot be performed" << endl;
                    return;
                }
            }
        }
        int c = 0;
        for (int k = 0; k < Info::nParts; k++) {
            for (int i = 0; i < (*Info::nJoints)[k]; i++) {
                encoders[k].push_back(current[c]);
                c++;
            }
        }

		//process the input file (specified by name) and use properly
        //the parameters provided to run the behavior
        ifstream inFile;
        inFile.open(("../resources/data/behaviors/" + name).c_str());
        if (!inFile.is_open() || inFile.bad()) {
            cout << "Input file error" << endl;
        }
        string part,joint,sign;
        vector<string> parts;

        //read the behavior details
        while(inFile >> part >> joint >> sign){
            parts.push_back(part);
            int jointIndex = atoi(joint.c_str());
            int j = jointIndex + (*Info::starts)[part];
            int partIndex = (*Info::parts)[part];
            encoders[partIndex][jointIndex] = Discretizer::paramToPos(param,j);

            //very ugly check to avoid clashing the head..!!
            if (encoders[0][0] >= MAXARM && encoders[0][1] <= MINSHOULDER) {
                return;
            }
            if (encoders[1][0] <= MINARM && encoders[1][1] >= MAXSHOULDER) {
                return;
            }
            if (encoders[0][0] <= MINARM && encoders[0][1] >= MAXSHOULDER) {
                return;
            }
            if (encoders[1][0] >= MAXARM && encoders[1][1] <= MINSHOULDER) {
                return;
            }
        }
        inFile.close();

        //run the movements required
        vector<MotorsThread> vt;
        for (int i = 0; i < parts.size(); i++) {
            int partIndex = (*Info::parts)[parts[i]];
            MotorsThread t(&Info::drivers[partIndex],encoders[partIndex]);
            vt.push_back(t);

        }
        for (int i = 0; i < parts.size(); i++) {
            vt[i].motion();
        }
        for (int i = 0; i < parts.size(); i++) {
            if(!vt[i].join()) {
                cout << "Behavior timeout" << endl;
            }
        }
        return;
	}
};

/**
 * A combination of basic Behavior.
 * It is used as an action in the learning used for standing up.
 */
class MacroBehavior {
private:
    vector<Behavior*> behaviors;
public:
    MacroBehavior(vector<Behavior*> behaviors){
        this->behaviors = behaviors;
	}

    /**
     * Run the macro-behavior
     */
    void go() {
		for (int i = 0; i < behaviors.size(); i++) {
            behaviors[i]->start();
		}
		for (int i = 0; i < behaviors.size(); i++) {
            behaviors[i]->join();
		}
        Time::delay(Info::macroWait);
	}
};

/**
 * Enable the robot to safely descent to a reset state after having reached the target state
 */
class Descent {
public:
    /**
     * Enable the robot to safely descent to a reset state after having reached the target state.
     * If the starting point is the target state I need to choose a proper descent strategy, otherwise I can use
     * a simple reset script.
     * @param pose the target pose
     * @param target whether the starting point is the target state
     */
    static void descent(string pose, bool target) {
        ifstream ifile;

        if (pose == "UNKNOWN") {
            return;
        }
        if (target) {
            ifile.open(("../resources/data/descents/" + pose + ".txt").c_str());
        }
        else {
            ifile.open(("../resources/data/resets/" + pose + ".txt").c_str());
        }
        while(true){ //until file ends

            string read;
            string tok;
            if(!getline(ifile,read)){ //read one line
                return;
            }
            stringstream ss(read);

            vector<MotorsThread> vt;
            for(int i=0;i<Info::nParts;i++){ //for each robot parts
                vector<double> encoders;
                //split the line and push the values in a vector
                for(int j=0; j<(*Info::nJoints)[i]; j++){
                    getline(ss,tok,',');
                    encoders.push_back(atof(tok.c_str()));
                }

                //pass everything to the threads
                MotorsThread t(&Info::drivers[i],encoders);
                vt.push_back(t);
           }

           for (int i = 0; i < vt.size(); i++) {
                vt[i].motion();
           }
           for (int i = 0; i < vt.size(); i++) {
                vt[i].join();
           }

           // wait for each motion to terminate
           Time::delay(Info::motionWait);
        }
    }
};
#endif
