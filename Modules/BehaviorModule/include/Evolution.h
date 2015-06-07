/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef Evolution__H__
#define Evolution__H__

#define NORMAL_STATE 0
#define TARGET_STATE 1
#define FALL_RESET 2
#define FEEDBACK_RESET 3
#define OVERHEATING_RESET 4
#define OVERLOAD_RESET 5

#define TIMEOUT_ENCODERS 1
#define TIMEOUT_RECOVERY 20
#define ENCODERMIN 0.001

#include "StateAction.h"
#include "Controller.h"
#include "Pose.h"

using namespace std;

/**
 * Used by Evolution, it runs the estimate of the robot current state
 */
class Estimate : public Thread {
private:
        vector<int> state;

        /**
         * Read the tilts on the two axis using the Phidget sensor
         * @return the two values of tilt
         */
        vector<int> getTilts() {
            vector<int> ret;
            Bottle* b;
            string tok;
            int k = 0;
            while (Info::sensorPort->getPendingReads() == 0 && k < 10) {
                Time::delay(0.1);
                k++;
            }
            if(k == 10) {
                return ret;
            }
            b = Info::sensorPort->read(false);
            stringstream ss(b->toString());
            for (int j = 0; j <= Info::tiltIndex; j++) {
                getline(ss,tok,' ');
            }
            int iTilt = Config::instance()->root["Domain"]["TiltIndex"].asInt();
            ret.push_back(Discretizer::discretize(atof(tok.c_str()),iTilt));
            getline(ss,tok,' ');
            ret.push_back(Discretizer::discretize(atof(tok.c_str()),iTilt + 1));
            return ret;
        }

        /**
         * Read the encoder from the Dynamixel specified
         * @param enc the encoder Yarp interface
         * @param j the joint
         * @param the output encoder
         */
        static void getEnc(IEncoders *enc, int j, double *val) {
            double v;
            enc->getEncoder(j,&v);
            *val = v;
        }

        /**
         * Read the encoders from all the Dynamixels
         * @return the encoders from all the servomotors
         */
        vector<int> getPositions() {
           vector<int> ret;
           vector<double> val;
           for(int i=0;i<Info::nParts;i++){ //for each robot parts
               IEncoders *encoders;
               if (!Info::drivers[i].view(encoders)) {
                 return ret;
               }
               for(int j=0;j<(*Info::nJoints)[i];j++){
                   double v = 0;
                   int k=0;
                   for (k = 0; k < 100; k++) {
                        boost::thread api_caller(Estimate::getEnc, encoders, j, &v);
                        api_caller.timed_join(boost::posix_time::seconds(TIMEOUT_ENCODERS));
                        if (v > ENCODERMIN) {
                            val.push_back(v);
                            break;
                        }
                   }
                   if (k == 100) {
                       return ret;
                   }
               }
           }

           for (int i = 0 ; i < val.size(); i++) {
               int v = Discretizer::discretize(val[i],i);
               ret.push_back(v);
           }
           return ret;
        }
public:
        /**
         * Run the estimate
         */
        void run() {
            state.clear();
            vector<int> pos = getPositions();
            for (int i = 0; i < pos.size(); i++) {
                state.push_back(pos[i]);
            }
            pos = getTilts();
            for (int i = 0; i < pos.size(); i++) {
                state.push_back(pos[i]);
            }
        }

        /**
         * Return the estimated state
         * @return the state
         */
        vector<int> getEstimate() {
            return state;
        }
};

/**
 * Class used to manage the evolution of the estimated state in our learning problem
 */
class
        Evolution {
private:
        vector<int> state;
        bool reset_flag;
        Guardian *thread;
        Estimate *est;
        vector<int> motorJoint;

public:
        Evolution(Guardian* thread) {
            this->thread = thread;
            est = new Estimate();
            est->start();
            est->join();
            state = est->getEstimate();
            int state_dim =  Config::instance()->root["Domain"]["Target"].size();
            while (state.size() != state_dim) {
                system("speaker-test -t sine -f 300 -p 2000 -l 1 2>&1");
                char c;
                cout << "CANNOT START! HELP ME PLEASE!" << endl;
                cout << "Type any key to continue" << endl;
                cin >> c;
                est->start();
                est->join();
                state = est->getEstimate();
            }
            for (int i = 0; i < Config::instance()->root["Domain"]["Discretization"].size(); i++) {
                if (Config::instance()->root["Domain"]["Discretization"][boost::to_string(i)].size() != 0) {
                    motorJoint.push_back(i);
                }
            }
            reset_flag = false;
        }

        /**
         * Estimate the robot state
         * @return the fault code or -1 if none occurred
         */
        int stateEstimation() {
            est->start();
            est->join();
            state = est->getEstimate();
            int fault = -1;
            int state_dim =  Config::instance()->root["Domain"]["Target"].size();

            // If the first estimate went awry, I wait for a while and try again.
            // If still something is wrong I reckon there is a fault and ask for assistance
            if (state.size() != state_dim) {
                Time::delay(TIMEOUT_RECOVERY);
                est->start();
                est->join();
                state = est->getEstimate();
            }
            if (state.size() != state_dim) {
                fault = Guardian::getLastError();
            }
            else {
                fault = -1;
            }
            while (state.size() != state_dim) {
                // If I enter here, it means this a real fault, so I send out the alarms,
                // ask for help and repeat until I get the assistance
                system("speaker-test -t sine -f 300 -p 2000 -l 1 2>&1");
                char c;
                cout << "HELP ME PLEASE!" << endl;
                cout << "Type any key to continue" << endl;
                cin >> c;
                est->start();
                est->join();
                state = est->getEstimate();
            }
            return fault;
        }

        /**
         * Estimate the robot pose (facedown/faceup/sideright/sideleft)
         * @param pose the Pose object to be used for running the estimation
         */
        void poseEstimation(Pose* pose) {
            if (Info::poseEstimation) {
                pose->classify();
                pose->test();
            }
            else {
                pose->pose = Info::pose;
            }
            while (pose->pose == "UNKNOWN") {
                char c;
                cout << "POSE UNKNOWN! HELP ME PLEASE!" << endl;
                cout << "Type any key to continue" << endl;
                cin >> c;
                pose->classify();
                pose->test();
            }
        }

        /**
         * Return the estimated robot state
         * @return the state
         */
        State getState() {
            return State(state);
        }

        /**
         * Check the robot state. Used by SARSA to decide the reward to be given.
         * @param target the target state
         */
        int check(State target) {
            int st = stateEstimation();
            if (st != -1) {
                // there is a fault, return the error code
                return st;
            }
            if (thread->fall) {
                // the robot fell
                cout << "FALL DETECTED" << endl;
                thread->fall = false;
                return FALL_RESET;
            }
            State thisS(state);
            if (thisS.state_id == target.state_id) {
                // the target state is reached
                cout << "TARGET DETECTED" << endl;
                return TARGET_STATE;
            }
            else {
                // nothing worth noting happened
                return NORMAL_STATE;
            }
        }
};
#endif
