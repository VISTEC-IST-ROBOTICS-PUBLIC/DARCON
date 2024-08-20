#ifndef MODULARNEURALCONTROL_H_
#define MODULARNEURALCONTROL_H_

#include "so2CPG.h"

#include <vector>
#include <cmath>
#include <cstdlib>


#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;


class modularController{


    private:
    
    std::vector<double> cpgVal{0.0,0.0}; 
    std::vector<double> cpgValOld;
    
    std::vector<double> cpgFlip{0.0,0.0};
    std::vector<double> cpgFlipOld{0.0,0.0};


    so2CPG *cpg; 

    double phi = 0.25; 
    double alpha = 1.5; 
    double reflex;

    double scale = 1.0; 

    double l1 = 1; 
    double l2 = 1; 

    // Mean filter 
    std::vector<double> meanList;
    double meanSize; 
    

    public:

    double meanFilter(double dataInput);

	modularController(double MI);
    modularController(double MI, double alpha);

	~modularController();
    void calculateForceFeedback(double x); 
    void calculateCPG(); 
    void stepDelayed(int motorname, double *y, std::vector<double> val); 
    void step(int motorname, double *y);

    // Control functions 
    void increaseReflex();
    void decreaseReflex();
    void setReflex(double MI); 
    void moveRight(); 
    void moveRightSoft(double speed); 
    void moveLeft(); 
    void moveLeftSoft(double speed);
    void moveForward(); 
    void moveBackward();
    void stopRobot();

    void enableForceFeedback();
    void disableForceFeedback();

    double getPhi();
    double getAlpha();
    double getReflex();
    vector<double> getCpgOutput();
    vector<double> getCpgPrevOutput();
    vector<double> getCpgActivation();

    std::vector<double> getNewAndOldCpgValue();

    void setScale(double val);

};

#endif 
