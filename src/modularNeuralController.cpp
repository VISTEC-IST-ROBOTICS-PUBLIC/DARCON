



#include "modularNeuralController.h"

modularController::modularController(double MI){
    reflex = MI; 
    cpg = new so2CPG(MI);
}

modularController::modularController(double MI, double alpha){
    reflex = MI; 
    cpg = new so2CPG(MI, alpha);

    // Setting up mean filter and size
    meanSize = 3; 
    meanList = std::vector<double>(meanSize);
}



void modularController::decreaseReflex(){
    reflex-=0.01; 
    cpg->setReflex(reflex);
}

void modularController::increaseReflex(){
    reflex+=0.01; 
    cpg->setReflex(reflex);
}

void modularController::setReflex(double MI){
    reflex = MI; 
    cpg->setReflex(reflex);
}

void modularController::enableForceFeedback(){
    cpg->enableForceFeedback();
}

void modularController::disableForceFeedback(){
    cpg->disableForceFeedback();
}

double modularController::getAlpha(){
    return alpha; 
}

double modularController::getPhi(){
    return phi; 
}

double modularController::getReflex(){
    return reflex;
}

vector<double> modularController::getCpgOutput(){
    return cpgVal;
}

vector<double> modularController::getCpgPrevOutput(){
    return cpgValOld;
}

vector<double> modularController::getCpgActivation(){
    return cpg->getActivation();
}

void modularController::calculateCPG(){
    // Calculate the CPG values        
    cpgValOld = cpgVal;
    cpgFlipOld[0] = -1*cpgVal[0];
    cpgFlipOld[1] = -1*cpgVal[1];

    cpgVal = cpg->calculate();

    cpgFlip[0] = -1*cpgVal[0];
    cpgFlip[1] = -1*cpgVal[1];

}

double modularController::meanFilter(double dataInput){

    double sumofvalues = 0;
    meanList[0] = dataInput;
    std::rotate(meanList.begin(), meanList.begin()+1, meanList.end());

    for(unsigned int k = 0; k < meanList.size(); k++){
        sumofvalues+= meanList[k];
    }
    sumofvalues/= meanSize;

    return sumofvalues;
}

void modularController::calculateForceFeedback( double x){
    // Calculate the CPG values        
    cpgValOld = cpgVal;
    cpgFlipOld[0] = -1*cpgVal[0];
    cpgFlipOld[1] = -1*cpgVal[1];

    cpgVal = cpg->calculatePhaseAdaptation(x);

    cpgFlip[0] = -1*cpgVal[0];
    cpgFlip[1] = -1*cpgVal[1];

}

void modularController::stopRobot(){
    l1 = l2 = 0; 
}


void modularController::setScale(double val){
    scale = val;
}

void modularController::moveBackward(){
    l1 = l2 = -1; 
}

void modularController::moveForward(){
    l1 = l2 = 1; 
}


void modularController::moveLeftSoft(double speed){
    l1 = speed; 
    l2 = 1; 
}

void modularController::moveLeft(){
    l1 = -1; 
    l2 = 1; 
}

void modularController::moveRightSoft(double speed){
    l1 = 1; 
    l2 = speed;
}

void modularController::moveRight(){
    l1 = 1;
    l2 = -1; 
}

std::vector<double> modularController::getNewAndOldCpgValue(){
    std::vector<double> val{cpgVal[0], cpgValOld[0]};
    return val;
}


void modularController::step(int motorname, double *y){

    // Controlling up/down 
    if (cpgValOld[0] - cpgVal[0] > 0)
        y[motorname + 6] = -0.9*scale;
    else
        y[motorname + 6] = cpgVal[0]*scale;
    
    // Controlling forward/backwards
    if(motorname > 2)
        y[motorname] = cpgVal[0]*l1*scale;
    else 
        y[motorname] = cpgVal[0]*l2*scale;


    // Controlling in/out

    y[motorname + 12] = 0; 
    

}

void modularController::stepDelayed(int motorname, double *y, std::vector<double> val){

    // Controlling up/down 
    if (val[1] -val[0] > 0)
        y[motorname + 6] = -0.9*scale;
    else
        y[motorname + 6] = val[0]*scale;
    
    // Controlling forward/backwards
    if(motorname > 2)
        y[motorname] = val[0]*l1*scale;
    else 
        y[motorname] = val[0]*l2*scale;


    // Controlling in/out

    y[motorname + 12] = 0; 
    
}

modularController::~modularController(){
    delete cpg;
}
