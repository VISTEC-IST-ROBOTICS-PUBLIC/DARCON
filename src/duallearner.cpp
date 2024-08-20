#include "duallearner.h"
#include <chrono>


#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::DualLearner: Constructor
/// \param w_: vector of 4 elements, CPG weights.
///
DualLearner::DualLearner(std::vector<double> w_, bool closeLoop){
    w = w_;
    lowpass_n = 3; 
    filtererror_n = 5;
    lowpass = std::vector<double>(lowpass_n);
    filtererror = std::vector<double>(filtererror_n);

    Af = 0.59;
    As = 0.9972;
    Bf = 0.005;
    Bs = 0.0005;

    liftamplitude = 0.3;
    wideamplitude = 0.3;


}

//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::rescale:
/// \param oldMax: original upper limit
/// \param oldMin: original lower limit
/// \param newMax: required upper limit
/// \param newMin: required lower limit
/// \param Parameter:
///

double DualLearner::rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter){
    return (((newMax-newMin)*(parameter-oldMin))/(oldMax-oldMin))+newMin;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::DualLearner: constructor overload
/// \param w_: vector of 4 elements, CPG weights.
/// \param learnParams_: vector of 4 elements, SF weight learning parameters.
///                      if vector size < 4, set noLearning and set SFweight
///                      to LearnParams_ .
///
DualLearner::DualLearner(std::vector<double> w_, std::vector<double> learnParams_, bool closeLoop, int index, int rand){
    legIndex = index;
    w = w_;
    if(learnParams_.size() == 4 ){
        Af = learnParams_[0];
        As = learnParams_[1];
        Bf = learnParams_[2];
        Bs = learnParams_[3];

    }else{
        learning_SF=false;
        SFweight = learnParams_[0];
    }

    lowpass_n = 4;
    filtererror_n = 5;
    lowpass = std::vector<double>(lowpass_n);
    filtererror = std::vector<double>(filtererror_n);


    if(learning_rbfn){
        std::vector<std::vector<double>> empty;
        rbfNet = new rbfn(index, empty, sigma[index]);
        rbfNet->train_rbfn();
        delete rbfNet;
    }

    rbfNet = new rbfn(index, sigma[index]);
    rbfNetSemicircle = new rbfn_semicircle(index, 0.2, 41);

    for (size_t i = 0; i < rand; i++)
    {
        CPG_step(); 
    }
        

}


//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::CPG_step
///
void DualLearner::CPG_step(){

    prevC1_ = C1_;
    prevC2_ = C2_;

    a1_ = C1_*w[0]+C2_*w[2]+B1_;
    a2_ = C2_*w[1]+C1_*w[3]+B2_;
    
    C1_ = tanh(a1_);
    C2_ = tanh(a2_);

    return;
}


//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::CPG_step
/// \param input1   Input to neuron 1 on CPG
/// \param input2   Input to neuron 2 on CPG
///
void DualLearner::CPG_step(double input1, double input2){
    
    // ************************** MI adaptation **************************** //
    // pls see adaptiveSF.cpp file to check MI of system
    // MI_adapt = true;
    if(MI_adapt == true){
        // MI = 0.01;
        w[2] = 0.18+MI;
        w[3] = -0.18-MI;
        // printf("\nJoe");
        // reduce original system --> no extention term || no feedback term
        input1 = 0;
        input2 = 0; 
    } // If for overwrite MI
    



    // ************************************************** //

    a1 = C1*w[0]+C2*w[2]+B1+input1; //for N1 = (input of N1 * w11) + (input of N2 * w12) + Sensor Input-1
    a2 = C2*w[1]+C1*w[3]+B2+input2; //for N2 = (input of N2 * w22) + (input of N1 * w21) + Sensor Input-2
    //*/
    C1 = tanh(a1);
    C2 = tanh(a2);


    return;
} // CPG_step function



//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::control_step - simpler version
/// \param forceSensorData  Contact sensor data
/// \param CPGinput1        Input to neuron 1 on CPG
/// \param CPGinput2        Input to neuron 2 on CPGtopop
/// \return                 Vector of motor commands
///
std::vector<double> DualLearner::control_step(double forceSensorData, double time, bool is_set_gamma, double gamma){


    // if force < 0 must be 0 
    double raw_filteredData = sFeedFilter(forceSensorData) ; // < 0 ? 0.01 : sFeedFilter(forceSensorData); 


    //store previous CPG signal values
    prevC1 = C1;
    prevC2 = C2;

    motorPosPrev = motorOutput[0]; 

    // ***************************************************** //
    //****** Set gamma from Outside  *********//

    if(is_set_gamma == true){
        if(MI == 0){
            MI = 0.05;
        }
        
        MI = gamma;
    }





    // ******************************************************* //
    //             FILTER FORCE SENSOR FROM 0 - 1              //
    // ******************************************************* //

    double filteredData = raw_filteredData;

    filteredData = filteredData / (8/*body weight*/ + 1.6121 /*all leg part*/ + 6/*foot weight*/) * 0.5 /*stimulate factor*/;
    
    filteredData = filteredData> 0 ? filteredData : 0;


    // parameter

    double CPGinput1 = -(SFweight)*(filteredData) * cos(a1); // *cos(a1);  --> cos , sin 
    double CPGinput2 = -(SFweight)*(filteredData) * sin(a2); // *sin(a2);  --> cos , cos 

    // ********************* //

    motorOutput[13] = roll_degree;                     // data[10]
    motorOutput[14] = pitch_degree;                    // data[11]
    
    motorOutput[15] =  raw_filteredData;               // data[12]
    motorOutput[16] =  filteredData;                   // data[13]
    motorOutput[17] =  force_filtered_signal;          // data[14]
    motorOutput[18] =  roll_filtered_signal;           // data[15]
    motorOutput[19] =  E_angle;                        // data[16]
    motorOutput[20] =  raw_offset;                     // data[17]
    motorOutput[21] =  prev_offset_TCjoint;            // data[18]
    motorOutput[22] =  (offset_TCjoint*M_PI/180);      // data[19]

    double store_SFweight = SFweight;
    
    CPG_step(CPGinput1, CPGinput2);
   

    std::vector<double> output;
    output = rbfNet->getNetworkOutput(cpgOutput);


    // ********************************************************** //
    //                    Middle leg Adaptation                   //
    // ********************************************************** //

    roll_degree = imu[0]*180/M_PI;
    pitch_degree = imu[1]*180/M_PI;
    yaw_degree = imu[2]*180/M_PI;
        
    double target_angle = 0.0;
    double current_angle = pitch_degree;
    E_angle = target_angle - current_angle;

    bool is_offset_adaptive = true;
    double g_s = leg_direction;
    // double g_s = 1;
    double decay_rate = 0.9995; // 0.998
    double stimulate_factor = 0.15;
    double theta_bias = 2;

    // *************************** Adaptive resilient control **************************** //
        
    if (is_offset_adaptive){
  
        force_filtered_signal = tanh(filteredData);
        roll_filtered_signal = sigmoid(roll_degree*(-g_s));
        
        raw_offset = -E_angle * roll_filtered_signal * force_filtered_signal; 

        offset_TCjoint = decay_rate * prev_offset_TCjoint + raw_offset*0.10;

        prev_offset_TCjoint = offset_TCjoint;

    } // If is_offset_adaptive

    // ********************************************************* //
 
    // ******* Update middle leg obset ******* //
    motorOutput[0] = output[0] + (offset_TCjoint*M_PI/180);
    motorOutput[1] = output[1];
    motorOutput[2] = output[2];

    // ********************************************************* //

    if(output.size() > 3){
        motorOutput[9] = output[3];                     // The z position from the RBFN
        motorOutput[10] = LowPassFilter(output[3]);     // The z position form the RBGN with lowpass filter for fwd model
    }
        

    if(time > 3.0){
        std::vector<double> result = LearningModel_step(raw_filteredData, false);
        fwdModelPredct = result[0];
        error = result[1];
        sensorVal = result[2];
        Gvalue = result[3];

        //*******************//
        K_s = result[4];  // Ks
        K_f = result[5];  // Kf

    }
    

    //Additional variables for Plotting
    motorOutput[3] = fwdModelPredct;        // Output of fwd Model
    motorOutput[4] = error;                 // Error btw fwd Model and Sensor
    motorOutput[5] = sensorVal;             // Actual filtered sensor value
    motorOutput[6] = store_SFweight; // SFweight;              // Weights for sensor feedback
    motorOutput[7] = Gvalue;                // G value
    motorOutput[8] = forceSensorData;       // Raw force data

    ////// motorOutput[9] = output[3];                     // The z position from the RBFN
    ////// motorOutput[10] = LowPassFilter(output[3]);     // The z position form the RBGN with lowpass filter for fwd model

    motorOutput[11] = K_s;  // Ks
    motorOutput[12] = K_f;  // Kf
    

    return motorOutput;
}

std::vector<double> DualLearner::getCpgOutput(){
    cpgOutput[0] = C1; 
    cpgOutput[1] = C2; 
    return cpgOutput; 
}


//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::LearningModel_step:
///     Sensory feedback strength learning phase
/// \param forceSensorData  Contact sensor data
///
/// Learning model--> compares real and predicted sensory values, that can be use to tune the sensory feedback strength
///

std::vector<double> DualLearner::LearningModel_step(double forceSensorData, bool delay){

    ModelOutput.clear();

    //1 Fwd Model                                                                              _    _    _
    double G = 0;
    G = motorOutput[10] > 0.005 ? 0 : 1;         // predict z-position from rbfn -->  _| |__| |__| |_

    coeff = 1.0;
    fwdMod = G;
    forceSensorData = forceSensorData > 5 ? 1 : 0;

    //3 calculate Error between predicted and actual sensor values
    efferenceCopy_error = (fwdMod-forceSensorData); // Error = forwardModel - (SFData * dslow * k)

    //4 Take absolute and lowpass of error --> this is good
    efferenceCopy_error = abs(efferenceCopy_error); 

    if(learning_SF){
        //5 Dual Learner (calculate W)
        // efferenceCopy_error = 0;

        Ks = Ks * As + efferenceCopy_error * Bs; // Ks(t+dt) = Ks(t)*As + error*Bs
        Kf = Kf * Af + efferenceCopy_error * Bf; // Kf(t+dt) = Kf(t)*Af + error*Bf
        W = (Ks + Kf); 

        //6 update weight
        SFweight = W;

    }

    // *********************** MI adapt dual learner ********************* //
    if (MI_adapt){
        // Fast setup constant 
        ds = 0.99 ;
        ls = 0.01;
        df = 0.999;
        lf = 0.005;
        Ms = ds * Ms  +  efferenceCopy_error * ls;
        MI = Ms + Mf;
    }

    ModelOutput.push_back(fwdMod);                     // index 0
    ModelOutput.push_back(efferenceCopy_error);        // index 1
    ModelOutput.push_back(forceSensorData);            // index 2
    ModelOutput.push_back(G);                          // index 3

    //******************//
    ModelOutput.push_back(Ks);                         // index 4
    ModelOutput.push_back(Kf);                         // index 5
    

    return ModelOutput;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::reset
/// Reset CPG
///
void DualLearner::reset(){
    B1 = 0.01;
    B2 = 0.01;
    C1 = 0;
    C2 = 0;
    return;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::sFeedFilter: Mean filter
/// \param dataInput    Filter input
/// \return             Filtered signal
///
double DualLearner::sFeedFilter(double dataInput){

    double sumofvalues = 0;
    lowpass[0] = dataInput;
    std::rotate(lowpass.begin(), lowpass.begin()+1, lowpass.end());

    for(unsigned int k = 0; k < lowpass.size(); k++){
        sumofvalues+= lowpass[k];
    }
    sumofvalues/= lowpass_n;

    return sumofvalues;
}


//////////////////////////////////////////////////////////////////////////////
/// \brief DualLearner::LowPassFilter: Low pass filter
/// \param dataInput    The z-value of the foot
/// \return             Filtered z-value
///
double DualLearner::LowPassFilter(double Input){

    double x = 0.30; 
    LowPassError = x*Input + (1-x)*LowPassError;

    return LowPassError;
}



double DualLearner::Derivative(double sense){

    if(prevsense != sense){
        derivSense = prevsense - sense ;
    }

    prevsense = sense;

    return derivSense;
}

double DualLearner::RBF(double x, double min, double max){
    double x_mean = (max+min)/2;
    double std_base = (max-min);
    double K = exp(  -pow((x-x_mean),2) / std_base  );
    return K;
}


double DualLearner::stepFunction(double value, double threshold){
    if(value >= threshold){
        return 1.0;
    }
    else{
        return 0.0;
    }

} //stepFunction


/**
 * Find maximum between two numbers.
 */
double DualLearner::max(double num1, double num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

/**
 * Find minimum between two numbers.
 */
double DualLearner::min(double num1, double num2) 
{
    return (num1 > num2 ) ? num2 : num1;
}


/**
 * Find sigmoid transfer function
 */

double DualLearner::sigmoid(double x)
{
    return 1/(1+exp(x));
}


double DualLearner::thresholdF(double input, double min, double max)
{
    double _min = 0;
    double _max = 0;

    if(min > max){
        _min = max;
        _max = min;
    }
    else{
        _min = min;
        _max = max;
    }

    if( input > _min && input < _max){
        return 0;
    }
    else{
        return 1;
    }
}
