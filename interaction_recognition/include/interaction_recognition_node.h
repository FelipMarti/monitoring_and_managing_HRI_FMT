/*
 *      This node is used to perform inference with a bayesian network.
 *      It's subscribed to interaction_monitor (annotations topic)
 *      to obtain data and then performes inference with a BN using SMILE libraries
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 */


#ifndef _INTERACTION_RECOGNITION_NODE_HPP
#define _INTERACTION_RECOGNITION_NODE_HPP

#include "ros/ros.h"
#include "data_parser/DataParsed.h"
#include "smile.h"

class InteractionRecognition {

private:

    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber sub; 
    void perform_inference_callback (const data_parser::DataParsed& msg);

    /// Variables
    // Bayesian Network
    DSL_network theNet;

    // Statistics
    int Stats_Results[9]; 
    // Stats_Results[0] => Correct; 
    // Stats_Results[1] => Epic Failure; 
    // Stats_Results[2] => 2 Similar; 
    // Stats_Results[3] => 3 Similar; 
    // Stats_Results[4] => 4 Similar; 
    // Stats_Results[5] => Unknown has category; 
    // Stats_Results[6] => 2 Similar but not the category, so FAIL; 
    // Stats_Results[7] => 3 Similar but not the category, so FAIL; 
    // Stats_Results[8] => WTF!; 
    
    // Dictionaries
    std::map<std::string,int> ObjCategory;
    void write_category_dictionary(void);
    std::map<std::string,int> GestureCategory;
    void write_gesture_dictionary(void);
    std::map<std::string,int> CommandCategory;
    void write_command_dictionary(void);



public:

    /**
     *  InteractionLearner Constructor 
     */
    InteractionRecognition (void);

    /**
     *  InteractionLearner Destructor 
     */
    ~InteractionRecognition (void);

    int Main (const char* path);


};


#endif



