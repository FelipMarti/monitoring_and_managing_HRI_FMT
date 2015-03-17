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









