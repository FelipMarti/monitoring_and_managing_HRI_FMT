/*
 *      This node is used to train a bayesian network.
 *      It's subscribed to interaction_monitor (annotations topic)
 *      to obtain data and then generates a BN using SMILEARN libraries
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 */


#ifndef _INTERACTION_LEARNER_NODE_HPP
#define _INTERACTION_LEARNER_NODE_HPP

#include "ros/ros.h"
#include "data_parser/DataParsed.h"
#include "smile.h"
#include "smilearn.h"

class InteractionLearner {

private:
    
    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber data_sub;
    void read_data_callback(const data_parser::DataParsed& msg); 

    // [publisher attributes]
    ros::Publisher joint_pub;


    /// Variables
    // Bayesian Network
    DSL_dataset dataBayesianNetwork;
    void set_nomenclature_to_dataset(void);
    std::string pathBN;
    DSL_network net;
    
    // Vectors to store data to train the BN 
    std::vector<int> lastCommandData;
    std::vector<int> usrAnnounceData;
    std::vector<int> usrGestureData;
    std::vector<int> headingAdjData;
    std::vector<int> distanceAdjData;
    std::vector<int> category;

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
    InteractionLearner (void);

    /**
     *  InteractionLearner Destructor 
     */
    ~InteractionLearner (void);
    
    int Main (const char* path, const char* file);




};


#endif
