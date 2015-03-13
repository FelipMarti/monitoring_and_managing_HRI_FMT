/*
 *      This is de Delta-3 state publisher node. It's subscribed to 
 *      delta3 axis_states, and publishes delta3 joint_states
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
    
    int Main();




};


#endif
