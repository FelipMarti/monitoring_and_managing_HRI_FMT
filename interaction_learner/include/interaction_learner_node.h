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


    // Variables
    // Bayesian Network
    DSL_dataset dataBayesianNetwork;

    std::vector<std::string> lastCommandData;
    std::vector<std::string> usrAnnounceData;
    std::vector<std::string> usrGestureData;
    std::vector<std::string> headingAdjData;
    std::vector<std::string> distanceAdjData;
    std::vector<std::string> usrPresent;

    // Dictionary
    std::map<std::string,std::string> ObjCategory;
    void write_category_dictionary(void);
    
    //Debug
    std::list<std::string> llista;


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
