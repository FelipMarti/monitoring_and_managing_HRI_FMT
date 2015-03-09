/*
 *      This is de Delta-3 state publisher node. It's subscribed to 
 *      delta3 axis_states, and publishes delta3 joint_states
 */

#ifndef _INTERACTION_LEARNER_NODE_HPP
#define _INTERACTION_LEARNER_NODE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "smile.h"
#include "data_parser/DataParsed.h"

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
    std::vector<std::string> LastCommandData;
    std::vector<std::string> UsrAnnounceData;
    std::vector<std::string> UsrGestureData;
    std::vector<std::string> HeadingAdjData;
    std::vector<std::string> DistanceAdjData;
    std::vector<std::string> UsrPresent;



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
