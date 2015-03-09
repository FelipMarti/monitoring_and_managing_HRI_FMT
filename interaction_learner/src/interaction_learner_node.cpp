/*
 *      This node is used to train a bayesian network.
 *      It's subscribed to interaction_monitor (annotations topic)
 *      to obtain data and then generates a BN using SMILEARN libraries
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 */


#include "interaction_learner_node.h"


InteractionLearner::InteractionLearner (void)
{

    // Init Subscriber
    this->data_sub = this->n.subscribe("annotations", 10,
                                &InteractionLearner::read_data_callback, this);

    // Init Bayesian Network learning variables
    LastCommandData.resize(0);    
    UsrAnnounceData.resize(0);
    UsrGestureData.resize(0);
    HeadingAdjData.resize(0);
    DistanceAdjData.resize(0);
    UsrPresent.resize(0);
    
}



InteractionLearner::~InteractionLearner (void)
{
}

void InteractionLearner::read_data_callback(const data_parser::DataParsed& msg) {
    ROS_INFO("[InteractionLearner] I've heard something");

    // Filling Bayesian Network learning variables
    LastCommandData.push_back("none");
    UsrAnnounceData.push_back("no");
    UsrGestureData.push_back("none");
    HeadingAdjData.push_back("no");
    DistanceAdjData.push_back("no");
    UsrPresent.push_back("none");

    for (int i=0;i<msg.data.size();++i) {
        if (msg.data[i].id == "usr_cmd") {
            LastCommandData.pop_back();
            LastCommandData.push_back(msg.data[i].list[0].text);
        }
        if (msg.data[i].id == "usr_announce") {
            UsrAnnounceData.pop_back();
            UsrAnnounceData.push_back("yes");
        }
        if (msg.data[i].id == "usr_gesture") {
            UsrGestureData.pop_back();
            UsrGestureData.push_back(msg.data[i].list[0].text);
        }
        if (msg.data[i].id == "usr_mov") {
            HeadingAdjData.pop_back();
            HeadingAdjData.push_back("yes");

            std::size_t found1=msg.data[i].list[0].text.find("closer");
            std::size_t found2=msg.data[i].list[0].text.find("further");
            if ( found1!=std::string::npos or found2!=std::string::npos ) {
                DistanceAdjData.pop_back();
                DistanceAdjData.push_back("yes");
            }

        }
        if (msg.data[i].id == "usr_present") {
            UsrPresent.pop_back();
            UsrPresent.push_back(msg.data[i].list[0].text);
        }

    }

for( std::vector<std::string>::const_iterator i = LastCommandData.begin(); 
     i != LastCommandData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = UsrAnnounceData.begin(); 
     i != UsrAnnounceData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = UsrGestureData.begin(); 
     i != UsrGestureData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = HeadingAdjData.begin(); 
     i != HeadingAdjData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = DistanceAdjData.begin(); 
     i != DistanceAdjData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;

for( std::vector<std::string>::const_iterator i = UsrPresent.begin(); 
     i != UsrPresent.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
        

}

int InteractionLearner::Main ()
{
    


    // Wait for callbacks
    ros::spin();
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "interaction_learner");
    
    InteractionLearner foo;
    return foo.Main();

}


