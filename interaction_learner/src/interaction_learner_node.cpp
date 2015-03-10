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
    lastCommandData.resize(0);    
    usrAnnounceData.resize(0);
    usrGestureData.resize(0);
    headingAdjData.resize(0);
    distanceAdjData.resize(0);
    usrPresent.resize(0);
    
}


InteractionLearner::~InteractionLearner (void)
{
}


void InteractionLearner::read_data_callback(const data_parser::DataParsed& msg) {
    ROS_INFO("[InteractionLearner] I've heard something");

    if (msg.data[0].id=="NoMOAR!") {
        // Generate Bayesian Network
        ROS_INFO("[InteractionLearner] Generating BN");
//TODO: Fill vars with ints, not strings
//        dataBayesianNetwork.AddIntVar("Last Command",&lastCommandData);
    }
    else { 
        // Filling Bayesian Network learning variables
        lastCommandData.push_back("none");
        usrAnnounceData.push_back("no");
        usrGestureData.push_back("none");
        headingAdjData.push_back("no");
        distanceAdjData.push_back("no");
        usrPresent.push_back("none");
    
        for (int i=0;i<msg.data.size();++i) {
            if (msg.data[i].id == "usr_cmd") {
                lastCommandData.pop_back();
                lastCommandData.push_back(msg.data[i].list[0].text);
            }
            if (msg.data[i].id == "usr_announce") {
                usrAnnounceData.pop_back();
                usrAnnounceData.push_back("yes");
            }
            if (msg.data[i].id == "usr_gesture") {
                usrGestureData.pop_back();
                usrGestureData.push_back(msg.data[i].list[0].text);
            }
            if (msg.data[i].id == "usr_mov") {
    
                std::size_t found1=msg.data[i].list[0].text.find("adj");
                std::size_t found2=msg.data[i].list[0].text.find("adjustment");
                if ( found1!=std::string::npos or found2!=std::string::npos ) {
                    headingAdjData.pop_back();
                    headingAdjData.push_back("yes");
                }
    
                found1=msg.data[i].list[0].text.find("closer");
                found2=msg.data[i].list[0].text.find("further");
                if ( found1!=std::string::npos or found2!=std::string::npos ) {
                    distanceAdjData.pop_back();
                    distanceAdjData.push_back("yes");
                }
    
            }
            if (msg.data[i].id == "usr_present") {
                usrPresent.pop_back();
                usrPresent.push_back(msg.data[i].list[0].text);
            }
    
        }
    }


//////////////////////DEBUG
for( std::vector<std::string>::const_iterator i = lastCommandData.begin(); 
     i != lastCommandData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = usrAnnounceData.begin(); 
     i != usrAnnounceData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = usrGestureData.begin(); 
     i != usrGestureData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = headingAdjData.begin(); 
     i != headingAdjData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
for( std::vector<std::string>::const_iterator i = distanceAdjData.begin(); 
     i != distanceAdjData.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;

for( std::vector<std::string>::const_iterator i = usrPresent.begin(); 
     i != usrPresent.end(); 
     ++i)
     std::cout << *i << ' ';
     std::cout<<std::endl;
//////////////////////ENDDEBUG
        

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


