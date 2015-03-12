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
        std::vector<std::string> vecNames;
        vecNames.push_back("lastCommand");
        vecNames.push_back("usrAnnounce");
        vecNames.push_back("usrGesture");
        vecNames.push_back("headingAdj");
        vecNames.push_back("distanceAdj");
        vecNames.push_back("usrPresent");
//        dataBayesianNetwork.Reshape(vecNames);
    
    llista.sort();
    llista.unique();
for (std::list<std::string>::iterator it=llista.begin(); it != llista.end(); ++it)
        std::cout << ' ' << *it;
std::cout<<std::endl;
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
                llista.push_back(msg.data[i].list[0].text);
            }
    
        }
    }



//////////////////////DEBUG 
/*
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
*/
//////////////////////ENDDEBUG
        

}


int InteractionLearner::Main ()
{

    write_category_dictionary();


    // Wait for callbacks
    ros::spin();
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "interaction_learner");
    
    InteractionLearner foo;
    return foo.Main();

}








void InteractionLearner::write_category_dictionary()
{

    ObjCategory["LUCAS_entrance"]="workspace";
    ObjCategory["LUCAS_room"]="region";
    ObjCategory["Xerox_machine"]="workspace";
    ObjCategory["armchair"]="object";
    ObjCategory["backboard"]="workspace";
    ObjCategory["basin"]="workspace";
    ObjCategory["basket"]="object";
    ObjCategory["beer"]="object";
    ObjCategory["big_table"]="workspace";
    ObjCategory["bin"]="object";
    ObjCategory["blue_chair"]="object";
    ObjCategory["board"]="unknown";
    ObjCategory["book"]="object";
    ObjCategory["books"]="object";
    ObjCategory["bookshelf"]="workspace";
    ObjCategory["bookshelves"]="workspace";
    ObjCategory["bottle"]="object";
    ObjCategory["bottle_of_water"]="object";
    ObjCategory["box"]="object";
    ObjCategory["buttons"]="unknown";
    ObjCategory["cable"]="object";
    ObjCategory["chair"]="object";
    ObjCategory["chairs"]="object";
    ObjCategory["chart"]="workplace";
    ObjCategory["coffe_machine"]="unknown";
    ObjCategory["coffe_maker"]="workplace";
    ObjCategory["coffee"]="object";
    ObjCategory["coffee-machine"]="unknown";
    ObjCategory["coffee_machine"]="unknown";
    ObjCategory["coffee_maker"]="unknown";
    ObjCategory["coffee_mug"]="object";
    ObjCategory["coffee_room"]="region";
    ObjCategory["coffeemachine"]="unknown";
    ObjCategory["coffeemaker"]="workplace";
    ObjCategory["computer"]="workspace";
    ObjCategory["computer_keyboard"]="object";
    ObjCategory["computer_monitor"]="workplace";
    ObjCategory["conference_room"]="region";
    ObjCategory["conference_table"]="workplace";
    ObjCategory["control_remote"]="object";
    ObjCategory["copy_machine"]="workspace";
    ObjCategory["copy_room"]="region";
    ObjCategory["copy_room_copy_machine_paper"]="workplace";
    ObjCategory["copying_machine"]="workplace";
    ObjCategory["copying_room"]="workplace";
    ObjCategory["cup"]="object";
    ObjCategory["cupboard"]="workplace";
    ObjCategory["desk"]="unknown";
    ObjCategory["desk_chair"]="object";
    ObjCategory["door"]="workplace";
    ObjCategory["drink"]="object";
    ObjCategory["dust bin"]="unknown";
    ObjCategory["dustbin"]="object";
    ObjCategory["entrance"]="workspace";
    ObjCategory["entrance_door"]="workspace";
    ObjCategory["entrance_to_LUCAS"]="workplace";
    ObjCategory["entrance_to_the_LUCAS"]="workplace";
    ObjCategory["entrance_to_the_LUCAS_room"]="workplace";
    ObjCategory["eraser"]="object";
    ObjCategory["file"]="object";
    ObjCategory["fridge"]="workplace";
    ObjCategory["garbage_can"]="object";
    ObjCategory["glass"]="object";
    ObjCategory["glass_of_water"]="object";
    ObjCategory["glasses"]="object";
    ObjCategory["hallway"]="region";
    ObjCategory["hanger"]="workplace";
    ObjCategory["helmet"]="object";
    ObjCategory["internet_cable"]="object";
    ObjCategory["kensington_lock"]="object";
    ObjCategory["key"]="object";
    ObjCategory["key_chain"]="object";
    ObjCategory["keys"]="object";
    ObjCategory["kitchen"]="unknown";
    ObjCategory["labtop"]="object";
    ObjCategory["labtop bag"]="object";
    ObjCategory["lamp"]="object";
    ObjCategory["light"]="object";
    ObjCategory["light_switches"]="unknown";
    ObjCategory["lucas_entrance"]="workplace";
    ObjCategory["lunch_room"]="region";
    ObjCategory["markers"]="object";
    ObjCategory["meeting_room"]="region";
    ObjCategory["menu"]="unknown";
    ObjCategory["microwave"]="workplace";
    ObjCategory["microwave_oven"]="workplace";
    ObjCategory["milk"]="object";
    ObjCategory["monitor"]="workplace";
    ObjCategory["mouse"]="object";
    ObjCategory["no_fan"]="unknown";
    ObjCategory["notebook"]="unknown";
    ObjCategory["object_printer"]="object";
    ObjCategory["office"]="region";
    ObjCategory["office_room"]="region";
    ObjCategory["oven"]="workplace";
    ObjCategory["own_arm"]="unknown";
    ObjCategory["paper"]="object";
    ObjCategory["paper_bin"]="object";
    ObjCategory["paper_clip"]="object";
    ObjCategory["papers"]="object";
    ObjCategory["pen"]="object";
    ObjCategory["phone"]="object";
    ObjCategory["photocopy_machine"]="workplace";
    ObjCategory["photocopying_machine"]="workplace";
    ObjCategory["postit"]="object";
    ObjCategory["printer"]="unknown";
    ObjCategory["printer_machine"]="unknown";
    ObjCategory["printer_room"]="workplace";
    ObjCategory["printing_room"]="workplace";
    ObjCategory["projector"]="workplace";
    ObjCategory["refrigerator"]="workplace";
    ObjCategory["remote"]="object";
    ObjCategory["remote_control"]="object";
    ObjCategory["room"]="region";
    ObjCategory["room_4105"]="region";
    ObjCategory["scanning_machine"]="workplace";
    ObjCategory["scissors"]="object";
    ObjCategory["screen"]="unknown";
    ObjCategory["seminar_room"]="region";
    ObjCategory["shelf"]="workplace";
    ObjCategory["sink"]="workplace";
    ObjCategory["small_pen"]="object";
    ObjCategory["small_table"]="workplace";
    ObjCategory["stapler"]="object";
    ObjCategory["table"]="workspace";
    ObjCategory["table_lamp"]="object";
    ObjCategory["tape"]="object";
    ObjCategory["telephone"]="object";
    ObjCategory["towel"]="object";
    ObjCategory["trash_bin"]="object";
    ObjCategory["trashbin"]="object";
    ObjCategory["waiting_room"]="region";
    ObjCategory["wall"]="unknown";
    ObjCategory["water"]="object";
    ObjCategory["water_bottle"]="object";
    ObjCategory["whiteboard"]="workplace";
    ObjCategory["window"]="workplace";


}
