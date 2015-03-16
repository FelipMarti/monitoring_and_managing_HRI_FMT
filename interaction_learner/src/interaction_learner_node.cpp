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
    category.resize(0);
    
}


InteractionLearner::~InteractionLearner (void)
{
}


void InteractionLearner::read_data_callback(const data_parser::DataParsed& msg) {
    ROS_INFO("[Interaction_Learner] I've heard something");

    if (msg.data[0].id=="NoMOAR!") {

        /// Generating Bayesian Network
        // Fill vars with data
        dataBayesianNetwork.SetNumberOfRecords(category.size());
        for (int i=0; i<category.size(); i++) {
            dataBayesianNetwork.SetInt(0,i,lastCommandData[i]);
            dataBayesianNetwork.SetInt(1,i,usrAnnounceData[i]);
            dataBayesianNetwork.SetInt(2,i,usrGestureData[i]);
            dataBayesianNetwork.SetInt(3,i,headingAdjData[i]);
            dataBayesianNetwork.SetInt(4,i,distanceAdjData[i]);
            dataBayesianNetwork.SetInt(5,i,category[i]);
        }

        ROS_INFO("[Interaction_Learner] with %d Vars and %d Annotations",
                 dataBayesianNetwork.GetNumberOfVariables(), 
                 dataBayesianNetwork.GetNumberOfRecords());


        // Match the data set and the network:
        std::vector<DSL_datasetMatch> matches;
        std::string err;
        if (dataBayesianNetwork.MatchNetwork(net, matches, err) != DSL_OKAY) {
            ROS_ERROR("[Interaction_Learner] Cannot match network... exiting.");
            ROS_ERROR("[Interaction_Learner] %s",err.c_str());
            exit(1);
        }
        else {
            ROS_INFO("[Interaction_Learner] BN matched successfully :)");
        }

        // Train BN
        ROS_INFO("[Interaction_Learner] Training BN");
        DSL_network result;
        DSL_nb naive;
        naive.classVariableId = "category";
        if (naive.Learn(dataBayesianNetwork,result)!=DSL_OKAY) {
            ROS_ERROR("[Interaction_Learner] Learning failed");
            exit(1);
        }
        else {
            pathBN=pathBN+"naivebayes.xdsl";
            result.WriteFile(pathBN.c_str());      
            ROS_INFO("[Interaction_Learner] BN generated in:");
            ROS_INFO("[Interaction_Learner] %s",pathBN.c_str());
            ROS_WARN("[Interaction_Learner] So, the node is stopped gently :)");
            exit(0);
        }

    }
    else { 
        // Filling Bayesian Network learning variables
        lastCommandData.push_back(5);   //none
        usrAnnounceData.push_back(0);   //no
        usrGestureData.push_back(5);    //none
        headingAdjData.push_back(0);    //no
        distanceAdjData.push_back(0);   //no
        category.push_back(3);          //Unknown
    
        for (int i=0;i<msg.data.size();++i) {
            if (msg.data[i].id == "usr_cmd") {
                lastCommandData.pop_back();
                lastCommandData.push_back(CommandCategory[msg.data[i].list[0].text]);
            }
            if (msg.data[i].id == "usr_announce") {
                usrAnnounceData.pop_back();
                usrAnnounceData.push_back(1);   //yes
            }
            if (msg.data[i].id == "usr_gesture") {
                usrGestureData.pop_back();
                usrGestureData.push_back(GestureCategory[msg.data[i].list[0].text]);
            }
            if (msg.data[i].id == "usr_mov") {
    
                std::size_t found1=msg.data[i].list[0].text.find("adj");
                std::size_t found2=msg.data[i].list[0].text.find("adjustment");
                if ( found1!=std::string::npos or found2!=std::string::npos ) {
                    headingAdjData.pop_back();
                    headingAdjData.push_back(1);    //yes
                }
    
                found1=msg.data[i].list[0].text.find("closer");
                found2=msg.data[i].list[0].text.find("further");
                if ( found1!=std::string::npos or found2!=std::string::npos ) {
                    distanceAdjData.pop_back();
                    distanceAdjData.push_back(1);   //yes
                }
    
            }
            if (msg.data[i].id == "usr_present") {
                category.pop_back();
                category.push_back(ObjCategory[msg.data[i].list[0].text]);
            }
        }

    }

}


int InteractionLearner::Main (const char* path, const char* file)
{

    // Path to write the new BN
    pathBN=path;
    
    // Open the BN network
    std::string full_path = (std::string) path + (std::string) file;
    if (net.ReadFile(full_path.c_str(), DSL_XDSL_FORMAT) != DSL_OKAY) {
        ROS_ERROR("[Interaction_Learner] Cannot read network... exiting.");
        exit(1);
    }
    else {
        ROS_INFO("[Interaction_Learner] BN opened successfully:");
        ROS_INFO("[Interaction_Learner] %s",full_path.c_str());
    }

    // Filling dicctionaries
    write_category_dictionary();
    write_gesture_dictionary();
    write_command_dictionary();

    // Preparing dataset for the BN
    set_nomenclature_to_dataset(); 

    // Wait for callbacks
    ros::spin();

}


int main(int argc, char** argv) 
{
    
    ros::init(argc, argv, "interaction_learner");
    if (argc != 3) {
        ROS_ERROR("[interaction_learner] usage: interaction_learner PATH Bayesian_Network");
        exit(1);
    }


    InteractionLearner foo;
    return foo.Main(argv[1], argv[2]);

}


/**
 *  Setting the correct nomenclature to match the dataset with the 
 *  Bayesian Network 
 */
void InteractionLearner::set_nomenclature_to_dataset()
{

    std::vector<std::string> vecNames;

    /// Node (var) lastCommand
    dataBayesianNetwork.AddIntVar("lastCommand");
    // states names
    vecNames.resize(0);
    vecNames.push_back("back");     // back     => 0
    vecNames.push_back("follow");   // follow   => 1
    vecNames.push_back("forward");  // forward  => 2
    vecNames.push_back("stop");     // stop     => 3
    vecNames.push_back("turn");     // turn     => 4
    vecNames.push_back("none");     // none     => 5
    dataBayesianNetwork.SetStateNames(0, vecNames);

    /// Node (var) usrAnnounceData 
    dataBayesianNetwork.AddIntVar("usrAnnounce");
    // states names
    vecNames.resize(0);
    vecNames.push_back("no");       // no  => 0
    vecNames.push_back("yes");      // yes => 1
    dataBayesianNetwork.SetStateNames(1, vecNames);

    /// Node (var) usrGesture 
    dataBayesianNetwork.AddIntVar("usrGesture");
    // states names
    vecNames.resize(0);
    vecNames.push_back("fingertip_point");  // fingertip_point  => 0
    vecNames.push_back("hand_point");       // hand_point       => 1
    vecNames.push_back("hold_item");        // hold_item        => 2
    vecNames.push_back("sweep_wave");       // sweep_wave       => 3
    vecNames.push_back("touch_full_hand");  // touch_full_hand  => 4
    vecNames.push_back("none");             // none             => 5
    dataBayesianNetwork.SetStateNames(2, vecNames);

    /// Node (var) headingAdjData
    dataBayesianNetwork.AddIntVar("headingAdj");
    // states names
    vecNames.resize(0);
    vecNames.push_back("no");       // no  => 0
    vecNames.push_back("yes");      // yes => 1
    dataBayesianNetwork.SetStateNames(3, vecNames);

    /// Node (var) distanceAdjData 
    dataBayesianNetwork.AddIntVar("distanceAdj");
    // states names
    vecNames.resize(0);
    vecNames.push_back("no");       // no  => 0
    vecNames.push_back("yes");      // yes => 1
    dataBayesianNetwork.SetStateNames(4, vecNames);

    /// Node (var) Category
    dataBayesianNetwork.AddIntVar("category");
    // states names
    vecNames.resize(0);
    vecNames.push_back("object");       // object    => 0 
    vecNames.push_back("region");       // region    => 1
    vecNames.push_back("workspace");    // workspace => 2
    vecNames.push_back("unknown");      // unknown   => 3
    dataBayesianNetwork.SetStateNames(5, vecNames);

}


/**
 *  Dictionary to define commands because some annotations have typographic errors.
 *
 *  back    => 0
 *  follow  => 1
 *  forward => 2
 *  stop    => 3
 *  turn    => 4
 *  none    => 5
 *
 */
void InteractionLearner::write_command_dictionary()
{
    CommandCategory["back"]=0;          //back
    CommandCategory["follow"]=1;        //follow
    CommandCategory["forward"]=2;       //forward
    CommandCategory["stop"]=3;          //stop
    CommandCategory["stopp"]=3;         //stop
    CommandCategory["turn_around"]=4;   //turn
    CommandCategory["turn_left"]=4;     //turn
    CommandCategory["turn_right"]=4;    //turn
    CommandCategory["none"]=5;          //none
}


/**
 *  Dictionary to define gestures because some annotations have typographic errors.
 *
 *  fingertip_point => 0
 *  hand_point      => 1
 *  hold_item       => 2
 *  sweep_wave      => 3
 *  touch_full_hand => 4
 *  none            => 5
 *
 */
void InteractionLearner::write_gesture_dictionary()
{
    GestureCategory["fignertip_point"]=0;       //fingertip_point
    GestureCategory["fingertip_point"]=0;       //fingertip_point
    GestureCategory["hand_point"]=1;            //hand_point
    GestureCategory["hand_point?"]=1;           //hand_point
    GestureCategory["hold item"]=2;             //hold_item
    GestureCategory["hold_item"]=2;             //hold_item
    GestureCategory["sweep_wave"]=3;            //sweep_wave
    GestureCategory["touch_full_hand"]=4;       //touch_full_hand
    GestureCategory["touch_item_full_hand"]=4;  //touch_full_hand
    GestureCategory["touch_item_full_hand / hold_item"]=4;  //touch_full_hand
    GestureCategory["touch_item_full_hand?"]=4; //touch_full_hand
    GestureCategory["none"]=5;                  //none
}


/**
 *  Dictionary to define the presentation category in object, region or workspace.
 *  Besides, unknown category is defined when it was not confirmed by the robot, or
 *  could cause ambiguity
 *
 *  object      => 0
 *  region      => 1
 *  workspace   => 2
 *  unknown     => 3
 *
 */
void InteractionLearner::write_category_dictionary()
{
    ObjCategory["LUCAS_entrance"]=2;      //workspace
    ObjCategory["LUCAS_room"]=1;          //region
    ObjCategory["Xerox_machine"]=2;       //workspace
    ObjCategory["armchair"]=0;            //object
    ObjCategory["backboard"]=2;           //workspace
    ObjCategory["basin"]=2;               //workspace
    ObjCategory["basket"]=0;              //object
    ObjCategory["beer"]=0;                //object
    ObjCategory["big_table"]=2;           //workspace
    ObjCategory["bin"]=0;                 //object
    ObjCategory["blue_chair"]=0;          //object
    ObjCategory["board"]=3;               //unknown
    ObjCategory["book"]=0;                //object
    ObjCategory["books"]=0;               //object
    ObjCategory["bookshelf"]=2;           //workspace
    ObjCategory["bookshelves"]=2;         //workspace
    ObjCategory["bottle"]=0;              //object
    ObjCategory["bottle_of_water"]=0;     //object
    ObjCategory["box"]=0;                 //object
    ObjCategory["buttons"]=3;             //unknown
    ObjCategory["cable"]=0;               //object
    ObjCategory["chair"]=0;               //object
    ObjCategory["chairs"]=0;              //object
    ObjCategory["chart"]=2;               //workspace
    ObjCategory["coffe_machine"]=3;       //unknown
    ObjCategory["coffe_maker"]=2;         //workspace
    ObjCategory["coffee"]=0;              //object
    ObjCategory["coffee-machine"]=3;      //unknown
    ObjCategory["coffee_machine"]=3;      //unknown
    ObjCategory["coffee_maker"]=3;        //unknown
    ObjCategory["coffee_mug"]=0;          //object
    ObjCategory["coffee_room"]=1;         //region
    ObjCategory["coffeemachine"]=3;       //unknown
    ObjCategory["coffeemaker"]=2;         //workspace
    ObjCategory["computer"]=2;            //workspace
    ObjCategory["computer_keyboard"]=0;   //object
    ObjCategory["computer_monitor"]=2;    //workspace
    ObjCategory["conference_room"]=1;     //region
    ObjCategory["conference_table"]=2;    //workspace
    ObjCategory["control_remote"]=0;      //object
    ObjCategory["copy_machine"]=2;        //workspace
    ObjCategory["copy_room"]=1;           //region
    ObjCategory["copy_room_copy_machine_paper"]=2;//workspace
    ObjCategory["copying_machine"]=2;     //workspace
    ObjCategory["copying_room"]=2;        //workspace
    ObjCategory["cup"]=0;                 //object
    ObjCategory["cupboard"]=2;            //workspace
    ObjCategory["desk"]=3;                //unknown
    ObjCategory["desk_chair"]=0;          //object
    ObjCategory["door"]=2;                //workspace
    ObjCategory["drink"]=0;               //object
    ObjCategory["dust bin"]=3;            //unknown
    ObjCategory["dustbin"]=0;             //object
    ObjCategory["entrance"]=2;            //workspace
    ObjCategory["entrance_door"]=2;       //workspace
    ObjCategory["entrance_to_LUCAS"]=2;   //workspace
    ObjCategory["entrance_to_the_LUCAS"]=2;       //workspace
    ObjCategory["entrance_to_the_LUCAS_room"]=2;  //workspace
    ObjCategory["eraser"]=0;              //object
    ObjCategory["file"]=0;                //object
    ObjCategory["fridge"]=2;              //workspace
    ObjCategory["garbage_can"]=0;         //object
    ObjCategory["glass"]=0;               //object
    ObjCategory["glass_of_water"]=0;      //object
    ObjCategory["glasses"]=0;             //object
    ObjCategory["hallway"]=1;             //region
    ObjCategory["hanger"]=2;              //workspace
    ObjCategory["helmet"]=0;              //object
    ObjCategory["internet_cable"]=0;      //object
    ObjCategory["kensington_lock"]=0;     //object
    ObjCategory["key"]=0;                 //object
    ObjCategory["key_chain"]=0;           //object
    ObjCategory["keys"]=0;                //object
    ObjCategory["kitchen"]=3;             //unknown
    ObjCategory["labtop"]=0;              //object
    ObjCategory["labtop bag"]=0;          //object
    ObjCategory["lamp"]=0;                //object
    ObjCategory["light"]=0;               //object
    ObjCategory["light_switches"]=3;      //unknown
    ObjCategory["lucas_entrance"]=2;      //workspace
    ObjCategory["lunch_room"]=1;          //region
    ObjCategory["markers"]=0;             //object
    ObjCategory["meeting_room"]=1;        //region
    ObjCategory["menu"]=3;                //unknown
    ObjCategory["microwave"]=2;           //workspace
    ObjCategory["microwave_oven"]=2;      //workspace
    ObjCategory["milk"]=0;                //object
    ObjCategory["monitor"]=2;             //workspace
    ObjCategory["mouse"]=0;               //object
    ObjCategory["no_fan"]=3;              //unknown
    ObjCategory["notebook"]=3;            //unknown
    ObjCategory["object_printer"]=0;      //object
    ObjCategory["office"]=1;              //region
    ObjCategory["office_room"]=1;         //region
    ObjCategory["oven"]=2;                //workspace
    ObjCategory["own_arm"]=3;             //unknown
    ObjCategory["paper"]=0;               //object
    ObjCategory["paper_bin"]=0;           //object
    ObjCategory["paper_clip"]=0;          //object
    ObjCategory["papers"]=0;              //object
    ObjCategory["pen"]=0;                 //object
    ObjCategory["phone"]=0;               //object
    ObjCategory["photocopy_machine"]=2;   //workspace
    ObjCategory["photocopying_machine"]=2;//workspace
    ObjCategory["postit"]=0;              //object
    ObjCategory["printer"]=3;             //unknown
    ObjCategory["printer_machine"]=3;     //unknown
    ObjCategory["printer_room"]=2;        //workspace
    ObjCategory["printing_room"]=2;       //workspace
    ObjCategory["projector"]=2;           //workspace
    ObjCategory["refrigerator"]=2;        //workspace
    ObjCategory["remote"]=0;              //object
    ObjCategory["remote_control"]=0;      //object
    ObjCategory["room"]=1;                //region
    ObjCategory["room_4105"]=1;           //region
    ObjCategory["scanning_machine"]=2;    //workspace
    ObjCategory["scissors"]=0;            //object
    ObjCategory["screen"]=3;              //unknown
    ObjCategory["seminar_room"]=1;        //region
    ObjCategory["shelf"]=2;               //workspace
    ObjCategory["sink"]=2;                //workspace
    ObjCategory["small_pen"]=0;           //object
    ObjCategory["small_table"]=2;         //workspace
    ObjCategory["stapler"]=0;             //object
    ObjCategory["table"]=2;               //workspace
    ObjCategory["table_lamp"]=0;          //object
    ObjCategory["tape"]=0;                //object
    ObjCategory["telephone"]=0;           //object
    ObjCategory["towel"]=0;               //object
    ObjCategory["trash_bin"]=0;           //object
    ObjCategory["trashbin"]=0;            //object
    ObjCategory["waiting_room"]=1;        //region
    ObjCategory["wall"]=3;                //unknown
    ObjCategory["water"]=0;               //object
    ObjCategory["water_bottle"]=0;        //object
    ObjCategory["whiteboard"]=2;          //workspace
    ObjCategory["window"]=2;              //workspace
}
