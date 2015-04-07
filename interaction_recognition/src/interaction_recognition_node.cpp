/*
 *      This node is used to perform inference with a bayesian network.
 *      It's subscribed to interaction_monitor (annotations topic)
 *      to obtain data and then performes inference with a BN using SMILE libraries
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 */


#include "interaction_recognition_node.h"


InteractionRecognition::InteractionRecognition (void)
{

    // Init Subscriber
    this->sub = this->n.subscribe("annotations", 10,
                        &InteractionRecognition::perform_inference_callback, this);

    // Init Stats
    for (int i=0; i<9; i++) {
         Stats_Results[i]=0; 
    }

}


InteractionRecognition::~InteractionRecognition (void)
{
}


void InteractionRecognition::perform_inference_callback(const data_parser::DataParsed& msg)
{

    if (msg.data[0].id=="NoMOAR!") {
        ROS_WARN("[Interaction_Learner] No more data!");
        ROS_WARN("[Interaction_Learner] So, the node is stopped gently :)");
        ROS_INFO("[Interaction_Learner] ********** STATISTICS **********");
        ROS_INFO("[Interaction_Learner] %d are OK!!", Stats_Results[0]);
        ROS_INFO("[Interaction_Learner] %d are Epic Failure!!", Stats_Results[1]);
        ROS_INFO("[Interaction_Learner] %d are Similar between 2", Stats_Results[2]);
        ROS_INFO("[Interaction_Learner] %d are Similar between 3", Stats_Results[3]);
        ROS_INFO("[Interaction_Learner] %d are Similar between 4", Stats_Results[4]);
        ROS_INFO("[Interaction_Learner] %d are Unknown classified", Stats_Results[5]);
        ROS_INFO("[Interaction_Learner] %d are Similar between 2, but FAIL", Stats_Results[6]);
        ROS_INFO("[Interaction_Learner] %d are Similar between 3, but FAIL", Stats_Results[7]);
        ROS_INFO("[Interaction_Learner] %d WTF!!", Stats_Results[8]);
        ROS_INFO("[Interaction_Learner] ********** ********** **********");
        exit(0);
    }
    
    /// Performing Inference
    ROS_INFO("[Interaction_Recognition] Performing Inference");
    
    // Vars to get the handle of node
    int category = theNet.FindNode("category"); 
    int lastCommand = theNet.FindNode("lastCommand"); 
    int usrAnnounce = theNet.FindNode("usrAnnounce"); 
    int usrGesture = theNet.FindNode("usrGesture"); 
    int headingAdj = theNet.FindNode("headingAdj"); 
    int distanceAdj = theNet.FindNode("distanceAdj"); 

    // Vars to store data
    int lastCommandData=5;  //none
    int usrAnnounceData=0;  //no
    int usrGestureData=5;   //none
    int headingAdjData=0;   //no
    int distanceAdjData=0;  //no
    std::string categoryData;     
    
    // Obtaining data to sett the evidence
    for (int i=0;i<msg.data.size();++i) {

        if (msg.data[i].id == "usr_cmd") {
            lastCommandData = CommandCategory[msg.data[i].list[0].text];
        }
        if (msg.data[i].id == "usr_announce") {
            usrAnnounceData = 1;   //yes
        }
        if (msg.data[i].id == "usr_gesture") {
            usrGestureData = GestureCategory[msg.data[i].list[0].text];
        }
        if (msg.data[i].id == "usr_mov") {

            std::size_t found1=msg.data[i].list[0].text.find("adj");
            std::size_t found2=msg.data[i].list[0].text.find("adjustment");
            if ( found1!=std::string::npos or found2!=std::string::npos ) {
                headingAdjData = 1;    //yes
            }

            found1=msg.data[i].list[0].text.find("closer");
            found2=msg.data[i].list[0].text.find("further");
            if ( found1!=std::string::npos or found2!=std::string::npos ) {
                distanceAdjData = 1;   //yes
            }

        }
        if (msg.data[i].id == "usr_present") {
            categoryData = msg.data[i].list[0].text;
        }

    }

    // Setting evidence
    theNet.GetNode(lastCommand)->Value()->SetEvidence(lastCommandData);
    theNet.GetNode(usrAnnounce)->Value()->SetEvidence(usrAnnounceData);
    theNet.GetNode(usrGesture)->Value()->SetEvidence(usrGestureData);
    theNet.GetNode(headingAdj)->Value()->SetEvidence(headingAdjData);
    theNet.GetNode(distanceAdj)->Value()->SetEvidence(distanceAdjData);

    // Update the network
    theNet.UpdateBeliefs();

    // Get the result values
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(category)->Value());
    DSL_idArray *theNames;
    theNames = theNet.GetNode(category)->Definition()->GetOutcomesNames();

    int objectIndex = theNames->FindPosition("object"); 
    int regionIndex = theNames->FindPosition("region"); 
    int workspaceIndex = theNames->FindPosition("workspace"); 
    int unknownIndex = theNames->FindPosition("unknown"); 

    double P_CategoryIs[4];
    // 0 P_CategoryIsObject
    // 1 P_CategoryIsRegion
    // 2 P_CategoryIsWorkspace
    // 3 P_CategoryIsUnknown

    theCoordinates[0] = objectIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = object)
    P_CategoryIs[0] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = object) = %f",P_CategoryIs[0]);

    theCoordinates[0] = regionIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = region)
    P_CategoryIs[1] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = region) = %f",P_CategoryIs[1]);

    theCoordinates[0] = workspaceIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = workspace)
    P_CategoryIs[2] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = workspace) = %f",P_CategoryIs[2]);

    theCoordinates[0] = unknownIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = unknown)
    P_CategoryIs[3] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = unknown) = %f",P_CategoryIs[3]);

    ROS_INFO("[Interaction_Recognition] User was presenting %s, category %d"
                    ,categoryData.c_str(), ObjCategory[categoryData]);

    /// UPDATING STATISTICS
    int CategoryWin=-1;
    double P_CategoryWin=-1;
    for (int i=0; i<4; i++) {
        if (P_CategoryWin < P_CategoryIs[i]) {
            P_CategoryWin = P_CategoryIs[i];
            CategoryWin=i;
        }
    }

    // Difference
    P_CategoryIs[0] = P_CategoryWin-P_CategoryIs[0];
    P_CategoryIs[1] = P_CategoryWin-P_CategoryIs[1];
    P_CategoryIs[2] = P_CategoryWin-P_CategoryIs[2];
    P_CategoryIs[3] = P_CategoryWin-P_CategoryIs[3];

    // Checking results category
    int equalCategory=0;
    double MAX_DIFF = 0.1;
    for (int i=0; i<4; i++) {
        if (P_CategoryIs[i]<MAX_DIFF) {
            equalCategory++;
        }
    }

    // Updating stats
    if (equalCategory==1 and ObjCategory[categoryData] == CategoryWin) {
        Stats_Results[0]++;     // Good Job!
    }
    else if (equalCategory==1 and ObjCategory[categoryData] != CategoryWin) {
        if (ObjCategory[categoryData] == 3) {
            Stats_Results[5]++;     // Nice, Unknown category classified!
        }
        else {
            Stats_Results[1]++;     // Epic Failure man!
        }
    }
    else if (equalCategory==2) {    // 2 are similar
        if ( P_CategoryIs[ObjCategory[categoryData]] < MAX_DIFF or 
             ObjCategory[categoryData] == 3) {
            Stats_Results[2]++;     // Category is among them, or is Unknown
        }
        else {
            Stats_Results[6]++;     // Category NOT among them, FAIL
        }
    }
    else if (equalCategory==3) {    // 3 are similar
        if ( P_CategoryIs[ObjCategory[categoryData]] < MAX_DIFF or 
             ObjCategory[categoryData] == 3) {
            Stats_Results[3]++;     // Category is among them, or is Unknown
        }
        else {
            Stats_Results[7]++;     // Category NOT among them, FAIL
        }
    }
    else if (equalCategory==4) {    // all are similar
        Stats_Results[4]++;
    }
    else {
        Stats_Results[8]++;         //WTF??
    }

    // Clear the evidence in nodes
    theNet.GetNode(lastCommand)->Value()->ClearEvidence();
    theNet.GetNode(usrAnnounce)->Value()->ClearEvidence();
    theNet.GetNode(usrGesture)->Value()->ClearEvidence();
    theNet.GetNode(headingAdj)->Value()->ClearEvidence();
    theNet.GetNode(distanceAdj)->Value()->ClearEvidence();


    ROS_INFO("[Interaction_Recognition] * * * * ") ;
    
}


int InteractionRecognition::Main (const char* path) {

    // Open BN GENIE file
    if (theNet.ReadFile(path) != 0) {
        ROS_ERROR("[interaction_recognition] Cannot oppen the Bayesian Network");
        ROS_ERROR("[interaction_recognition] BN=\"%s\"",path);
        return 1;
    }
    else {
        ROS_INFO("[interaction_recognition] Bayesian Network opened successfully");
        ROS_INFO("[interaction_recognition] BN=\"%s\"",path);
    }

    // Filling dicctionaries
    write_category_dictionary();
    write_gesture_dictionary();
    write_command_dictionary();

    // Wait for callbacks
    ros::spin();

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "interaction_recognition");
    if (argc != 2) {
        ROS_ERROR("[interaction_recognition] usage: interaction_recognition BAYESIAN_NET");
        exit(1);
    }

    InteractionRecognition foo;
    return foo.Main(argv[1]);

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
void InteractionRecognition::write_command_dictionary()
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
void InteractionRecognition::write_gesture_dictionary()
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
void InteractionRecognition::write_category_dictionary()
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
