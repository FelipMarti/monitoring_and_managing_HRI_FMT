#include "ros/ros.h"
#include "std_msgs/String.h"
#include "smile.h"

/**
 * This tutorial tries to compile and link SMILE examples in ROS.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "interaction_recognition");
    if (argc != 2) {
        ROS_ERROR("[interaction_recognition] usage: interaction_recognition BAYESIAN_NET");
        return 1;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("annotation", 10, chatterCallback);



    // Create the network
    DSL_network theNet;
    // Open network GENIE file
    if (theNet.ReadFile(argv[1]) != 0) {
        ROS_ERROR("[interaction_recognition] Cannot oppen the Bayesian Network");
        ROS_ERROR("[interaction_recognition] BN=\"%s\"",argv[1]);
        return 1;
    }
    else {
        ROS_INFO("[interaction_recognition] Bayesian Network opened successfully");
        ROS_INFO("[interaction_recognition] BN=\"%s\"",argv[1]);
    } 
    // Update belives
    theNet.UpdateBeliefs();
    ROS_INFO("[interaction_recognition] Updating Beliefs");


    //TESTING stuff
    // get the resulting value
    int gain = theNet.FindNode("Gain");
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(gain)->Value());
    printf("\nThese are the expected utilities:\n\n");
    DSL_idArray *theOutcomesNames;
    DSL_intArray theIndexingParents;
    DSL_nodeDefinition *theDefinition;
    char *hisOutcomeName;
    const char *hisName;
    double ExpectedUtility;
    int aParent;
    int hisOutcome;
    int x;
    int result = DSL_OKAY;
    theIndexingParents = theNet.GetNode(gain)->Value()->GetIndexingParents();
    theCoordinates.GoFirst(); // goes to (0,0)
    
    /* print the expected utility of each choice */
    while (result != DSL_OUT_OF_RANGE) {
      printf("Policy:\n");
      for (x=0; x<theIndexingParents.NumItems(); x++) {
        aParent = theIndexingParents[x];
        theDefinition = theNet.GetNode(aParent)->Definition();
        theOutcomesNames = theDefinition->GetOutcomesNames();
    
        // get the name of the current outcome for this node
        // the current outcome is in the coordinates
        hisOutcome = theCoordinates[x];
        hisOutcomeName = (*theOutcomesNames)[hisOutcome];
        hisName = theNet.GetNode(aParent)->Info().Header().GetId();
        printf(" Node \"%s\" = %s\n",hisName,hisOutcomeName);
      };
      ExpectedUtility = theCoordinates.UncheckedValue();
      printf(" Expected Utility = %f\n\n",ExpectedUtility);
      result = theCoordinates.Next();
    };

  

    ros::spin();

    return 0;
}
