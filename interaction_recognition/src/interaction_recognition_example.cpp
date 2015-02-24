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


/**
 * SMILE Tutorial functions.
 */
void CreateNetwork(void) {
    DSL_network theNet;
    
    // create node "Success"
    int success = theNet.AddNode(DSL_CPT,"Success");
    
    // setting number (and name) of outcomes
    DSL_idArray someNames;
    someNames.Add("Success");
    someNames.Add("Failure");
    theNet.GetNode(success)->Definition()->SetNumberOfOutcomes(someNames);  
    
    // create node "Forecast"
    int forecast = theNet.AddNode(DSL_CPT,"Forecast");
    
    // setting number (and name) of outcomes
    someNames.Flush();
    someNames.Add("Good");
    someNames.Add("Moderate");
    someNames.Add("Poor");
    theNet.GetNode(forecast)->Definition()->SetNumberOfOutcomes(someNames);  
    
    // add arc from "Success" to "Forecast"
    theNet.AddArc(success,forecast);
    
    // now fill in the conditional distribution for node "Success" using
    // direct method. The probabilities are:
    // P("Success" = Success) = 0.2
    // P("Success" = Failure) = 0.8 
    DSL_doubleArray theProbs;
    theProbs.SetSize(2); // it has to be an array
    theProbs[0] = 0.2;
    theProbs[1] = 0.8;
    theNet.GetNode(success)->Definition()->SetDefinition(theProbs);
    
    // now fill in the conditional distribution for node "Forecast" using a system of 
    // coordinates. The probabilities are:
    // P("Forecast" = Good | "Success" = Success) = 0.4
    // P("Forecast" = Moderate | "Success" = Success) = 0.4
    // P("Forecast" = Poor | "Success" = Success) = 0.2
    // P("Forecast" = Good | "Success" = Failure) = 0.1
    // P("Forecast" = Moderate | "Success" = Failure) = 0.3
    // P("Forecast" = Poor | "Success" = Failure) = 0.6
    DSL_sysCoordinates theCoordinates (*theNet.GetNode(forecast)->Definition());
    theCoordinates.UncheckedValue() = 0.4;
    theCoordinates.Next();
    theCoordinates.UncheckedValue() = 0.4;
    theCoordinates.Next();
    theCoordinates.UncheckedValue() = 0.2;
    theCoordinates.Next();
    theCoordinates.UncheckedValue() = 0.1;
    theCoordinates.Next();
    theCoordinates.UncheckedValue() = 0.3;
    theCoordinates.Next();
    theCoordinates.UncheckedValue() = 0.6; 
    theNet.WriteFile("tutorial.dsl");
};



void InfereceWithBayesNet(void) {
    DSL_network theNet;
    theNet.ReadFile("tutorial.dsl");
      
    // use clustering algorithm
    theNet.SetDefaultBNAlgorithm(DSL_ALG_BN_LAURITZEN);
    
    
    /* say that we want to compute P("Forecast" = Moderate) */
    // update the network
    theNet.UpdateBeliefs();
    
    // get the handle of node "Forecast"
    int forecast = theNet.FindNode("Forecast");
    
    // get the result value
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(forecast)->Value());
    DSL_idArray *theNames;
    theNames = theNet.GetNode(forecast)->Definition()->GetOutcomesNames();  
    int moderateIndex = theNames->FindPosition("Moderate"); // should be 1
    theCoordinates[0] = moderateIndex;
    theCoordinates.GoToCurrentPosition();
    
    // get P("Forecast" = Moderate)
    double P_ForecastIsModerate = theCoordinates.UncheckedValue();
    printf("P(\"Forecast\" = Moderate) = %f\n",P_ForecastIsModerate);
    
    
    /* now we want to compute P("Success" = Failure | "Forecast" = Good) */
    // first, introduce the evidence
    // 0 is the index of state [Good]
    theNet.GetNode(forecast)->Value()->SetEvidence(0);
    
    // update the network again
    theNet.UpdateBeliefs();
    
    // get the handle of node "Success"
    int success = theNet.FindNode("Success");
    
    // get the result value
    theCoordinates.LinkTo(*theNet.GetNode(success)->Value());
    theCoordinates[0] = 1; // 1 is the index of state [Failure]
    theCoordinates.GoToCurrentPosition();
    double P_SuccIsFailGivenForeIsGood = theCoordinates.UncheckedValue();
    printf("P(\"Success\" = Failure | \"Forecast\" = Good) = ");
    printf("%f\n",P_SuccIsFailGivenForeIsGood);
    
    
    /* now we want to compute P("Success" = Success | "Forecast" = Poor) */
    // first, clear the evidence in node "Forecast"
    theNet.GetNode(forecast)->Value()->ClearEvidence();
    
    // introduce the evidence in node "Success"
    // 2 is the index of state [Poor]
    theNet.GetNode(forecast)->Value()->SetEvidence(2);
    
    // update the network again
    theNet.UpdateBeliefs();
    
    // get the result value
    theCoordinates.LinkTo(*theNet.GetNode(success)->Value());
    theCoordinates[0] = 0; // 0 is the index of state [Success]
    theCoordinates.GoToCurrentPosition();
    double P_SuccIsSuccGivenForeIsPoor = theCoordinates.UncheckedValue();
    printf("P(\"Success\" = Success | \"Forecast\" = Poor) = ");
    printf("%f\n", P_SuccIsSuccGivenForeIsPoor);
};

void UpgradeToInfluenceDiagram(void) {
    DSL_network theNet;
    theNet.ReadFile("tutorial.dsl");
    
    // create decision node "Invest"
    int invest = theNet.AddNode(DSL_LIST,"Invest");
    
    // setting number (and name) of choices
    DSL_stringArray someNames;
    someNames.Add("Invest");
    someNames.Add("DoNotInvest");
    theNet.GetNode(invest)->Definition()->SetNumberOfOutcomes(someNames);
    
    // create value node "Gain"
    int gain = theNet.AddNode(DSL_TABLE,"Gain");
    
    // add arc from "Invest" to "Gain"
    theNet.AddArc(invest,gain);
    
    // add arc from "Success" to "Gain"
    int success = theNet.FindNode("Success");
    theNet.AddArc(success,gain);
    
    // now fill in the utilities for the node "Gain"
    // The utilities are:
    // U("Invest" = Invest, "Success" = Success) = 10000
    // U("Invest" = Invest, "Success" = Failure) = -5000
    // U("Invest" = DoNotInvest, "Success" = Success) = 500
    // U("Invest" = DoNotInvest, "Success" = Failure) = 500
    // get the internal matrix of the definition of node "Gain"
    DSL_Dmatrix *theMatrix;
    theNet.GetNode(gain)->Definition()->GetDefinition(&theMatrix);
    
    // and set the values directly
    theMatrix->Subscript(0) = 10000;
    theMatrix->Subscript(1) = -5000;
    theMatrix->Subscript(2) = 500;
    theMatrix->Subscript(3) = 500;
    theNet.WriteFile("tutorial.dsl");
};

  void InferenceWithInfluenceDiagram(void) {
    DSL_network theNet;
    theNet.ReadFile("tutorial.dsl");
    
    // update the network
    theNet.UpdateBeliefs();
    
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
  };

  void ComputeValueOfInformation(void) {
    DSL_network theNet;
    theNet.ReadFile("tutorial.dsl");
    DSL_valueOfInformation theValue(&theNet);
    
    // get the handle of nodes "Forecast" and "Invest"
    int forecast = theNet.FindNode("Forecast");
    int invest = theNet.FindNode("Invest");
   theValue.AddNode(forecast);
    theValue.SetDecision(invest);
    theNet.ValueOfInformation(theValue);
    DSL_Dmatrix &theResult = theValue.GetValues();
   double EVIForecast = theResult[0];
    printf("Expected Value of Information(\"Forecast\") = %f\n",EVIForecast);
    theNet.WriteFile("tutorial.dsl");
  };

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("annotation", 10, chatterCallback);

  
    CreateNetwork();  
    InfereceWithBayesNet();  
    UpgradeToInfluenceDiagram();  
    InferenceWithInfluenceDiagram();  
    ComputeValueOfInformation();  
    return(DSL_OKAY);



    ros::spin();

    return 0;
}
