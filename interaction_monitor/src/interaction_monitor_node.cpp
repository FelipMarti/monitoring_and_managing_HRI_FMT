#include "ros/ros.h"
#include "data_parser/Parse.h"
#include "data_parser/DataParsed.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "interaction_monitor");
    if (argc != 2) {
        ROS_ERROR("[interaction_monitor] usage: interaction_monitor PATH_FILE");
        return 1;
    }

    ros::NodeHandle n;
    ros::Rate loop_rate(1000);

    ros::ServiceClient client = n.serviceClient<data_parser::Parse>("parse_data");
    data_parser::Parse srv;

    srv.request.path = argv[1];
    if (client.call(srv)) {
        ROS_INFO("[interaction_monitor] Calling parse_data_srv");
    }
    else {
        ROS_ERROR("[interaction_monitor] Failed to call service parse_data_srv");
        return 1;
    }
    data_parser::DataParsed dataXmlFile=srv.response.data;

    /// Creating publisher
    ros::Publisher annotation_pub = n.advertise<data_parser::DataParsed>("annotations",10);

    ROS_INFO("[interaction_monitor] Start Publishing!");
    int currentTime = 0;

    /// Main Loop
    while (ros::ok()) {

        // Topic message
        data_parser::DataParsed pub_msg;
        pub_msg.data.resize(dataXmlFile.data.size());

        // Var to finish the node running in case there is no more things
        // to publish
        int checkIfNothingElseToPublish=0;

        /// Filling message topics
        for (int i=0;i<dataXmlFile.data.size();i++) {

            // Check no empty vector to avoid segmentation fault
            if (dataXmlFile.data[i].list.size() > 0) {

                // Check init time to publish data or not
                if (dataXmlFile.data[i].list[0].tini <= currentTime) {
                    //Temp vars to Fill pub_msg
                    data_parser::Annotation tmpAnnotation;
                    tmpAnnotation.text=dataXmlFile.data[i].list[0].text;
                    tmpAnnotation.tini=dataXmlFile.data[i].list[0].tini;
                    tmpAnnotation.tend=dataXmlFile.data[i].list[0].tend;

                    data_parser::AnnotationList tmpAnnotationList;
                    tmpAnnotationList.list.push_back(tmpAnnotation);
                    tmpAnnotationList.id=dataXmlFile.data[i].id;

                    //Fill pub_msg
                    pub_msg.data[i]=tmpAnnotationList;
                }
                else {
                    //Fill blank
                    data_parser::AnnotationList tmpAnnotationList;
                    tmpAnnotationList.id=dataXmlFile.data[i].id;

                    pub_msg.data[i]=tmpAnnotationList;
                }
                // Check end time to erase element
                if (dataXmlFile.data[i].list[0].tend == currentTime) {
                    dataXmlFile.data[i].list.erase(dataXmlFile.data[i].list.begin());

                }

            }
            else {
                checkIfNothingElseToPublish++;
            }
            
        }
        ROS_INFO("Time %dms",currentTime);

        /// Publishing message
        annotation_pub.publish(pub_msg);

        /// Check if we can stop running the node
        if (checkIfNothingElseToPublish == dataXmlFile.data.size()) {
            ROS_INFO("[interaction_monitor] Nothing else to publish!!!");
            ROS_INFO("[interaction_monitor] So, we stop gently the node :)");
            return 0;    
        }

        ros::spinOnce();

        loop_rate.sleep();
        ++currentTime;
    }



    return 0;
}

