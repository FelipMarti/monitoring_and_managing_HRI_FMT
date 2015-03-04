#include "ros/ros.h"
#include "data_parser/Parse.h"
#include "data_parser/DataParsed.h"


/**
 *  This node calls the parser rosservice and publishes all the data that appears
 *  before an object/region/ws presentation frmo the user with no more of 10 seconds 
 *  of difference
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interaction_monitor");
    if (argc != 2) {
        ROS_ERROR("[interaction_monitor] usage: interaction_monitor PATH_FILE");
        return 1;
    }

    ros::NodeHandle n;
    ros::Rate loop_rate(200);

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

    /// Main Loop
    while (ros::ok()) {

        /// Find usr_present annotations
        int iterator_i=0;
        bool trobat=false;
        data_parser::AnnotationList usrPresentAnnotations;
        while (iterator_i<dataXmlFile.data.size() and !trobat) {
            if (dataXmlFile.data[iterator_i].id=="usr_present") {
                usrPresentAnnotations = dataXmlFile.data[iterator_i];
                ROS_INFO("TROBAT");
                trobat=true;
            }
            iterator_i++;
        }

        /// Fill msg taking into account usr_present annotation
        for (int i=0;i<usrPresentAnnotations.list.size();i++) {
            
            // Topic message
            data_parser::DataParsed pub_msg;
            pub_msg.data.resize(dataXmlFile.data.size());

            /// Filling msg
            for (int j=0;j<dataXmlFile.data.size();j++) {
                if (dataXmlFile.data[j].id != "usr_present") {
                   
                    //TODO: for each iteration check time and delete,
                    //TODO: so iterate per each dataXmlFile.data[j].list[x]
                    //TODO: and just save one, the last one
                    int k=0;
                    while (dataXmlFile.data[j].list.size() < 0) {
                        if (dataXmlFile.data[j].list[k].tini >      
                            usrPresentAnnotations.list[0].tini-10000 and
                            usrPresentAnnotations.list[0].tend >
                            dataXmlFile.data[j].list[k].tini ) {
                            //TODO: INSERT!!!
                            //TODO: WHY 10000? 10s!!!
                            //TODO: Delete 1st element
                        }
                    }

                }
            }


            /// Publishing message
            annotation_pub.publish(pub_msg);
            
        }



        

/*
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
                if (dataXmlFile.data[i].list[0].tend <= currentTime) {
                    dataXmlFile.data[i].list.erase(dataXmlFile.data[i].list.begin());

                }

            }
            else {
                checkIfNothingElseToPublish++;
            }
            
        }
        ROS_INFO("Time %02d:%02d.%03d ms  (%dms)",
        currentTime/1000/60,currentTime/1000%60,currentTime%1000,currentTime);

        /// Publishing message
        annotation_pub.publish(pub_msg);

        /// Check if we can stop running the node
        if (checkIfNothingElseToPublish == dataXmlFile.data.size()) {
            ROS_INFO("[interaction_monitor] Nothing else to publish!!!");
            ROS_INFO("[interaction_monitor] So, the node is stopped gently :)");
            return 0;    
        }

        ros::spinOnce();

        loop_rate.sleep();
        /// Publishing data parsed in intervals of 500ms
        currentTime+=500;
*/

    }



    return 0;
}

