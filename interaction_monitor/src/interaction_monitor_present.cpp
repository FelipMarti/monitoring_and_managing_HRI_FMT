#include "ros/ros.h"
#include "data_parser/Parse.h"
#include "data_parser/DataParsed.h"


/**
 *  This node calls the parser rosservice and publishes all the data that appears
 *  before an object/region/ws presentation fome the user with no more of 10 seconds 
 *  of difference
 */

int main(int argc, char **argv)
{

    /// Checking entrance
    ros::init(argc, argv, "interaction_monitor");
    if (argc < 2) {
        ROS_ERROR("[interaction_monitor] usage: interaction_monitor PATH_FILE(s)");
        return 1;
    }

    /// Init basic stuff
    ros::NodeHandle n;
    ros::Rate loop_rate(2);

    ros::ServiceClient client = n.serviceClient<data_parser::Parse>("parse_data");
    data_parser::Parse srv;

    /// Calling ROS service
    data_parser::DataParsed dataXmlFile[argc-1];
    for (int i;i<argc-1;++i) {
        srv.request.path = argv[i+1];
        if (client.call(srv)) {
            ROS_INFO("[interaction_monitor] Calling parse_data_srv");
        }
        else {
            ROS_ERROR("[interaction_monitor] Failed to call service parse_data_srv");
            return 1;
        }
        dataXmlFile[i]=srv.response.data;
    }

    /// Find usr_present annotations
    data_parser::AnnotationList usrPresentAnnotations[argc-1];
    for (int i;i<argc-1;++i) {
        int iterator_i=0;
        bool trobat=false;
        while (iterator_i<dataXmlFile[i].data.size() and !trobat) {
            if (dataXmlFile[i].data[iterator_i].id=="usr_present") {
                usrPresentAnnotations[i] = dataXmlFile[i].data[iterator_i];
                trobat=true;
            }
            iterator_i++;
        }
    }

    /// Creating publisher
    ros::Publisher annotation_pub = n.advertise<data_parser::DataParsed>("annotations",10);

    /// Waiting one second to get ready the publisher 
    ROS_INFO("[interaction_monitor] Waiting 1s to get ready the publisher");
    ros::Duration(1, 0).sleep();
    ROS_INFO("[interaction_monitor] Start Publishing!"); 

    //TODO: TEST THE ABOVE SECTION!
    //TODO: iterate through all the lists usrPresentAnnotations[i].list[j]
    //TODO: Change dataXmlFile[i] and usrPresentAnnotations[i]
    //TODO: add new bulce or think about how to do it
/*
    /// Fill msg taking into account usr_present annotation
    while (ros::ok() and usrPresentAnnotations.list.size()>0) {

        // Topic message
        data_parser::DataParsed pub_msg;
        pub_msg.data.resize(dataXmlFile.data.size());

        /// Filling msg, with all different kind of annotation
        for (int j=0;j<dataXmlFile.data.size();j++) {
              
            int k=0;
            const int tenSeconds=10000;
            bool noMoar=false;
            /// Obtaining the last annotation in 10s before presentation
            while (dataXmlFile.data[j].list.size() > 0 and !noMoar) {
                if (usrPresentAnnotations.list[0].tini-tenSeconds < 
                  dataXmlFile.data[j].list[k].tini and 
                  usrPresentAnnotations.list[0].tend >
                  dataXmlFile.data[j].list[k].tini ) {
                    // INSERT!!!
                    // Temp vars to Fill pub_msg
                    data_parser::Annotation tmpAnnotation;
                    tmpAnnotation.text=dataXmlFile.data[j].list[0].text;
                    tmpAnnotation.tini=dataXmlFile.data[j].list[0].tini;
                    tmpAnnotation.tend=dataXmlFile.data[j].list[0].tend;

                    data_parser::AnnotationList tmpAnnotationList;
                    tmpAnnotationList.list.resize(1);
                    tmpAnnotationList.list[0]=tmpAnnotation;
                    tmpAnnotationList.id=dataXmlFile.data[j].id;

                    // Fill pub_msg
                    pub_msg.data[j]=tmpAnnotationList;

                    // Delete 1st element
                    dataXmlFile.data[j].list.erase(dataXmlFile.data[j].list.begin());

                }
                // if the element is farther than 10s of the presentation
                // just delete it and continue searching
                else if (usrPresentAnnotations.list[0].tini-tenSeconds >
                    dataXmlFile.data[j].list[k].tini) {
                    // Delete 1st element
                    dataXmlFile.data[j].list.erase(dataXmlFile.data[j].list.begin());
                } 
                // No more elements in the list, so skip the bucle 
                else if (usrPresentAnnotations.list[0].tend <
                    dataXmlFile.data[j].list[k].tini) {
                    noMoar=true;
                }

            }

        }

        /// Publishing message
        annotation_pub.publish(pub_msg);
       
        usrPresentAnnotations.list.erase(usrPresentAnnotations.list.begin());

        ros::spinOnce();
        loop_rate.sleep();

    }
*/
    return 0;

}

