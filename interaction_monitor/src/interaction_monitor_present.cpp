#include "ros/ros.h"
#include "data_parser/Parse.h"
#include "data_parser/DataParsed.h"


/**
 *  This node calls the parser rosservice, once for each input file 
 *  and publishes all the Annotations that appears just before an 
 *  object/region/ws user presentation with no more of 10 seconds of difference
 */

int main(int argc, char **argv)
{

    /// Checking entrance
    ros::init(argc, argv, "interaction_monitor");
    if (argc < 3) {
        ROS_ERROR("[interaction_monitor] usage: interaction_monitor PATH_FILE(s) FILE_1 FILE_2 .. FILE_N");
        return 1;
    }

    /// Init basic stuff
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    ros::ServiceClient client = n.serviceClient<data_parser::Parse>("parse_data");
    data_parser::Parse srv;

    /// Calling ROS service to parse files
    data_parser::DataParsed dataXmlFile[argc-2];
    for (int i=0;i<argc-2;++i) {
        srv.request.path = (std::string)argv[1] + (std::string)argv[i+2];
        if (client.call(srv)) {
            ROS_INFO("[interaction_monitor] Calling parse_data_srv");
        }
        else {
            ROS_ERROR("[interaction_monitor] Failed to call service parse_data_srv");
            return 1;
        }
        dataXmlFile[i]=srv.response.data;
    }

    /// Find "usr_present" annotations 
    data_parser::AnnotationList usrPresentAnnotations[argc-2];
    for (int i=0;i<argc-2;++i) {
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
    // Waiting one second to get ready the publisher 
    ROS_INFO("[interaction_monitor] Waiting 0.5s to get ready the publisher");
    ros::Duration(0.5).sleep();
    ROS_INFO("[interaction_monitor] Start Publishing!"); 

    /// Fill msg taking into account usr_present annotations in all the files
    for (int i=0; i<argc-2; i++) {
        while (ros::ok() and usrPresentAnnotations[i].list.size()>0) {
    
            // Topic message
            data_parser::DataParsed pub_msg;
            pub_msg.data.resize(dataXmlFile[i].data.size());
    
            /// Filling msg, with all different kind of annotations
            for (int j=0;j<dataXmlFile[i].data.size();j++) {
                  
                int k=0;
                const int tenSeconds=10000;
                bool noMoar=false;
                /// Obtaining the last annotation in 10s before presentation
                while (dataXmlFile[i].data[j].list.size() > 0 and !noMoar) {
                    if (usrPresentAnnotations[i].list[0].tini-tenSeconds < 
                      dataXmlFile[i].data[j].list[k].tend and 
                      usrPresentAnnotations[i].list[0].tend >
                      dataXmlFile[i].data[j].list[k].tini ) {

                        // INSERT!!!
                        // Temp vars to Fill pub_msg
                        data_parser::Annotation tmpAnnotation;
                        tmpAnnotation.text=dataXmlFile[i].data[j].list[0].text;
                        tmpAnnotation.tini=dataXmlFile[i].data[j].list[0].tini;
                        tmpAnnotation.tend=dataXmlFile[i].data[j].list[0].tend;
    
                        data_parser::AnnotationList tmpAnnotationList;
                        tmpAnnotationList.list.resize(1);
                        tmpAnnotationList.list[0]=tmpAnnotation;
                        tmpAnnotationList.id=dataXmlFile[i].data[j].id;
    
                        // Fill pub_msg
                        pub_msg.data[j]=tmpAnnotationList;
    
                        // Delete 1st element
                        dataXmlFile[i].data[j].list.erase(dataXmlFile[i].data[j].list.begin());
    
                    }
                    // if the element is farther than 10s of the presentation
                    // just delete it and continue searching
                    else if (usrPresentAnnotations[i].list[0].tini-tenSeconds >
                        dataXmlFile[i].data[j].list[k].tini) {
                        // Delete 1st element
                        dataXmlFile[i].data[j].list.erase(dataXmlFile[i].data[j].list.begin());
                    } 
                    // No more elements in the list, so skip the bucle 
                    else if (usrPresentAnnotations[i].list[0].tend <
                        dataXmlFile[i].data[j].list[k].tini) {
                        noMoar=true;
                    }
    
                }
    
            }
    
            /// Publishing message
            annotation_pub.publish(pub_msg);
           
            usrPresentAnnotations[i].list.erase(usrPresentAnnotations[i].list.begin());
    
            ros::spinOnce();
            loop_rate.sleep();
    
        }

    }
    return 0;

}

