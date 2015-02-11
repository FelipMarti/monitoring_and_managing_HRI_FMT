#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tinyxml.h>

#include <sstream>

/**
 *  Node to parse an XML file where all the information of the HRI is stored
 *  This node also published the data information 
 */


struct Annotation {
    double initTime;
    double endTime;
    std::string text;
};

struct AnnotationList {
    std::string id;
    std::vector <Annotation> list;
};

void data_parser(const char *path, std::vector <AnnotationList> &dataParsed ) {
    /// Reading XML file to obtain data
    ROS_INFO("[interaction_monitor]: Parsing XML file");

    TiXmlDocument xml_doc(path);
    if (!xml_doc.LoadFile()) {
        ROS_ERROR("[interaction_monitor]: ERROR when loading XML file");
        exit(1);
    }   
    TiXmlElement* root = xml_doc.FirstChildElement();
    if (root == NULL) {
        ROS_ERROR("[interaction_monitor]: Failed to load file: No root element");
        exit(1);
    }    
    
    /// Extracting data Annotations from XML file
    dataParsed.resize(0);
    int numListAnnotation=0; 
    //ROOT
    for(TiXmlElement* elem = root->FirstChildElement();
        elem !=0; elem = elem->NextSiblingElement()) {

        //TIERS
        std::string elemName = elem->Value();
        if (elemName == "TIER") {
            AnnotationList tmpAnnList;
            tmpAnnList.id=elem->Attribute("TIER_ID");
            tmpAnnList.list.resize(0);
            dataParsed.push_back(tmpAnnList);

            //ANNOTATION
            for(TiXmlElement* child = elem->FirstChildElement();
                child !=0; child = child->NextSiblingElement()) {
                
                //ALIGNABLE_ANNOTATION
                TiXmlElement* subchild = child->FirstChildElement();
                if (subchild) {

                    //ANNOTATION_VALUE, here we have the useful data 
                    TiXmlElement* subsubchild = subchild->FirstChildElement();
                    if (subsubchild) {
                        Annotation tmpAnn;
                        // To aviod core dump problems :s
                        // when the string is empty!
                        if (subsubchild->GetText()>0) {
                            tmpAnn.text=subsubchild->GetText();
                        }
                        else {
                            tmpAnn.text="";
                        }
                        //Time extraction
                        const char* timeIni = subchild->Attribute("TIME_SLOT_REF1");
                        const char* timeEnd = subchild->Attribute("TIME_SLOT_REF2");
                        std::cout<<timeIni<<std::endl;
                        std::cout<<timeEnd<<std::endl;
                        //TODO
                        //TODO OBTAIN TIME!!!!!!
                        //TODO
                        tmpAnn.initTime=0;
                        tmpAnn.endTime=0;
                        //Filling data
                        dataParsed[numListAnnotation].list.push_back(tmpAnn); 
                        
                    } 

                }

            }
            //Change of kind of annotation
            numListAnnotation++; 
        }
    }

    ROS_INFO("[interaction_monitor]: XML file Parsed!");
    
}






int main(int argc, char **argv)
{

    ros::init(argc, argv, "parser");
    if (argc != 2) {
        ROS_ERROR("[interaction_monitor]: Input parameters error");
        ROS_ERROR("[interaction_monitor]: PATH to the XML data HRI file is needed");
        ROS_ERROR("[interaction_monitor]: Try to launch this node with its launch file");
        exit(1);
    }

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);
    
    // Before publishing we have to parse the XML file to read the data
    std::vector <AnnotationList> dataParsed;
    data_parser(argv[1], dataParsed);
///////////////DEBUG////////////////////
    std::cout<<"TOTAL LIST: "<<dataParsed.size()<<std::endl;
for(int i=0;i<dataParsed.size();i++) {
    std::cout<<dataParsed[i].id<<std::endl;
    std::cout<<dataParsed[i].list.size()<<std::endl;
    
}
    
/////////////ENDDEBUG///////////////////
    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;
    
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
    
 //       ROS_INFO("%s", msg.data.c_str());
    
        chatter_pub.publish(msg);
    
        ros::spinOnce();
    
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
