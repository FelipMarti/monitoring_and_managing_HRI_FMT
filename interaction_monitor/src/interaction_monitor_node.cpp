#include "ros/ros.h"
#include "interaction_monitor/Annotation.h"
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


/**
 *  Function to parse the action time, init time and ending time. 
 */
void parse_time(TiXmlElement* root, std::string timeIni, std::string timeEnd, 
                double &tIni, double &tEnd) 
{
    //ROOT
    for(TiXmlElement* elem = root->FirstChildElement();
        elem !=0; elem = elem->NextSiblingElement()) {
        
        //TIME_ORDER
        std::string elemName = elem->Value();
        if (elemName == "TIME_ORDER") {
            //Searching TIME_SLOT_ID
            bool exit=false;
            TiXmlElement* child = elem->FirstChildElement();
            while (child!=0 or !exit) {
                
                //Comparing TIME_VALUE attribute to obtain time in ms
                std::string elemName = child->Attribute("TIME_SLOT_ID"); 
                if (timeIni==elemName) {
                    tIni=atof(child->Attribute("TIME_VALUE"));
                }
                if (timeEnd==elemName) {
                    tEnd=atof(child->Attribute("TIME_VALUE"));
                    exit=true;
                }

                child = child->NextSiblingElement();

            }
        }
   } 
 
}


/**
 *  Function to parse all the data info. 
 */
void data_parser(const char *path, std::vector <AnnotationList> &dataParsed ) 
{
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
                        //Time slot extraction
                        const char* timeIni = subchild->Attribute("TIME_SLOT_REF1");
                        const char* timeEnd = subchild->Attribute("TIME_SLOT_REF2");
                        //Time value extraction
                        parse_time(root, timeIni, timeEnd, tmpAnn.initTime, tmpAnn.endTime);

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
        ROS_ERROR("[interaction_monitor]: Try to launch this node again with an XML file");
        exit(1);
    }

    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    
    // Before publishing we have to parse the XML file to read the data
    std::vector <AnnotationList> dataParsed;
    data_parser(argv[1], dataParsed);

///////////////DEBUG////////////////////
std::cout<<"MORE INFO: "<<std::endl;
for(int i=0;i<dataParsed.size();i++) {
    std::cout<<dataParsed[i].id<<std::endl;
    for (int j=0;j<dataParsed[i].list.size();j++) {
        std::cout<<dataParsed[i].list[j].text<<std::endl;
        std::cout<<dataParsed[i].list[j].initTime<<std::endl;
        std::cout<<dataParsed[i].list[j].endTime<<std::endl;
    }
}
/////////////ENDDEBUG///////////////////

    // Creating publishers
    ros::Publisher annotation_pub[dataParsed.size()];
    for (int i=0; i<dataParsed.size(); i++) { 
        annotation_pub[i]=n.advertise<interaction_monitor::Annotation>(dataParsed[i].id, 10);
    }

    /**
     * A timer counter to publish the video data 
     */
    ROS_INFO("[interaction_monitor]: Start Publishing!");
    int count_time = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        interaction_monitor::Annotation annotation_msg[dataParsed.size()];
    
        //TODO FILL MSG TOPICS
        //Taking into account time
        //and poping vectors, once endT==current_time

        // Publishing messages 
        for (int i=0; i<dataParsed.size(); i++) {
            annotation_pub[i].publish(annotation_msg[i]);
        }

        ros::spinOnce();
    
        loop_rate.sleep();
        ++count_time;
    }

    return 0;
}
