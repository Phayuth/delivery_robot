#ifndef PUBLISHER_SUBSCRIBER_H 
#define PUBLISHER_SUBSCRIBER_H  
#include <ros/ros.h> 
#include <string>

template<typename Publish_Topic_1, typename Publish_Topic_2, typename Subscribe_Topic> class Publisher2Subscriber1
{ 
public: 	
	Publisher2Subscriber1(){};
	Publisher2Subscriber1(std::string publishTopicName_1,std::string publishTopicName_2,std::string subscribeTopicName, int queueSize)
	{
		publisher1Object = nH.advertise<Publish_Topic_1>(publishTopicName_1,queueSize);
		publisher2Object = nH.advertise<Publish_Topic_2>(publishTopicName_2,queueSize);
		subscriberObject = nH.subscribe<Subscribe_Topic>(subscribeTopicName,queueSize,&Publisher2Subscriber1::subscriberCallback,this);
	};

	void subscriberCallback(const typename Subscribe_Topic::ConstPtr& recievedMsg);
protected: 	
	ros::Subscriber subscriberObject;
	ros::Publisher publisher1Object;
	ros::Publisher publisher2Object;
	ros::NodeHandle nH;
};  
#endif