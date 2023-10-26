#include <ros/ros.h>
#include <std_msgs/String.h>

class MyNode
{
public:
    MyNode() : nh_()
    {
        // Inizializza il publisher
        pub_ = nh_.advertise<std_msgs::String>("my_topic", 10);

        // Inizializza il subscriber
        sub_ = nh_.subscribe("my_topic", 10, &MyNode::callback, this);
    }

    void publishMessage()
    {
        std_msgs::String msg;
        msg.data = "Hello, ROS!";
        pub_.publish(msg);
    }

    void callback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Received: %s", msg->data.c_str());
    }

    void run()
    {
        ros::Rate loop_rate(1); // Frequenza di pubblicazione (1 Hz)

        while (ros::ok())
        {
            publishMessage();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    MyNode my_node;
    my_node.run();

    return 0;
}



/*
#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_transform_listener_node");
    ros::NodeHandle node;

    tf::TransformListener listener;

    // Il frame di destinazione
    std::string target_frame = "openni_rgb_optical_frame";

    // Il frame di riferimento
    std::string source_frame = "world";

    // Frequenza con cui si cerca la trasformazione (in Hz)
    double rate = 10.0;
    ros::Rate loop_rate(rate);

		tf::StampedTransform transform;
		
    while (ros::ok()) {
        
        try {
            // Cerca la trasformazione tra i due frame
            listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

            // Stampa la trasformazione
            ROS_INFO("Trasformazione tra %s e %s:", target_frame.c_str(), source_frame.c_str());
            ROS_INFO("Traslazione (x, y, z): (%f, %f, %f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            ROS_INFO("Rotazione (quaternion): (%f, %f, %f, %f)", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Errore nella ricerca della trasformazione: %s", ex.what());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
		
    return 0;
}

*/

/*
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main (int argc, char** argv)
{

	ros::init(argc, argv, "listener");
	ros::NodeHandle node;
	
	tf::TransformBroadcaster br;
	tf::Transform transform;
		

	ros::spin();
	
	return 0;
}

*/

//QUESTO NON FUNZIONA POICHE' TF NON HA HEADER A QUANTO PARE


/*
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tfMessage.h>
#include "sensor_msgs/PointCloud2.h"
using namespace message_filters;

void callback(const tf::tfMessage::ConstPtr& tf, const sensor_msgs::PointCloud2::ConstPtr& point_cloud)
{
  ROS_INFO("Messaggi Ricevuti");
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "listener");
	
	ros::NodeHandle nh;
	message_filters::Subscriber<tf::tfMessage> tf_sub(nh, "tf", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "camera/rgb/points", 1);
	
	typedef sync_policies::ApproximateTime<tf::tfMessage, sensor_msgs::PointCloud2> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), tf_sub, pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();	
	
	return 0;
}
*/



/*
		+++QUESTO SOTTO FUNZIONA+++	
*/

/*

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
pcl::PointCloud<pcl::PointXYZ> cloud;


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(*msg, pcl_cloud);
	cloud += pcl_cloud;
	pcl::io::savePCDFileASCII("/home/output_point_cloud.pcd", cloud);
	
	ROS_INFO("Ricevuto messaggio di tipo PointCloud");
}

void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
	for (const auto& transform : msg->transforms)
	{
		ROS_INFO("Frame ID: %s, Child Frame ID: %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    ROS_INFO("Translation (x, y, z): %.2f, %.2f, %.2f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    ROS_INFO("Rotation (x, y, z, w): %.2f, %.2f, %.2f, %.2f", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
	}
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.

  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.

  
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
		//ros::Subscriber sub = n.subscribe("tf", 1000,  tfCallback);
		ros::Subscriber sub2 = n.subscribe("orb_slam3/tracked_points", 1000, pointCloudCallback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.

  ros::spin();

  return 0;
} */
