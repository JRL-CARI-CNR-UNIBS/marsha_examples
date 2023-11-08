#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <subscription_notifier/subscription_notifier.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosdyn_core/primitives.h>

int main (int argc, char** argv)
{
   ros::init (argc, argv, "pose_transform");
   ros::NodeHandle nh;

   ros::Publisher array_pub=nh.advertise<geometry_msgs::PoseArray> ("poses",1);

   std::vector<std::string> tof_cameras;
   if (!nh.getParam("tof_cameras",tof_cameras))
   {
     ROS_ERROR("No centroids speficied");
     return -1;
   }

   std::string reference_frame;
   if (!nh.getParam("reference_frame",reference_frame))
   {
     ROS_ERROR("No reference_frame speficied");
     throw std::invalid_argument("No reference_frame speficied");
   }

   tf::TransformListener listener;
   std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray>>> poses_recs;

   for (const std::string& centroid: tof_cameras)
   {
     std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray>> ptr(new ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray>(nh,centroid+"/poses",100));
     poses_recs.push_back(ptr);
   }

   ros::Rate lp(60.0);
   while(ros::ok())
   {
     ros::spinOnce();

     geometry_msgs::PoseArray poses;
     bool data_available = false;

     for(std::size_t idx=0;idx<tof_cameras.size();idx++)
     {
       if(not poses_recs.at(idx)->isANewDataAvailable())
         continue;
       else
         data_available = true;

       geometry_msgs::PoseArray cam_array=poses_recs.at(idx)->getData();

       Eigen::Affine3d T_base_camera;
       T_base_camera.setIdentity();
       tf::StampedTransform tf_base_camera;

       bool error=false;
       if(cam_array.header.frame_id.compare(reference_frame))
       {
         if(not listener.waitForTransform(reference_frame.c_str(),cam_array.header.frame_id,cam_array.header.stamp,ros::Duration(0.01)))
         {
           error=true;
         }
         else
         {
           listener.lookupTransform(reference_frame,cam_array.header.frame_id,cam_array.header.stamp,tf_base_camera);
           tf::poseTFToEigen(tf_base_camera,T_base_camera);
         }
       }
       else
       {
         tf::poseEigenToTF(T_base_camera,tf_base_camera);
       }


       for(const geometry_msgs::Pose& p: cam_array.poses)
       {
         Eigen::Vector3d point_in_c;
         point_in_c(0)=p.position.x;
         point_in_c(1)=p.position.y;
         point_in_c(2)=p.position.z;

         Eigen::Vector3d point_in_r=T_base_camera*point_in_c;

         geometry_msgs::Pose p_in_r;
         p_in_r.position.x=point_in_r(0);
         p_in_r.position.y=point_in_r(1);
         p_in_r.position.z=point_in_r(2);

         poses.poses.push_back(p_in_r);
       }
     }

     if(data_available) // if data_available == false you would publish empty poses -> tf continuously appear and disappear!
     {
       poses.header.stamp=ros::Time::now();
       poses.header.frame_id=reference_frame;

       array_pub.publish(poses);
     }

     lp.sleep();
   }

  return 0;
}
