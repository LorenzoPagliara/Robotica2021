#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv){

 
  ros::init(argc, argv, "tf2_listener");
  ros::NodeHandle nh;

  /**
  * Memorizza i frame noti.
  */
  tf2_ros::Buffer tfBuffer;

  /*
  * Questa classe fornisce un modo semplice per richiedere e ricevere le informazioni di trasformazione
  * tra frame.
  */
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::string source = "flange";
  
  ros::Rate rate(1);

  while (nh.ok()){
    /**
    * Esprime una trasformazione tra le coordinate del frame header.frame_id e le coordinate del frame child_frame_id.
    * La trasformazione è contenuta nell'oggetto Transform.
    */
    geometry_msgs::TransformStamped ts;

    /**
    * Un tipo di dato equivalente a geometry_msgs/TransformStamped.
    */
    tf2::Stamped<tf2::Transform> stampedTransform;
    
    const char *target[7] = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6"};

    for(int i = 0; i < 7; i++) {
      try{
        /*
        * Si ottiene la trasformazione tra i frame source e target.
        */ 
        ts = tfBuffer.lookupTransform(target[i], source, ros::Time(0), ros::Duration(3.0));

        /**
         * Conversione di ts in tf2 Transform object.
        */
        tf2::fromMsg(ts, stampedTransform);
      }
      catch (tf2::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      /**
      * Restituisce un vettore traslazione.
      */
      tf2::Vector3 trans = stampedTransform.getOrigin();

      /**
      * Restituisce un quaternione rappresentante la rotazione.
      */
      tf2::Quaternion quat = stampedTransform.getRotation();

      /**
      * La classe Matrix3x3 implementa una matrice di rotazione 3x3.
      */ 
      tf2::Matrix3x3 rot;

      /**
      * Si ottiene la matrice dal quaternione. 
      */
      rot.setRotation(quat);
      
      double roll, pitch, yaw;

      /**
       * Si ottiene la matrice rappresentata come roll di pitch e yaw sugli assi fissi XYZ.
      */
      rot.getRPY(roll, pitch, yaw);

      /**
       *  Restituisce un angolo [0, 2Pi] di rotazione rappresentato dal quaternione.
      */
      double angle = quat.getAngle();

      /**
       * Restituisce l'asse di rotazione rappresentato dal quaternione. 
      */
      tf2::Vector3 axis = quat.getAxis();

      std::cout << target[i] << " - " << source << std::endl;
      std::cout << "- Translation: [" << roundf(trans.getX()*100)/100 << "," << roundf(trans.getY()*100)/100 << "," << roundf(trans.getZ()*100)/100 << "]" << std::endl;
      std::cout << "- Rotation: in Rotation Matrix: ";
  
      for(int i = 0; i < 3; i++){
        std::cout << std::endl << "                                ";
        for(int j = 0; j < 3; j++){
            std::cout << roundf(rot[i][j]*100)/100 << " ";
        }      
      } 
      std::cout << std::endl;    
      std::cout << "            in Euler RPY: [" << roll << "," << pitch << "," << yaw << "]" << std::endl;
      std::cout << "            in Axis-Angle: [" << axis.getX() << "," << axis.getY() << "," << axis.getZ() << "," << angle << "]" << std::endl;
      std::cout << std::endl << std::endl; 
    }
    rate.sleep();
    
  }
  return 0;
};