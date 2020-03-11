#include "agrob_map_transform/agrob_gnss_transform.h"
#include "agrob_map_transform/agrob_gnss_conversions.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcException.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <cmath>
#include "math_utils.h"
#include <string>

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"


//***********************************************************************
//
//
//
//***********************************************************************
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;
const int POSE_SIZE = 6;
const int TWIST_SIZE = 6;
const int POSITION_SIZE = 3;
const int ORIENTATION_SIZE = 3;
const int ACCELERATION_SIZE = 3;


//***********************************************************************
//
//
//
//***********************************************************************
void appendPrefix(std::string tfPrefix, std::string &frameId){
    // Strip all leading slashes for tf2 compliance
    if (!frameId.empty() && frameId.at(0) == '/')
    {
        frameId = frameId.substr(1);
    }

    if (!tfPrefix.empty() && tfPrefix.at(0) == '/')
    {
        tfPrefix = tfPrefix.substr(1);
    }

    // If we do have a tf prefix, then put a slash in between
    if (!tfPrefix.empty())
    {
        frameId = tfPrefix + "/" + frameId;
    }
}

//***********************************************************************
//
//
//
//***********************************************************************
void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw){
    tf2::Matrix3x3 orTmp(quat);
    orTmp.getRPY(roll, pitch, yaw);
}


//***********************************************************************
//
//
//
//***********************************************************************
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans,
                         const bool silent){
    bool retVal = true;

    // First try to transform the data at the requested time
    try{
        tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform,
                     targetFrameTrans);
    }catch (tf2::TransformException &ex){
        // The issue might be that the transforms that are available are not close
        // enough temporally to be used. In that case, just use the latest available
        // transform and warn the user.
        try{
            tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0)).transform,
                         targetFrameTrans);

            if (!silent){
                ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                                                " was unavailable for the time requested. Using latest instead.\n");
            }
        }catch(tf2::TransformException &ex){
            if (!silent){
                ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                                                                 " to " << targetFrame << ". Error was " << ex.what() << "\n");
            }
            retVal = false;
        }
    }

    // Transforming from a frame id to itself can fail when the tf tree isn't
    // being broadcast (e.g., for some bag files). This is the only failure that
    // would throw an exception, so check for this situation before giving up.
    if (!retVal){
        if (targetFrame == sourceFrame){
            targetFrameTrans.setIdentity();
            retVal = true;
        }
    }

    return retVal;
}

//***********************************************************************
//
//
//
//***********************************************************************
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         tf2::Transform &targetFrameTrans,
                         const bool silent){
    return lookupTransformSafe(buffer, targetFrame, sourceFrame, time, ros::Duration(0), targetFrameTrans, silent);
}


//***********************************************************************
//
//
//
//***********************************************************************
double clampRotation(double rotation){
    while (rotation > PI){
        rotation -= TAU;
    }

    while (rotation < -PI){
        rotation += TAU;
    }
    return rotation;
}


//***********************************************************************
//
//
//
//***********************************************************************
namespace RobotLocalization{



//***********************************************************************
//
//
//
//***********************************************************************

  NavSatTransform::NavSatTransform(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    broadcast_utm_transform_(false),
    broadcast_utm_transform_as_parent_frame_(false),
    gps_updated_(false),
    has_transform_gps_(false),
    has_transform_imu_(false),
    has_transform_odom_(false),
    odom_updated_(false),
    publish_gps_(false),
    transform_good_(false),
    use_manual_datum_(false),
    use_odometry_yaw_(false),
    zero_altitude_(false),
    magnetic_declination_(0.0),
    utm_odom_tf_yaw_(0.0),
    yaw_offset_(0.0),
    base_link_frame_id_("base_link"),
    gps_frame_id_(""),
    utm_zone_(""),
    world_frame_id_("odom"),
    robot_model("agrobv18"),
    transform_timeout_(ros::Duration(0)),
    tf_listener_(tf_buffer_){

    ROS_INFO("Waiting for valid clock time...");
    ros::Time::waitForValid();
    ROS_INFO("Valid clock time received. Starting node.");

    latest_utm_covariance_.resize(POSE_SIZE, POSE_SIZE);
    latest_odom_covariance_.resize(POSE_SIZE, POSE_SIZE);

    double frequency;
    double delay = 0.0;
    double transform_timeout = 0.0;
    AGROB_datum_autocorrection_stage=0;
    AGROB_global_PF_counter = 0;

    solution_ranges.resize(360,0);

    PFHisto = cv::Mat::zeros(100,360, CV_8UC3);
    PFPesos = cv::Mat::zeros(100,360, CV_8UC3);

    // Load the parameters we need
    nh_priv.getParam("magnetic_declination_radians", magnetic_declination_);
    nh_priv.param("yaw_offset", yaw_offset_, 0.0);
    nh_priv.param("broadcast_utm_transform", broadcast_utm_transform_, false);
    nh_priv.param("broadcast_utm_transform_as_parent_frame", broadcast_utm_transform_as_parent_frame_, false);
    nh_priv.param("zero_altitude", zero_altitude_, false);
    nh_priv.param("publish_filtered_gps", publish_gps_, true);
    nh_priv.param("use_odometry_yaw", use_odometry_yaw_, false);
    nh_priv.param("PF_for_datum_orientation_estimation", use_PFDATUM_, false);
    nh_priv.param("wait_for_datum", use_manual_datum_, false);
    nh_priv.param("frequency", frequency, 10.0);
    nh_priv.param("delay", delay, 0.0);
    nh_priv.param("transform_timeout", transform_timeout, 0.0);
    nh_priv.param<std::string>("robot_model", robot_model, "agrobv16");

    transform_timeout_.fromSec(transform_timeout);

    // Subscribe to the messages and services we need
    datum_srv_ = nh.advertiseService("datum", &NavSatTransform::datumCallback, this);

    pose_to_polar_srv_ = nh.advertiseService("pose_to_polar", &NavSatTransform::pose2polar_Callback, this);

    polar_to_pose_srv_ = nh.advertiseService("polar_to_pose", &NavSatTransform::polar2pose_Callback, this);



    if (use_manual_datum_ && nh_priv.hasParam("datum")) {
      XmlRpc::XmlRpcValue datum_config;

      try {
        double datum_lat;
        double datum_lon;
        double datum_yaw;

        nh_priv.getParam("datum", datum_config);

        // Handle datum specification. Users should always specify a baseLinkFrameId_ in the
        // datum config, but we had a release where it wasn't used, so we'll maintain compatibility.
        ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(datum_config.size() >= 3);

        if (datum_config.size() > 3) {
          ROS_WARN_STREAM("Deprecated datum parameter configuration detected. Only the first three parameters "
              "(latitude, longitude, yaw) will be used. frame_ids will be derived from odometry and navsat inputs.");
        }

        std::ostringstream ostr;
        ostr << std::setprecision(20) << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
        std::istringstream istr(ostr.str());
        istr >> datum_lat >> datum_lon >> datum_yaw;

        // Try to resolve tf_prefix
        std::string tf_prefix = "";
        std::string tf_prefix_path = "";
        if (nh_priv.searchParam("tf_prefix", tf_prefix_path))
        {
          nh_priv.getParam(tf_prefix_path, tf_prefix);
        }

        // Append the tf prefix in a tf2-friendly manner
        appendPrefix(tf_prefix, world_frame_id_);
        appendPrefix(tf_prefix, base_link_frame_id_);

        agrob_map_transform::SetDatum::Request request;
        request.geo_pose.position.latitude = datum_lat;
        request.geo_pose.position.longitude = datum_lon;
        request.geo_pose.position.altitude = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, datum_yaw);
        request.geo_pose.orientation = tf2::toMsg(quat);
        agrob_map_transform::SetDatum::Response response;
        datumCallback(request, response);
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << datum_config.getType() << ")");
      }
    }

    odom_sub_ = nh.subscribe("odometry/filtered", 1, &NavSatTransform::odomCallback, this);
    gps_sub_  = nh.subscribe("gps/fix", 1, &NavSatTransform::gpsFixCallback, this);

    if (!use_odometry_yaw_ && !use_manual_datum_){
      imu_sub_ = nh.subscribe("imu/data", 1, &NavSatTransform::imuCallback, this);
    }

    gps_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/agrob_map/gps_odometry", 10);
    imu_odom_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/agrob_map/imu/rpy",10);
   

    if (publish_gps_){
      filtered_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/agrob_map/gps/filtered", 10);
    }

    // Sleep for the parameterized amount of time, to give
    // other nodes time to start up (not always necessary)
    ros::Duration start_delay(delay);
    start_delay.sleep();

    periodicUpdateTimer_ = nh.createTimer(ros::Duration(1./frequency), &NavSatTransform::periodicUpdate, this);
  }


//***********************************************************************
//
//
//
//***********************************************************************
  NavSatTransform::~NavSatTransform(){
  }


/**************************************************************************************************************
  * Function Name: AGROB_datum_autocorrection()
  * Inputs: gps_odom
  * Output: Corrected Datum orientation
  * Description: Partcile filter that corrects the datum orientation according to the GPS coordinates
  **************************************************************************************************************/

void NavSatTransform::AGROB_datum_autocorrection(nav_msgs::Odometry gps_odom){
//AGROB_last_odometry_information
    if(AGROB_datum_autocorrection_stage == 0) {
        ROS_ERROR("Initialization of AGROB DATUM");
        AGROB_datum_autocorrection_stage++;
    }else{

      double x,y;
       if (robot_model == "agrobv18") {
        y = AGROB_last_odometry_information.pose.pose.position.x;
        x = AGROB_last_odometry_information.pose.pose.position.z;
      }else{
        x = AGROB_last_odometry_information.pose.pose.position.x;
        y = AGROB_last_odometry_information.pose.pose.position.y;
      }

      double distance = sqrt(  (gps_odom.pose.pose.position.x-x)*(gps_odom.pose.pose.position.x-x) + (gps_odom.pose.pose.position.y-y)*(gps_odom.pose.pose.position.y-y) );
      double center_map= sqrt( gps_odom.pose.pose.position.x*gps_odom.pose.pose.position.x +  gps_odom.pose.pose.position.y*gps_odom.pose.pose.position.y );

       
       if(AGROB_datum_autocorrection_stage == 1){
         if(center_map < 2.0){
             ROS_ERROR("We are near to DATUM location");
             if(distance < 5.0){
                ROS_ERROR("We are near to DATUM location is OK");
                AGROB_datum_autocorrection_stage=2;

             }else{
                ROS_ERROR("We are near to DATUM location is BAD...NEEDS to be corrected %d", AGROB_datum_autocorrection_stage);
                AGROB_datum_autocorrection_stage = -1;

             }

         }else{
               ROS_ERROR("We are outside of DATUM location");
               AGROB_datum_autocorrection_stage=-1;

         }
       }else if(AGROB_datum_autocorrection_stage == 2){ 
         //inicialização do filro
          ROS_ERROR("inicialização do filtro");
         for(int i=0 ; i< 360; i++){
            AGROB_PF_datum_orientation[i][0]=i;
            AGROB_PF_datum_orientation[i][1]=1.0;
         }
            AGROB_datum_autocorrection_stage=3;
       }else if(AGROB_datum_autocorrection_stage == 3){
         AGROB_global_PF_counter++;
         //ROS_ERROR("iteração do filtro");
            double dist_temp_max=0.0;
            for(int i=0 ; i< 360; i++){
              double xtemp, ytemp, dist_temp;
               xtemp = cos( AGROB_PF_datum_orientation[i][0]*M_PI/180.0 )* x - sin( AGROB_PF_datum_orientation[i][0]*M_PI/180.0 )* y;
               ytemp = sin( AGROB_PF_datum_orientation[i][0]*M_PI/180.0 )* x + cos( AGROB_PF_datum_orientation[i][0]*M_PI/180.0 )* y;
               dist_temp =  sqrt(  (gps_odom.pose.pose.position.x-xtemp)*(gps_odom.pose.pose.position.x-xtemp) + (gps_odom.pose.pose.position.y-ytemp)*(gps_odom.pose.pose.position.y-ytemp) );
               AGROB_PF_datum_orientation[i][2] = dist_temp;
               if(dist_temp_max< dist_temp) {
                 dist_temp_max=dist_temp;
               }
            }

          double peso_max=0.0;
          for(int i=0 ; i< 360; i++){

            // ROS_INFO ("CENTER MAP %.3f", center_map);
            // if (center_map > 0.5) {
            //   // AGROB_PF_datum_orientation[i][1] =  AGROB_PF_datum_orientation[i][1] * 0.5 + (1.0 - AGROB_PF_datum_orientation[i][2] / dist_temp_max) * .5;

            //   double  PF_peso=1.0;
            //   if(center_map > 5.0) PF_peso  = 5.0;
            //   AGROB_PF_datum_orientation[i][1] = ( AGROB_PF_datum_orientation[i][1]*(double)AGROB_global_PF_counter +
            //     (1.0 - AGROB_PF_datum_orientation[i][2]/ dist_temp_max)*PF_peso ) /
            //     (double)(AGROB_global_PF_counter + PF_peso);}
            AGROB_PF_datum_orientation[i][1] = ( AGROB_PF_datum_orientation[i][1]*(double)AGROB_global_PF_counter +
                 (1.0 - AGROB_PF_datum_orientation[i][2]/ dist_temp_max)*center_map ) /
                 (double)(AGROB_global_PF_counter + center_map);


              if (peso_max < AGROB_PF_datum_orientation[i][1]) {
                peso_max = AGROB_PF_datum_orientation[i][1];
              }
            
          }

          int indexT=0, num_max=0;
          for(int i=0 ; i< 360; i++){

            if( peso_max == AGROB_PF_datum_orientation[i][1] ){
              num_max++;
              indexT=i;

              solution_ranges[i]++;
              if (solution_ranges[i] >= 100) solution_ranges[i] = 99;

              if (PFHisto.at<cv::Vec3b>(cv::Point(i, 1)).val[2] == 0) {
               cv::line(PFHisto, cv::Point(i,0), cv::Point(i, solution_ranges[i]), cv::Scalar(255, 0, 0), 1, 8);
              }

              if (num_max < 5) {
                //ROS_WARN("Solution [%d º]: %d", i, solution_ranges[i]);
                cv::imshow("histo", PFHisto);
                cv::waitKey(5);
              }
            }

            cv::line(PFPesos, cv::Point(i,0), cv::Point(i,
                     (int32_t) (AGROB_PF_datum_orientation[i][1]*100.0)),
                     cv::Scalar(0, 255, 0), 1, 8);

            // ROS_INFO(" Pesos [%d]: %d %f",  i, (int32_t) (AGROB_PF_datum_orientation[i][1]*100.0), AGROB_PF_datum_orientation[i][1]*100.0);
            cv::imshow("Pesos", PFPesos);
            cv::waitKey(5);
            PFPesos.release();
            PFPesos = cv::Mat::zeros(100,360, CV_8UC3);

          }

          if(num_max==1){
            ROS_ERROR("We have 1 solution = %d ", indexT);

            int32_t i = indexT;

            cv::line(PFHisto, cv::Point(i,0), cv::Point(i, solution_ranges[i]), cv::Scalar(0, 0, 255), 1, 8);

            //ROS_WARN("Solution [%d º]: %d", i, solution_ranges[i]);
            cv::imshow("histo", PFHisto);
            cv::waitKey(5);
          } else{
            ROS_ERROR("We have %d a solutions", num_max);
          }


       }else{
          ROS_ERROR("We are near to DATUM location is BAD...NEEDS to be corrected %d", AGROB_datum_autocorrection_stage);
       }

    }
}


//***********************************************************************
//
//
//
//***********************************************************************
//  void NavSatTransform::run()
  void NavSatTransform::periodicUpdate(const ros::TimerEvent& event){
    if (!transform_good_){
      computeTransform();

      if (transform_good_ && !use_odometry_yaw_ && !use_manual_datum_){
        // Once we have the transform, we don't need the IMU
        imu_sub_.shutdown();
      }
     // ROS_ERROR ("aqui1");


    }else{
      nav_msgs::Odometry gps_odom;
     // ROS_ERROR ("aqui2");
      if (prepareGpsOdometry(gps_odom)){
        gps_odom_pub_.publish(gps_odom);
        //std::cout << "gps \n" << gps_odom << std::endl;
        if(use_PFDATUM_){
          AGROB_datum_autocorrection(gps_odom);
        }
      }

      if (publish_gps_){
        sensor_msgs::NavSatFix odom_gps;
        if (prepareFilteredGps(odom_gps)){
          filtered_gps_pub_.publish(odom_gps);
        }
      }
    }
  }


//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::computeTransform(){
    // Only do this if:
    // 1. We haven't computed the odom_frame->utm_frame transform before
    // 2. We've received the data we need
    if (!transform_good_ &&
        has_transform_odom_ &&
        has_transform_gps_ &&
        has_transform_imu_)
    {
      // The UTM pose we have is given at the location of the GPS sensor on the robot. We need to get the UTM pose of
      // the robot's origin.
      tf2::Transform transform_utm_pose_corrected;
      if (!use_manual_datum_)
      {
        getRobotOriginUtmPose(transform_utm_pose_, transform_utm_pose_corrected, ros::Time(0));
      }
      else
      {
        transform_utm_pose_corrected = transform_utm_pose_;
      }

      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf2::Matrix3x3 mat(transform_orientation_);

      // Convert to RPY
      double imu_roll;
      double imu_pitch;
      double imu_yaw;
      mat.getRPY(imu_roll, imu_pitch, imu_yaw);

      /* The IMU's heading was likely originally reported w.r.t. magnetic north.
       * However, all the nodes in agrob_map_transform assume that orientation data,
       * including that reported by IMUs, is reported in an ENU frame, with a 0 yaw
       * value being reported when facing east and increasing counter-clockwise (i.e.,
       * towards north). To make the world frame ENU aligned, where X is east
       * and Y is north, we have to take into account three additional considerations:
       *   1. The IMU may have its non-ENU frame data transformed to ENU, but there's
       *      a possibility that its data has not been corrected for magnetic
       *      declination. We need to account for this. A positive magnetic
       *      declination is counter-clockwise in an ENU frame. Therefore, if
       *      we have a magnetic declination of N radians, then when the sensor
       *      is facing a heading of N, it reports 0. Therefore, we need to add
       *      the declination angle.
       *   2. To account for any other offsets that may not be accounted for by the
       *      IMU driver or any interim processing node, we expose a yaw offset that
       *      lets users work with navsat_transform_node.
       *   3. UTM grid isn't aligned with True East\North. To account for the difference
       *      we need to add meridian convergence angle.
       */
      imu_yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magnetic_declination_ <<
                      ", user-specified offset of " << yaw_offset_ <<
                      " and meridian convergence of " << utm_meridian_convergence_ << "." <<
                      " Transform heading factor is now " << imu_yaw);

      // Convert to tf-friendly structures
      tf2::Quaternion imu_quat;
      imu_quat.setRPY(0.0, 0.0, imu_yaw);

      // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      tf2::Transform utm_pose_with_orientation;
      utm_pose_with_orientation.setOrigin(transform_utm_pose_corrected.getOrigin());
      utm_pose_with_orientation.setRotation(imu_quat);

      utm_world_transform_.mult(transform_world_pose_, utm_pose_with_orientation.inverse());

      utm_world_trans_inverse_ = utm_world_transform_.inverse();

 //     ROS_INFO_STREAM("Transform world frame pose is: " << transform_world_pose_);
//      ROS_INFO_STREAM("World frame->utm transform is " << utm_world_transform_);

      transform_good_ = true;

      // Send out the (static) UTM transform in case anyone else would like to use it.
      if (broadcast_utm_transform_)
      {
        geometry_msgs::TransformStamped utm_transform_stamped;
        utm_transform_stamped.header.stamp = ros::Time::now();
        utm_transform_stamped.header.frame_id = (broadcast_utm_transform_as_parent_frame_ ? "utm" : world_frame_id_);
        utm_transform_stamped.child_frame_id = (broadcast_utm_transform_as_parent_frame_ ? world_frame_id_ : "utm");
        utm_transform_stamped.transform = (broadcast_utm_transform_as_parent_frame_ ?
                                             tf2::toMsg(utm_world_trans_inverse_) : tf2::toMsg(utm_world_transform_));
        utm_transform_stamped.transform.translation.z = (zero_altitude_ ?
                                                           0.0 : utm_transform_stamped.transform.translation.z);
        utm_broadcaster_.sendTransform(utm_transform_stamped);
      }
    }
  }


//***********************************************************************
//
//
//
//***********************************************************************
  bool NavSatTransform::datumCallback(agrob_map_transform::SetDatum::Request& request,
                                      agrob_map_transform::SetDatum::Response&){
    // If we get a service call with a manual datum, even if we already computed the transform using the robot's
    // initial pose, then we want to assume that we are using a datum from now on, and we want other methods to
    // not attempt to transform the values we are specifying here.
    use_manual_datum_ = true;

    transform_good_ = false;

    sensor_msgs::NavSatFix *fix = new sensor_msgs::NavSatFix();
    fix->latitude = request.geo_pose.position.latitude;
    fix->longitude = request.geo_pose.position.longitude;
    fix->altitude = request.geo_pose.position.altitude;
    fix->header.stamp = ros::Time::now();
    fix->position_covariance[0] = 0.1;
    fix->position_covariance[4] = 0.1;
    fix->position_covariance[8] = 0.1;
    fix->position_covariance_type = sensor_msgs::NavSatStatus::STATUS_FIX;
    sensor_msgs::NavSatFixConstPtr fix_ptr(fix);
    setTransformGps(fix_ptr);

    nav_msgs::Odometry *odom = new nav_msgs::Odometry();
    odom->pose.pose.orientation.x = 0;
    odom->pose.pose.orientation.y = 0;
    odom->pose.pose.orientation.z = 0;
    odom->pose.pose.orientation.w = 1;
    odom->pose.pose.position.x = 0;
    odom->pose.pose.position.y = 0;
    odom->pose.pose.position.z = 0;
    odom->header.frame_id = world_frame_id_;
    odom->child_frame_id = base_link_frame_id_;
    nav_msgs::OdometryConstPtr odom_ptr(odom);
    setTransformOdometry(odom_ptr);

    sensor_msgs::Imu *imu = new sensor_msgs::Imu();
    imu->orientation = request.geo_pose.orientation;
    imu->header.frame_id = base_link_frame_id_;
    sensor_msgs::ImuConstPtr imu_ptr(imu);
    imuCallback(imu_ptr);

    return true;
  }


//***********************************************************************
//
//
//
//***********************************************************************
bool NavSatTransform::pose2polar_Callback(agrob_map_transform::GetPolars::Request& request,
                                    agrob_map_transform::GetPolars::Response& response) {

  // This service will automatically convert a local Pose to a Wgs84 pose

  // TODO(LUIS): Isto trabalha de acordo com o DATUM, para ter o efeito que eu quero, a orientação do DATUM deve ser 0!
  //             Update: Passei este problema para o lado do utilizador (também eu), que tem a responsabilidade de rodar
  //             os pontos com a orientação do datum antes de fazer a conversão

  geometry_msgs::PoseWithCovarianceStamped *pose = new geometry_msgs::PoseWithCovarianceStamped();
  *pose = request.local_pose;

  sensor_msgs::NavSatFix gps_point;
  Eigen::MatrixXd odom_covariance_;

  if (transform_good_) {

    tf2::Transform odom_as_utm;
    tf2::Transform pose_tf;

    odom_covariance_.resize(POSE_SIZE, POSE_SIZE);

    tf2::fromMsg(pose->pose.pose, pose_tf);

    odom_as_utm.mult(utm_world_trans_inverse_, pose_tf);
    odom_as_utm.setRotation(tf2::Quaternion::getIdentity());


    // Rotate the covariance as well
    tf2::Matrix3x3 rot(utm_world_trans_inverse_.getRotation());
    Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
    rot_6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd) {
      rot_6d(rInd, 0) = rot.getRow(rInd).getX();
      rot_6d(rInd, 1) = rot.getRow(rInd).getY();
      rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Rotate the covariance
    odom_covariance_ = rot_6d * odom_covariance_.eval() * rot_6d.transpose();

    // Now convert the data back to lat/long and place into the message
    NavsatConversions::UTMtoLL(odom_as_utm.getOrigin().getY(),
                               odom_as_utm.getOrigin().getX(),
                               utm_zone_,
                               gps_point.latitude,
                               gps_point.longitude);
    gps_point.altitude = odom_as_utm.getOrigin().getZ();

    // Copy the measurement's covariance matrix back
    for (size_t i = 0; i < POSITION_SIZE; i++) {
      for (size_t j = 0; j < POSITION_SIZE; j++) {
        gps_point.position_covariance[POSITION_SIZE * i + j] = odom_covariance_(i, j);
      }
    }

    gps_point.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
    gps_point.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    gps_point.header.frame_id = "gps";
    gps_point.header.stamp = odom_update_time_;

  }

  response.geo_pose = gps_point;
  return true;
}


//***********************************************************************
//
//
//
//***********************************************************************
bool NavSatTransform::polar2pose_Callback  (agrob_map_transform::GetPose::Request& request,
                                          agrob_map_transform::GetPose::Response& response) {

  // This service will automatically convert a Wgs84 pose to a local Pose
  sensor_msgs::NavSatFix *gps_point = new sensor_msgs::NavSatFix();
  *gps_point = request.geo_pose;
  nav_msgs::Odometry gps_odom;

  if (transform_good_) {
    tf2::Transform transformed_utm_gps;
    tf2::Transform utm_pose_;
    Eigen::MatrixXd utm_covariance_;
    utm_covariance_.resize(POSE_SIZE, POSE_SIZE);

    double utmX = 0;
    double utmY = 0;
    std::string utm_zone_tmp;
    NavsatConversions::LLtoUTM(gps_point->latitude, gps_point->longitude, utmY, utmX, utm_zone_tmp);
    utm_pose_.setOrigin(tf2::Vector3(utmX, utmY, gps_point->altitude));
    utm_covariance_.setZero();


    transformed_utm_gps.mult(utm_world_transform_, utm_pose_);
    transformed_utm_gps.setRotation(tf2::Quaternion::getIdentity());

    // Set header information stamp because we would like to know the robot's position at that timestamp
    gps_odom.header.frame_id = world_frame_id_;
    gps_odom.header.stamp = gps_update_time_;


    // Rotate the covariance as well
    tf2::Matrix3x3 rot(utm_world_transform_.getRotation());
    Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
    rot_6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd) {
      rot_6d(rInd, 0) = rot.getRow(rInd).getX();
      rot_6d(rInd, 1) = rot.getRow(rInd).getY();
      rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot_6d(rInd + POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot_6d(rInd + POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot_6d(rInd + POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Rotate the covariance
    utm_covariance_ = rot_6d * utm_covariance_.eval() * rot_6d.transpose();

    // Now fill out the message. Set the orientation to the identity.
    tf2::toMsg(transformed_utm_gps, gps_odom.pose.pose);
    gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

    // Copy the measurement's covariance matrix so that we can rotate it later
    for (size_t i = 0; i < POSE_SIZE; i++) {
      for (size_t j = 0; j < POSE_SIZE; j++) {
        gps_odom.pose.covariance[POSE_SIZE * i + j] = utm_covariance_(i, j);
      }
    }

    response.local_pose.pose.pose.position =  gps_odom.pose.pose.position;
    response.local_pose.pose.pose.orientation =  gps_odom.pose.pose.orientation;
    response.local_pose.pose.covariance =  gps_odom.pose.covariance;

  } else {
    return false;
  }
  return true;
}



//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::getRobotOriginUtmPose(const tf2::Transform &gps_utm_pose,
                                              tf2::Transform &robot_utm_pose,
                                              const ros::Time &transform_time){
    robot_utm_pose.setIdentity();

    // Get linear offset from origin for the GPS
    tf2::Transform offset;
    bool can_transform = lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 ros::Duration(transform_timeout_),
                                                                 offset);

    if (can_transform){
      // Get the orientation we'll use for our UTM->world transform
      tf2::Quaternion utm_orientation = transform_orientation_;
      tf2::Matrix3x3 mat(utm_orientation);

      // Add the offsets
      double roll;
      double pitch;
      double yaw;
      mat.getRPY(roll, pitch, yaw);
      yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
      utm_orientation.setRPY(roll, pitch, yaw);

      // Rotate the GPS linear offset by the orientation
      // Zero out the orientation, because the GPS orientation is meaningless, and if it's non-zero, it will make the
      // the computation of robot_utm_pose erroneous.
      offset.setOrigin(tf2::quatRotate(utm_orientation, offset.getOrigin()));
      offset.setRotation(tf2::Quaternion::getIdentity());

      // Update the initial pose
      robot_utm_pose = offset.inverse() * gps_utm_pose;
    }else{
      if (gps_frame_id_ != ""){
        ROS_WARN_STREAM_ONCE("Unable to obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
          " transform. Will assume navsat device is mounted at robot's origin");
      }

      robot_utm_pose = gps_utm_pose;
    }
  }


//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                                tf2::Transform &robot_odom_pose,
                                                const ros::Time &transform_time){
    robot_odom_pose.setIdentity();

    // Remove the offset from base_link
    tf2::Transform gps_offset_rotated;
    bool can_transform = lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 transform_timeout_,
                                                                 gps_offset_rotated);

    if (can_transform){
      tf2::Transform robot_orientation;
      can_transform = lookupTransformSafe(tf_buffer_,
                                                              world_frame_id_,
                                                              base_link_frame_id_,
                                                              transform_time,
                                                              transform_timeout_,
                                                              robot_orientation);

      if (can_transform){
        // Zero out rotation because we don't care about the orientation of the
        // GPS receiver relative to base_link
        gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
        gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
        robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
      }else{
        ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << world_frame_id_ << "->" << base_link_frame_id_ <<
          " transform. Will not remove offset of navsat device from robot's origin.");
      }
    }else{
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
        " transform. Will not remove offset of navsat device from robot's origin.");
    }
  }



//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg){

      gps_frame_id_ = msg->header.frame_id;

    if (gps_frame_id_.empty()){
      ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
        "origin.");
    }

    // Make sure the GPS data is usable
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                     !std::isnan(msg->altitude) &&
                     !std::isnan(msg->latitude) &&
                     !std::isnan(msg->longitude));

    if (good_gps){
      // If we haven't computed the transform yet, then
      // store this message as the initial GPS data to use
      if (!transform_good_ && !use_manual_datum_){
        setTransformGps(msg);
      }

      double utmX = 0;
      double utmY = 0;
      std::string utm_zone_tmp;
      NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utm_zone_tmp);
      latest_utm_pose_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));
      latest_utm_covariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSITION_SIZE; i++){
        for (size_t j = 0; j < POSITION_SIZE; j++){
          latest_utm_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
        }
      }

      gps_update_time_ = msg->header.stamp;
      gps_updated_ = true;
    }
  }


//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg){
    // We need the baseLinkFrameId_ from the odometry message, so
    // we need to wait until we receive it.
    if (has_transform_odom_){
      /* This method only gets called if we don't yet have the
       * IMU data (the subscriber gets shut down once we compute
       * the transform), so we can assumed that every IMU message
       * that comes here is meant to be used for that purpose. */
      tf2::fromMsg(msg->orientation, transform_orientation_);

      // Correct for the IMU's orientation w.r.t. base_link
      tf2::Transform target_frame_trans;
      bool can_transform = lookupTransformSafe(tf_buffer_,base_link_frame_id_, msg->header.frame_id, msg->header.stamp, transform_timeout_, target_frame_trans);

      if (can_transform){
        double roll_offset = 0;
        double pitch_offset = 0;
        double yaw_offset = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        quatToRPY(target_frame_trans.getRotation(), roll_offset, pitch_offset, yaw_offset);
        quatToRPY(transform_orientation_, roll, pitch, yaw);

        ROS_DEBUG_STREAM("Initial orientation is " << transform_orientation_);

        // Apply the offset (making sure to bound them), and throw them in a vector
        tf2::Vector3 rpy_angles(clampRotation(roll - roll_offset),
                                clampRotation(pitch - pitch_offset),
                                clampRotation(yaw - yaw_offset));

        // Now we need to rotate the roll and pitch by the yaw offset value.
        // Imagine a case where an IMU is mounted facing sideways. In that case
        // pitch for the IMU's world frame is roll for the robot.
        tf2::Matrix3x3 mat;
        mat.setRPY(0.0, 0.0, yaw_offset);
        rpy_angles = mat * rpy_angles;
        transform_orientation_.setRPY(rpy_angles.getX(), rpy_angles.getY(), rpy_angles.getZ());

        ROS_DEBUG_STREAM("Initial corrected orientation roll, pitch, yaw is (" <<
                         rpy_angles.getX() << ", " << rpy_angles.getY() << ", " << rpy_angles.getZ() << ")");

        has_transform_imu_ = true;
      }
    }
  }


//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    //world_frame_id_ = msg->header.frame_id;
    base_link_frame_id_ ="base_link" ; //msg->child_frame_id;

    AGROB_last_odometry_information.pose.pose = msg->pose.pose;
    AGROB_last_odometry_information.header = msg->header;

    if (!transform_good_ && !use_manual_datum_) {
      setTransformOdometry(msg);
    }
   
   geometry_msgs::Vector3Stamped data_odometry;
   tf2::Quaternion imu_quat;
   
   tf2::Matrix3x3 mat2(transform_orientation_);
   double g_roll;
   double g_pitch;
   double g_yaw;
   mat2.getRPY(g_roll, g_pitch, g_yaw);

   tf2_ros::Buffer tfBuffer;
   tf2_ros::TransformListener tf2_listener(tfBuffer);
   geometry_msgs::TransformStamped base_link_to_leap_motion; // My frames are named "base_link" and "leap_motion"

   try {

     base_link_to_leap_motion = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0) );

   } catch (tf2::TransformException ex) {
    //  flag_error = true;
    ROS_ERROR("TransformException: %s", ex.what());
   }

     imu_quat.setX(base_link_to_leap_motion.transform.rotation.x);
     imu_quat.setY(base_link_to_leap_motion.transform.rotation.y);
     imu_quat.setZ(base_link_to_leap_motion.transform.rotation.z);
     imu_quat.setW(base_link_to_leap_motion.transform.rotation.w);
     tf2::Matrix3x3 mat(imu_quat);

     double roll;
     double pitch;
     double yaw;

     mat.getRPY(roll, pitch, yaw);

     data_odometry.vector.z = -1.0*yaw+1.570795 - g_yaw ;
     //ROS_WARN("DEU %lf %lf %lf(", roll, pitch , yaw);

     //data_odometry.vector.z = pitch;
     /// imu_quat.setRPY(0.0, 0.0, msg->pose.pose);
     //data_odometry.orientation
     imu_odom_pub_.publish(data_odometry);

      nav_msgs::Odometry pose_temp;
      pose_temp.pose = msg->pose;

      if (robot_model == "agrobv18") {
        double yy;
        yy= pose_temp.pose.pose.position.y;
        pose_temp.pose.pose.position.y = pose_temp.pose.pose.position.x;
        pose_temp.pose.pose.position.x = pose_temp.pose.pose.position.z;
        pose_temp.pose.pose.position.z = yy;
      } else{ printf("OK\n\n");};


      tf2::fromMsg(pose_temp.pose.pose, latest_world_pose_);

      latest_odom_covariance_.setZero();
      for (size_t row = 0; row < POSE_SIZE; ++row)
      {
        for (size_t col = 0; col < POSE_SIZE; ++col)
        {
          latest_odom_covariance_(row, col) = msg->pose.covariance[row * POSE_SIZE + col];
        }
      }

      odom_update_time_ = msg->header.stamp;
      odom_updated_ = true;
  }



//***********************************************************************
//
//
//
//***********************************************************************
  bool NavSatTransform::prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps){
    bool new_data = false;

    if (transform_good_ && odom_updated_) {
      tf2::Transform odom_as_utm;

      odom_as_utm.mult(utm_world_trans_inverse_, latest_world_pose_);
      odom_as_utm.setRotation(tf2::Quaternion::getIdentity());


      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utm_world_trans_inverse_.getRotation());
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latest_odom_covariance_ = rot_6d * latest_odom_covariance_.eval() * rot_6d.transpose();

      // Now convert the data back to lat/long and place into the message
      NavsatConversions::UTMtoLL(odom_as_utm.getOrigin().getY(),
                                 odom_as_utm.getOrigin().getX(),
                                 utm_zone_,
                                 filtered_gps.latitude,
                                 filtered_gps.longitude);
      filtered_gps.altitude = odom_as_utm.getOrigin().getZ();

      // Copy the measurement's covariance matrix back
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          filtered_gps.position_covariance[POSITION_SIZE * i + j] = latest_odom_covariance_(i, j);
        }
      }

      filtered_gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      filtered_gps.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      filtered_gps.header.frame_id = "gps";
      filtered_gps.header.stamp = odom_update_time_;

      // Mark this GPS as used
      odom_updated_ = false;
      new_data = true;
    }

    return new_data;
  }



//***********************************************************************
//
//
//
//***********************************************************************
  bool NavSatTransform::prepareGpsOdometry(nav_msgs::Odometry &gps_odom){

    bool new_data = false;

    if (transform_good_ && gps_updated_  && odom_updated_){
      tf2::Transform transformed_utm_gps;

      transformed_utm_gps.mult(utm_world_transform_, latest_utm_pose_);
      transformed_utm_gps.setRotation(tf2::Quaternion::getIdentity());

      // Set header information stamp because we would like to know the robot's position at that timestamp
      gps_odom.header.frame_id = world_frame_id_;
      gps_odom.header.stamp = gps_update_time_;

      // Want the pose of the vehicle origin, not the GPS
      tf2::Transform transformed_utm_robot;
      getRobotOriginWorldPose(transformed_utm_gps, transformed_utm_robot, gps_odom.header.stamp);

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utm_world_transform_.getRotation());
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latest_utm_covariance_ = rot_6d * latest_utm_covariance_.eval() * rot_6d.transpose();

      // Now fill out the message. Set the orientation to the identity.
      tf2::toMsg(transformed_utm_robot, gps_odom.pose.pose);
      gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSE_SIZE; i++)
      {
        for (size_t j = 0; j < POSE_SIZE; j++)
        {
          gps_odom.pose.covariance[POSE_SIZE * i + j] = latest_utm_covariance_(i, j);
        }
      }



      // Mark this GPS as used
      gps_updated_ = false;
      new_data = true;
    }

    return new_data;
  }



//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    double utm_x = 0;
    double utm_y = 0;
    NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone_, utm_meridian_convergence_);
    utm_meridian_convergence_ *= NavsatConversions::RADIANS_PER_DEGREE;

    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " <<
                    msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum UTM coordinate is (" << std::fixed << utm_x << ", " << utm_y << ")");

    transform_utm_pose_.setOrigin(tf2::Vector3(utm_x, utm_y, msg->altitude));
    transform_utm_pose_.setRotation(tf2::Quaternion::getIdentity());
    has_transform_gps_ = true;
  }



//***********************************************************************
//
//
//
//***********************************************************************
  void NavSatTransform::setTransformOdometry(const nav_msgs::OdometryConstPtr& msg){
    tf2::fromMsg(msg->pose.pose, transform_world_pose_);
    has_transform_odom_ = true;

//    ROS_INFO_STREAM("Initial odometry pose is " << transform_world_pose_);

    // Users can optionally use the (potentially fused) heading from
    // the odometry source, which may have multiple fused sources of
    // heading data, and so would act as a better heading for the
    // UTM->world_frame transform.
    if (!transform_good_ && use_odometry_yaw_ && !use_manual_datum_){
      sensor_msgs::Imu *imu = new sensor_msgs::Imu();
      imu->orientation = msg->pose.pose.orientation;
      imu->header.frame_id = msg->child_frame_id;
      imu->header.stamp = msg->header.stamp;
      sensor_msgs::ImuConstPtr imuPtr(imu);
      imuCallback(imuPtr);
    }
  }


//***********************************************************************
//
//
//
//***********************************************************************

  double NavSatTransform::constrainAngle(double t){

    t = fmod(t + M_PI, 2 * M_PI);
    if (t < 0)
    {
      t += 2 * M_PI;
    }
    return t - M_PI;
  }
}  // namespace RobotLocalization
