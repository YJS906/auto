#include "limo_localization/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <Eigen/Geometry>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <memory>
#include <algorithm>
#include <array>
#include <iostream>

namespace ll=limo_localization;
using Image=sensor_msgs::msg::Image;
using Info=sensor_msgs::msg::CameraInfo;
using Odom=nav_msgs::msg::Odometry;
using PoseMessage=geometry_msgs::msg::PoseWithCovarianceStamped;
static double seconds(const builtin_interfaces::msg::Time &t) {return t.sec+t.nanosec*1e-9;}
static Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion &q) {
  Eigen::Quaterniond r(q.w,q.x,q.y,q.z);
  if(!r.coeffs().allFinite() || r.norm()<1e-6) throw std::invalid_argument("Invalid quaternion");
  return r.normalized();
}
static ll::Pose planar(const geometry_msgs::msg::Pose &p) {
  auto r=quaternion(p.orientation).toRotationMatrix();
  return {p.position.x,p.position.y,std::atan2(r(1,0),r(0,0))};
}
static geometry_msgs::msg::Pose poseMessage(const ll::Pose &p) {
  geometry_msgs::msg::Pose msg;msg.position.x=p.x;msg.position.y=p.y;
  msg.orientation.z=std::sin(p.yaw*.5);msg.orientation.w=std::cos(p.yaw*.5);return msg;
}

class Localizer:public rclcpp::Node {
public:
  Localizer():Node("road_marking_localizer"),buffer_(get_clock()),listener_(buffer_),broadcaster_(*this) {
    cv::setNumThreads(declare_parameter("opencv_threads",1));
    method_=declare_parameter<std::string>("matcher","distance");
    extraction_=declare_parameter<std::string>("projection","ipm");
    if((method_!="distance" && method_!="icp") || (extraction_!="ipm" && extraction_!="rgbd"))
      throw std::invalid_argument("matcher: distance/icp; projection: ipm/rgbd");
    base_=declare_parameter<std::string>("base_frame","base_link");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    publish_tf_=declare_parameter("publish_tf",true);
    initial_={declare_parameter("initial_x",1.66),declare_parameter("initial_y",5.53),declare_parameter("initial_yaw",.03)};
    initial_covariance_=ll::Mat3::Identity()*.01;filter_.reset(initial_,initial_covariance_);
    motion_.reset({},initial_covariance_);
    seeded_=!declare_parameter("global_initialization",true);
    search_options_.xy_step=declare_parameter("search_xy_step",.20);
    search_options_.yaw_step=declare_parameter("search_yaw_step",.2617993877991494);
    search_options_.candidates=declare_parameter("search_candidates",32);
    search_options_.score_margin=declare_parameter("search_score_margin",.004);
    search_options_.max_score=declare_parameter("search_max_score",.025);
    search_options_.min_eigenvalue=declare_parameter("search_min_eigenvalue",.002);
    global_rate_=declare_parameter("global_search_rate",1.0);
    audit_period_=declare_parameter("global_audit_period",2.0);
    lost_timeout_=declare_parameter("tracking_timeout",.8);
    confirmation_.required=declare_parameter("confirmation_frames",3);
    if(!std::isfinite(global_rate_+audit_period_+lost_timeout_) || global_rate_<=0 || global_rate_>10 ||
       audit_period_<=0 || lost_timeout_<=0 || confirmation_.required<2 || confirmation_.required>20)
      throw std::invalid_argument("Invalid global localization timing/confirmation settings");
    filter_.velocity_noise=declare_parameter("velocity_noise",.03);
    filter_.yaw_rate_noise=declare_parameter("yaw_rate_noise",.03);
    imu_weight_=declare_parameter("imu_weight",.8);
    sync_tolerance_=declare_parameter("sync_tolerance",.060);
    max_age_=declare_parameter("max_image_age",.5);
    max_tilt_=declare_parameter("max_tilt",.12);
    rate_=declare_parameter("processing_rate",15.0);
    features_.white_min=declare_parameter("white_min",160);
    features_.max_chroma=declare_parameter("max_chroma",90);
    features_.stride=declare_parameter("pixel_stride",3);
    features_.max_points=declare_parameter("max_points",800);
    features_.ground_z=declare_parameter("ground_z",-.045);
    features_.ground_tolerance=declare_parameter("ground_tolerance",.025);
    features_.min_forward=declare_parameter("min_forward",.15);
    features_.max_forward=declare_parameter("max_forward",2.0);
    features_.max_lateral=declare_parameter("max_lateral",1.2);
    options_.iterations=declare_parameter("iterations",15);
    options_.min_points=declare_parameter("min_points",40);
    options_.max_rmse=declare_parameter("max_rmse",.035);
    options_.max_distance=declare_parameter("max_correspondence",.15);
    options_.min_inlier_ratio=declare_parameter("min_inlier_ratio",.5);
    options_.max_translation=declare_parameter("max_correction",.2);
    options_.max_yaw=declare_parameter("max_yaw_correction",.25);
    for(double value:{rate_,imu_weight_,sync_tolerance_,max_age_,max_tilt_,features_.ground_z,
        features_.ground_tolerance,features_.min_forward,features_.max_forward,features_.max_lateral,
        filter_.velocity_noise,filter_.yaw_rate_noise})
      if(!std::isfinite(value))throw std::invalid_argument("Localization parameters must be finite");
    if(!std::isfinite(rate_) || rate_<=0 || rate_>120 || imu_weight_<0 || imu_weight_>1 ||
       sync_tolerance_<=0 || max_age_<=0 || max_tilt_<=0 || features_.stride<1 ||
       features_.max_points<options_.min_points || features_.white_min<0 || features_.white_min>255 ||
       features_.max_chroma<0 || features_.max_chroma>255 || features_.max_forward<=features_.min_forward ||
       features_.ground_tolerance<=0 || features_.max_lateral<=0 || filter_.velocity_noise<=0 || filter_.yaw_rate_noise<=0)
      throw std::invalid_argument("Invalid localization parameter range");
    map_=std::make_unique<ll::MarkingMap>(ll::MarkingMap::fromTexture(
      declare_parameter<std::string>("map_image",""),declare_parameter("map_width",10.0),
      declare_parameter("map_height",6.0),declare_parameter("map_resolution",.01)));
    // Validate matcher settings before accepting sensor data.
    map_->align({},initial_,method_,options_);
    map_->search({},search_options_);
    pose_pub_=create_publisher<PoseMessage>("localization/pose",10);
    visual_pub_=create_publisher<PoseMessage>("localization/visual_pose",10);
    diag_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("localization/diagnostics",10);
    ready_pub_=create_publisher<std_msgs::msg::Bool>("localization/ready",rclcpp::QoS(1).transient_local());
    state_pub_=create_publisher<std_msgs::msg::String>("localization/state",rclcpp::QoS(1).transient_local());
    candidates_pub_=create_publisher<geometry_msgs::msg::PoseArray>("localization/candidates",1);
    publishState();
    auto qos=rclcpp::SensorDataQoS().keep_last(2);
    color_sub_=create_subscription<Image>("image",qos,[this](Image::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);color_=msg;colors_.push_back(msg);
      while(colors_.size()>6){colors_.pop_front();}
      ++received_;cv_.notify_one();});
    depth_sub_=create_subscription<Image>("depth",qos,[this](Image::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);depths_.push_back(msg);while(depths_.size()>6)depths_.pop_front();cv_.notify_one();});
    info_sub_=create_subscription<Info>("camera_info",qos,[this](Info::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);color_info_=msg;});
    depth_info_sub_=create_subscription<Info>("depth_info",qos,[this](Info::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);depth_info_=msg;});
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("imu",qos,[this](sensor_msgs::msg::Imu::ConstSharedPtr msg){onImu(*msg);});
    odom_sub_=create_subscription<Odom>("odom",qos,[this](Odom::ConstSharedPtr msg){onOdom(*msg);});
    initial_sub_=create_subscription<PoseMessage>("initialpose",10,[this](PoseMessage::ConstSharedPtr msg){
      try {
        if(msg->header.frame_id!=map_frame_) throw std::invalid_argument("initialpose frame must be map_frame");
        ll::Mat3 covariance;const int ids[]={0,1,5};
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)covariance(i,j)=msg->pose.covariance[ids[i]*6+ids[j]];
        std::lock_guard<std::mutex> lock(mutex_);
        filter_.reset(planar(msg->pose.pose),covariance);last_odom_time_=-1;last_visual_time_=-1;
        seeded_=true;observations_.clear();confirmation_.reset();consecutive_rejected_=0;
        mode_=Mode::Confirming;publishState();
        ++generation_;status_="initialpose received; waiting for odometry and markings";
      }catch(const std::exception &e){RCLCPP_WARN(get_logger(),"Initial pose rejected: %s",e.what());}});
    heartbeat_=create_wall_timer(std::chrono::milliseconds(100),[this]{diagnostics();});
    worker_=std::thread([this]{work();});
    RCLCPP_INFO(get_logger(),"%s / %s, CPU OpenCV; no ground truth subscription",extraction_.c_str(),method_.c_str());
  }
  ~Localizer() override {
    {std::lock_guard<std::mutex> lock(mutex_);running_=false;cv_.notify_all();}
    worker_.join();
  }
private:
  enum class Mode {Searching,Confirming,Tracking,Lost};
  const char *modeName() const {
    switch(mode_){case Mode::Searching:return "SEARCHING";case Mode::Confirming:return "CONFIRMING";
      case Mode::Tracking:return "TRACKING";case Mode::Lost:return "LOST";}
    return "LOST";
  }
  void publishState() {
    std_msgs::msg::Bool ready;ready.data=mode_==Mode::Tracking;ready_pub_->publish(ready);
    std_msgs::msg::String state;state.data=modeName();state_pub_->publish(state);
  }
  void lose(const std::string &reason) {
    mode_=Mode::Lost;seeded_=false;confirmation_.reset();observations_.clear();
    consecutive_rejected_=0;last_audit_=-1e9;++generation_;status_=reason;publishState();
  }
  ll::Camera camera(const Info &info,const std::string &frame,const rclcpp::Time &stamp) {
    if(!info.header.frame_id.empty() && info.header.frame_id!=frame)
      throw std::invalid_argument("Image/CameraInfo frame mismatch");
    if(!info.distortion_model.empty() && info.distortion_model!="plumb_bob" && info.distortion_model!="rational_polynomial")
      throw std::invalid_argument("Rectify unsupported camera distortion before input");
    auto tf=buffer_.lookupTransform(base_,frame,stamp);
    ll::Camera c;c.width=info.width;c.height=info.height;c.fx=info.k[0];c.fy=info.k[4];c.cx=info.k[2];c.cy=info.k[5];
    c.distortion=info.d;c.rotation=quaternion(tf.transform.rotation).toRotationMatrix();
    c.translation={tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z};c.validate();return c;
  }
  void onImu(const sensor_msgs::msg::Imu &msg) {
    if(msg.angular_velocity_covariance[0]<0) return;
    try {
      auto tf=buffer_.lookupTransform(base_,msg.header.frame_id,rclcpp::Time(msg.header.stamp));
      ll::Mat3 r=quaternion(tf.transform.rotation).toRotationMatrix();
      ll::Vec3 angular=r*ll::Vec3(msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z);
      if(!angular.allFinite()) return;
      double tilt=0;
      if(msg.orientation_covariance[0]>=0) {
        ll::Mat3 world_from_base=quaternion(msg.orientation).toRotationMatrix()*r.transpose();
        tilt=std::acos(std::clamp(world_from_base(2,2),-1.0,1.0));
      }
      std::lock_guard<std::mutex> lock(mutex_);
      imus_.push_back({seconds(msg.header.stamp),angular.z(),tilt});while(imus_.size()>500)imus_.pop_front();
      if(mode_==Mode::Tracking && tilt>max_tilt_)lose("tilt/lift detected; global relocalization required");
    }catch(const std::exception &) {/* A valid TF is required before using an IMU sample. */}
  }
  void onOdom(const Odom &msg) {
    try {
      const double t=seconds(msg.header.stamp),v=msg.twist.twist.linear.x;
      double w=msg.twist.twist.angular.z;
      if(msg.child_frame_id!=base_ || !std::isfinite(v+w)) return;
      auto raw=planar(msg.pose.pose);
      std::lock_guard<std::mutex> lock(mutex_);
      if(t>now().seconds()+.02 || now().seconds()-t>.5)return;
      if(last_odom_time_>=0 && t<last_odom_time_) {
        if(now().seconds()>=last_odom_time_-.1)return;  // late packet, not a /clock reset
        filter_.reset(initial_,initial_covariance_);color_.reset();colors_.clear();depths_.clear();imus_.clear();
        motion_.reset({},initial_covariance_);last_visual_time_=-1;lose("clock reset");
      }
      if(last_odom_time_>=0 && t-last_odom_time_>.5) {
        auto state=filter_.latest();filter_.reset(state.pose,state.covariance+ll::Mat3::Identity()*.01);
        motion_.reset(motion_.latest().pose,initial_covariance_);lose("odometry gap; relocalizing");
      }
      double best=.05;
      for(auto it=imus_.rbegin();it!=imus_.rend();++it)
        if(std::abs((*it)[0]-t)<best){best=std::abs((*it)[0]-t);w=(1-imu_weight_)*msg.twist.twist.angular.z+imu_weight_*(*it)[1];}
      filter_.predict(t,v,w);motion_.predict(t,v,w);last_odom_time_=t;
      if(filter_.empty()) return;
      if(mode_!=Mode::Tracking)return;  // no authoritative pose/TF before confirmation
      const auto &state=filter_.latest();
      PoseMessage out;out.header.stamp=msg.header.stamp;out.header.frame_id=map_frame_;
      out.pose.pose=poseMessage(state.pose);out.pose.pose.position.z=-features_.ground_z;
      fillCovariance(out,state.covariance);pose_pub_->publish(out);
      if(publish_tf_) {
        auto correction=ll::compose(state.pose,ll::inverse(raw));
        geometry_msgs::msg::TransformStamped tf;tf.header=out.header;tf.child_frame_id=msg.header.frame_id;
        tf.transform.translation.x=correction.x;tf.transform.translation.y=correction.y;
        tf.transform.translation.z=-features_.ground_z-msg.pose.pose.position.z;
        tf.transform.rotation=poseMessage(correction).orientation;broadcaster_.sendTransform(tf);
      }
    }catch(const std::exception &e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Odom rejected: %s",e.what());}
  }
  static void fillCovariance(PoseMessage &msg,const ll::Mat3 &covariance) {
    const int ids[]={0,1,5};msg.pose.covariance[14]=msg.pose.covariance[21]=msg.pose.covariance[28]=1e6;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)msg.pose.covariance[ids[i]*6+ids[j]]=covariance(i,j);
  }
  void work() {
    double processed=-1;
    uint64_t processed_generation=0;
    auto next=std::chrono::steady_clock::now();
    while(true) {
      Image::ConstSharedPtr rgb,depth;Info::ConstSharedPtr info,depth_info;
      ll::State prediction,depth_prediction,motion;uint64_t generation;bool global=false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[&]{return !running_ || (color_ && seconds(color_->header.stamp)!=processed);});
        if(!running_) return;
        cv_.wait_until(lock,next,[&]{return !running_;});if(!running_) return;
        rgb=color_;info=color_info_;depth_info=depth_info_;generation=generation_;
        if(processed_generation!=generation){processed=-1;processed_generation=generation;}
        if(extraction_=="rgbd") {
          // Pair captured frames, not the latest independently arriving images.
          // Depth often arrives after the RGB frame with the same timestamp.
          rgb.reset();
          for(auto it=colors_.rbegin();it!=colors_.rend() && !rgb;++it) {
            if(seconds((*it)->header.stamp)<=processed) continue;
            double best=sync_tolerance_;
            for(const auto &d:depths_)if(std::abs(seconds(d->header.stamp)-seconds((*it)->header.stamp))<best) {
              depth=d;best=std::abs(seconds(d->header.stamp)-seconds((*it)->header.stamp));rgb=*it;
            }
          }
          if(!rgb || !depth_info){status_="waiting for synchronized depth and calibration";
            cv_.wait_for(lock,std::chrono::milliseconds(10));continue;}
        }
        double t=seconds(rgb->header.stamp);
        if(!info || !filter_.at(t,prediction) || !motion_.at(t,motion)) {
          status_="waiting for calibration/time-bracketing odometry";
          cv_.wait_for(lock,std::chrono::milliseconds(10));continue;
        }
        if(depth && !motion_.at(seconds(depth->header.stamp),depth_prediction)) {
          status_="waiting for depth-time odometry";cv_.wait_for(lock,std::chrono::milliseconds(10));continue;
        }
        processed=t;++processed_;
        double age=now().seconds()-t;
        if(age<-.01 || age>max_age_) {status_="image stale/future";continue;}
        bool tilted=false;
        for(const auto &imu:imus_)if(std::abs(imu[0]-t)<.05 && imu[2]>max_tilt_)tilted=true;
        if(tilted){status_="tilted ground: visual correction withheld";continue;}
        global=!seeded_ && (mode_!=Mode::Tracking || t-last_audit_>=audit_period_);
      }
      next=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/rate_));
      const auto start=std::chrono::steady_clock::now();
      try {
        auto color_camera=camera(*info,rgb->header.frame_id,rclcpp::Time(rgb->header.stamp));
        auto color_view=cv_bridge::toCvShare(rgb,"bgr8");
        ll::Camera depth_camera;cv::Mat depth_image;
        cv_bridge::CvImageConstPtr depth_view;
        if(depth){depth_camera=camera(*depth_info,depth->header.frame_id,rclcpp::Time(depth->header.stamp));
          auto displacement=ll::compose(ll::inverse(motion.pose),depth_prediction.pose);
          ll::Mat3 rotation=Eigen::AngleAxisd(displacement.yaw,ll::Vec3::UnitZ()).toRotationMatrix();
          depth_camera.translation=rotation*depth_camera.translation+ll::Vec3(displacement.x,displacement.y,0);
          depth_camera.rotation=rotation*depth_camera.rotation;
          depth_view=cv_bridge::toCvShare(depth);depth_image=depth_view->image;}
        auto points=ll::extractFeatures(color_view->image,color_camera,features_,depth_image,depth_camera);
        auto extracted=std::chrono::steady_clock::now();
        ll::GlobalMatch global_match;
        double search_ms=0;
        if(global) {
          std::vector<Eigen::Vector2d> accumulated;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if(generation!=generation_)continue;
            if(mode_==Mode::Tracking || points.size()<static_cast<size_t>(options_.min_points))observations_.clear();
            observations_.push_back({motion.pose,seconds(rgb->header.stamp),points});
            while(observations_.size()>3 || seconds(rgb->header.stamp)-observations_.front().time>3)observations_.pop_front();
            for(const auto &observation:observations_) {
              auto relative=ll::compose(ll::inverse(motion.pose),observation.motion);
              for(const auto &p:observation.points)accumulated.push_back(relative.apply(p));
            }
          }
          // Bound refinement work independently of history size.
          if(accumulated.size()>1600) {
            std::vector<Eigen::Vector2d> sampled;
            for(size_t i=0;i<1600;++i)sampled.push_back(accumulated[i*accumulated.size()/1600]);
            accumulated=std::move(sampled);
          }
          if(points.size()>=static_cast<size_t>(options_.min_points)) {
            const auto search_start=std::chrono::steady_clock::now();
            global_match=map_->search(accumulated,search_options_);
            search_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-search_start).count();
          }
        }
        auto match=map_->align(points,prediction.pose,method_,options_);
        auto end=std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        if(generation!=generation_) continue;
        last_features_=points.size();last_rmse_=match.rmse;last_inliers_=match.inlier_ratio;last_eigen_=match.min_eigenvalue;
        extraction_ms_=std::chrono::duration<double,std::milli>(extracted-start).count();
        match_ms_=std::chrono::duration<double,std::milli>(end-extracted).count();
        latency_ms_=(now().seconds()-seconds(rgb->header.stamp))*1000;
        const double t=seconds(rgb->header.stamp);
        bool accepted=false;
        if(global) {
          last_audit_=t;global_search_ms_=search_ms;++global_searches_;
          global_margin_=global_match.margin;global_candidates_=global_match.candidates.size();
          global_score_=global_match.candidates.empty()?.15:global_match.candidates.front().score;
          geometry_msgs::msg::PoseArray candidates;candidates.header=rgb->header;candidates.header.frame_id=map_frame_;
          for(size_t i=0;i<std::min(size_t(8),global_match.candidates.size());++i)
            candidates.poses.push_back(poseMessage(global_match.candidates[i].match.pose));
          candidates_pub_->publish(candidates);
          if(mode_==Mode::Tracking && global_match.resolved) {
            const auto &pose=global_match.candidates.front().match.pose;
            if(std::hypot(pose.x-prediction.pose.x,pose.y-prediction.pose.y)>.3 ||
               std::abs(ll::wrap(pose.yaw-prediction.pose.yaw))>.3)
              lose("global audit contradicts tracked pose; confirming new location");
          }
          if(mode_!=Mode::Tracking) {
            const bool confirmed=!global_match.candidates.empty() && confirmation_.observe(
              global_match.candidates.front().match.pose,motion.pose,t,global_match.resolved);
            if(confirmed) {
              match=global_match.candidates.front().match;
              accepted=filter_.reanchor(t,match.pose,match.covariance);
              if(accepted){mode_=Mode::Tracking;observations_.clear();confirmation_.reset();publishState();}
            }else {
              if(!global_match.resolved)confirmation_.reset();
              mode_=global_match.resolved?Mode::Confirming:(mode_==Mode::Lost?Mode::Lost:Mode::Searching);
              status_=global_match.resolved?"confirming unique global hypothesis":"ambiguous or insufficient global evidence";
              publishState();
            }
            if(mode_!=Mode::Tracking)next=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/global_rate_));
          }
        }
        if(!accepted && mode_==Mode::Tracking)accepted=match.accepted && filter_.correct(t,match.pose,match.covariance);
        else if(!global && mode_!=Mode::Tracking && seeded_) {
          if(confirmation_.observe(match.pose,motion.pose,t,match.accepted)) {
            accepted=filter_.reanchor(t,match.pose,match.covariance);
            if(accepted){mode_=Mode::Tracking;seeded_=false;confirmation_.reset();publishState();}
          }
          if(!accepted)status_="confirming supplied initial pose";
        }
        if(accepted) {
          ++accepted_;consecutive_rejected_=0;last_visual_time_=t;status_="tracking";
          PoseMessage visual;visual.header=rgb->header;visual.header.frame_id=map_frame_;
          visual.pose.pose=poseMessage(match.pose);visual.pose.pose.position.z=-features_.ground_z;
          fillCovariance(visual,match.covariance);visual_pub_->publish(visual);
        }else {
          ++rejected_;
          if(mode_==Mode::Tracking || (seeded_ && !match.accepted)) {
            ++consecutive_rejected_;
            if(consecutive_rejected_>=5)lose("consecutive corrections rejected; global search");
            else if(mode_==Mode::Tracking)status_="visual update rejected; short prediction continues";
          }else if(seeded_ && match.accepted)consecutive_rejected_=0;
        }
        diagnosticsUnlocked();
      }catch(const std::exception &e){
        std::lock_guard<std::mutex> lock(mutex_);status_=e.what();++rejected_;diagnosticsUnlocked();
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Visual update: %s",e.what());
      }
    }
  }
  void diagnostics(){
    std::lock_guard<std::mutex> lock(mutex_);
    if(mode_==Mode::Tracking && (now().seconds()-last_visual_time_>lost_timeout_ || now().seconds()-last_odom_time_>.3))
      lose("sensor/visual timeout; global relocalization required");
    publishState();diagnosticsUnlocked();
  }
  void diagnosticsUnlocked() {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="road_marking_localizer";s.hardware_id="cpu";s.message=status_;
    double age=last_visual_time_<0?1e6:now().seconds()-last_visual_time_;
    s.level=mode_==Mode::Tracking?0:mode_==Mode::Lost?2:1;
    diagnostic_msgs::msg::KeyValue state;state.key="state";state.value=modeName();s.values.push_back(state);
    auto add=[&](const std::string &key,auto value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=std::to_string(value);s.values.push_back(kv);};
    add("received",received_);add("processed",processed_);add("accepted",accepted_);add("rejected",rejected_);
    add("features",last_features_);add("rmse_m",last_rmse_);add("inlier_ratio",last_inliers_);add("min_eigenvalue",last_eigen_);
    add("extraction_ms",extraction_ms_);add("matching_ms",match_ms_);add("capture_to_result_sim_ms",latency_ms_);
    add("visual_age_sim_s",age);add("global_score_m",global_score_);add("global_margin_m",global_margin_);
    add("global_candidates",global_candidates_);add("confirmation_count",confirmation_.count());
    add("global_search_ms",global_search_ms_);add("global_searches",global_searches_);
    array.status.push_back(s);diag_pub_->publish(array);
  }
  std::string method_,extraction_,base_,map_frame_,status_="waiting for sensors";
  bool publish_tf_,running_=true;
  Mode mode_=Mode::Searching;
  bool seeded_=false;
  double global_rate_=1,audit_period_=2,lost_timeout_=.8,last_audit_=-1e9,global_score_=.15,global_margin_=0;
  size_t global_candidates_=0;
  double global_search_ms_=0;uint64_t global_searches_=0;
  int consecutive_rejected_=0;
  ll::SearchOptions search_options_;ll::PoseConfirmation confirmation_;ll::PoseFilter motion_;
  struct Observation {ll::Pose motion;double time;std::vector<Eigen::Vector2d> points;};
  std::deque<Observation> observations_;
  double imu_weight_,sync_tolerance_,max_age_,max_tilt_,rate_,last_odom_time_=-1,last_visual_time_=-1;
  double last_rmse_=0,last_inliers_=0,last_eigen_=0,extraction_ms_=0,match_ms_=0,latency_ms_=0;
  size_t last_features_=0;
  uint64_t received_=0,processed_=0,accepted_=0,rejected_=0,generation_=0;
  std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
  ll::Pose initial_;ll::Mat3 initial_covariance_;ll::PoseFilter filter_;ll::FeatureOptions features_;ll::MatchOptions options_;
  std::unique_ptr<ll::MarkingMap> map_;
  Image::ConstSharedPtr color_;Info::ConstSharedPtr color_info_,depth_info_;
  std::deque<Image::ConstSharedPtr> colors_,depths_;std::deque<std::array<double,3>> imus_;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Publisher<PoseMessage>::SharedPtr pose_pub_,visual_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidates_pub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_,depth_sub_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_,depth_info_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<PoseMessage>::SharedPtr initial_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<Localizer>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);executor.spin();
  }catch(const std::exception &e){std::cerr<<"Localization startup failed: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
