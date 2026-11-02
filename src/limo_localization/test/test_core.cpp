#include "limo_localization/core.hpp"
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

using namespace limo_localization;
static void check(bool condition,const char *message) {if(!condition)throw std::runtime_error(message);}
int main() {
  try {
    Pose a{1,2,.4},b{-.3,.6,-.2};auto identity=compose(a,inverse(a));
    check(std::hypot(identity.x,identity.y)<1e-9 && std::abs(identity.yaw)<1e-9,"SE2 inverse");
    auto v=compose(a,b).apply({.1,.2}),w=a.apply(b.apply({.1,.2}));check((v-w).norm()<1e-9,"SE2 composition");
    cv::Mat mask=cv::Mat::zeros(300,400,CV_8U);
    cv::line(mask,{40,220},{340,220},255,2);cv::line(mask,{180,40},{180,250},255,2);
    cv::circle(mask,{270,130},35,255,2);
    MarkingMap map(mask,.01);
    Pose truth{1.1,1.2,.2},guess{1.16,1.16,.24};
    std::vector<Eigen::Vector2d> points;
    for(int y=0;y<mask.rows;y+=2)for(int x=0;x<mask.cols;x+=2)
      if(mask.at<uint8_t>(y,x))points.push_back(inverse(truth).apply({(x+.5)*.01,(mask.rows-y-.5)*.01}));
    for(auto method:{"icp","distance"}) {
      MatchOptions options;options.iterations=40;auto match=map.align(points,guess,method,options);
      std::cout<<method<<" error="<<std::hypot(match.pose.x-truth.x,match.pose.y-truth.y)<<" yaw="<<wrap(match.pose.yaw-truth.yaw)<<'\n';
      check(match.accepted,"Known transform accepted");
      check(std::hypot(match.pose.x-truth.x,match.pose.y-truth.y)<.025,"Known translation recovered");
      check(std::abs(wrap(match.pose.yaw-truth.yaw))<.02,"Known yaw recovered");
      check(!map.align({},guess,method).accepted,"Empty observations rejected");
      auto invalid=points;invalid[0].x()=std::nan("");check(!map.align(invalid,guess,method).accepted,"NaN rejected");
    }
    SearchOptions search;
    auto found=map.search(points,search);
    std::cout<<"global candidates="<<found.candidates.size()<<" margin="<<found.margin<<'\n';
    check(!found.candidates.empty(),"Global search finds candidates without any pose prior");
    std::cout<<"global pose="<<found.candidates[0].match.pose.x<<","<<found.candidates[0].match.pose.y<<","<<found.candidates[0].match.pose.yaw<<'\n';
    check(found.resolved,"Distinctive complete geometry has a unique global match");
    check(std::hypot(found.candidates[0].match.pose.x-truth.x,found.candidates[0].match.pose.y-truth.y)<.04,"Global position recovered");
    check(std::abs(wrap(found.candidates[0].match.pose.yaw-truth.yaw))<.03,"Unknown global heading recovered");
    check(!map.search({}).resolved,"Empty frame never establishes localization");
    cv::Mat duplicated=cv::Mat::zeros(300,800,CV_8U);
    mask.copyTo(duplicated(cv::Rect(0,0,400,300)));mask.copyTo(duplicated(cv::Rect(400,0,400,300)));
    auto ambiguous=MarkingMap(duplicated,.01).search(points,search);
    check(ambiguous.candidates.size()>=2 && !ambiguous.resolved,"Identical separated markings remain ambiguous");
    PoseConfirmation confirmation;Pose anchor{1,2,.1};
    check(!confirmation.observe(anchor,{},0,true),"Single exposure cannot initialize");
    check(!confirmation.observe(anchor,{},0,true) && confirmation.count()==1,"Duplicate frame cannot confirm");
    check(!confirmation.observe(compose(anchor,{.2,0,0}),{.2,0,0},.5,true),"Second motion-consistent observation");
    check(confirmation.observe(compose(anchor,{.4,0,0}),{.4,0,0},1,true),"Three consistent observations establish pose");
    check(!confirmation.observe(anchor,{},2,false) && confirmation.count()==0,"Ambiguity clears confirmation");
    check(!confirmation.observe(anchor,{},3,true),"Restart confirmation");
    check(!confirmation.observe({5,4,1},{},3.5,true) && confirmation.count()==1,"Inconsistent pose restarts confirmation");
    check(!confirmation.observe({5,4,1},{},6,true) && confirmation.count()==1,"Long frame gap restarts confirmation");
    cv::Mat line=cv::Mat::zeros(200,300,CV_8U);cv::line(line,{10,100},{290,100},255,1);
    MarkingMap line_map(line,.01);std::vector<Eigen::Vector2d> line_points;
    for(int x=50;x<250;++x)line_points.emplace_back((x+.5)*.01,.995);
    auto weak=line_map.align(line_points,{.03,.03,0},"distance");
    check(weak.min_eigenvalue<1e-4,"Parallel-line weak direction detected");
    check(weak.covariance(0,0)>.1,"Unobservable motion not overconfident");
    check(!line_map.search(line_points).resolved,"Straight line cannot establish a global position");
    PoseFilter filter;filter.reset({0,0,0},Mat3::Identity()*.01);
    filter.predict(0,1,0);filter.predict(1,1,0);filter.predict(2,1,0);
    check(std::abs(filter.latest().pose.x-2)<1e-9,"Wheel prediction");
    check(filter.correct(1,{1.1,0,0},Mat3::Identity()*.001),"Delayed measurement accepted");
    check(filter.latest().pose.x>2.08 && filter.latest().pose.x<2.11,"Prediction replay after delayed correction");
    check(!filter.correct(.5,{.5,0,0},Mat3::Identity()*.001),"Out-of-order camera update rejected");
    check(!filter.correct(2,{100,100,0},Mat3::Identity()*.001),"Innovation outlier rejected");
    PoseFilter between;between.reset({0,0,0},Mat3::Identity()*.01);
    between.predict(0,1,0);between.predict(1,1,0);between.predict(2,1,0);
    check(between.correct(.5,{.6,0,0},Mat3::Identity()*.001),"Sub-odometry-time update");
    State interpolated;check(between.at(.6,interpolated) && interpolated.pose.x>.68,"Correction retained between wheel samples");
    check(between.reanchor(.6,{4,3,-.4},Mat3::Identity()*.001),"Confirmed kidnapped pose can bypass local innovation gate");
    check(std::abs(between.latest().pose.x-(4+1.4*std::cos(.4)))<1e-6,"Reanchoring replays subsequent motion");
    check(!between.reanchor(-1,{0,0,0},Mat3::Identity()),"Reanchor cannot use absent capture-time history");
    check(!between.reanchor(2,{0,0,0},-Mat3::Identity()),"Reanchor rejects invalid covariance");
    PoseFilter turning;turning.reset({0,0,3.13},Mat3::Identity()*.01);
    turning.predict(0,0,.1);turning.predict(.2,0,.1);
    check(turning.correct(.2,{0,0,-3.13},Mat3::Identity()*.001),"Yaw innovation crosses pi continuously");
    Camera camera;camera.width=40;camera.height=30;camera.fx=camera.fy=30;camera.cx=20;camera.cy=15;
    camera.rotation<<0,0,1,-1,0,0,0,-1,0;camera.translation={0,0,.3};
    FeatureOptions fo;fo.ground_z=0;fo.min_forward=.1;fo.max_forward=3;
    cv::Mat white(30,40,CV_8UC3,cv::Scalar(230,230,230));
    auto ipm=extractFeatures(white,camera,fo);check(!ipm.empty(),"IPM ground rays");
    cv::Mat depth(30,40,CV_32F,cv::Scalar(std::nanf("")));
    check(extractFeatures(white,camera,fo,depth,camera).empty(),"Invalid depth rejected");
    // Analytic plane depth: optical Z is forward, optical Y points down.
    for(int y=16;y<30;++y)for(int x=0;x<40;++x)depth.at<float>(y,x)=.3*30/(y-15);
    auto rgbd=extractFeatures(white,camera,fo,depth,camera);
    check(!rgbd.empty(),"RGB-D calibrated projection");
    for(auto p:rgbd)check(p.allFinite(),"Finite RGB-D points");
    Camera low=camera;low.width=20;low.height=15;low.fx=low.fy=15;low.cx=10;low.cy=7.5;
    low.translation.x()=.04;
    cv::Mat low_depth(15,20,CV_16U,cv::Scalar(0));
    for(int y=8;y<15;++y)for(int x=0;x<20;++x)low_depth.at<uint16_t>(y,x)=std::lround(1000*.3*15/(y-7.5));
    check(!extractFeatures(white,camera,fo,low_depth,low).empty(),"Different depth resolution/extrinsics and millimetre encoding");
    fo.max_points=5;check(extractFeatures(white,camera,fo).size()==5,"Bounded feature count");
    std::cout<<"All localization core checks passed\n";return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
