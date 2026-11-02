#include "limo_localization/core.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace limo_localization {
double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
Eigen::Vector2d Pose::apply(const Eigen::Vector2d &p) const {
  return {x+std::cos(yaw)*p.x()-std::sin(yaw)*p.y(),
          y+std::sin(yaw)*p.x()+std::cos(yaw)*p.y()};
}
Pose compose(const Pose &a,const Pose &b) {
  auto p=a.apply({b.x,b.y}); return {p.x(),p.y(),wrap(a.yaw+b.yaw)};
}
Pose inverse(const Pose &p) {
  double c=std::cos(p.yaw),s=std::sin(p.yaw);
  return {-c*p.x-s*p.y,s*p.x-c*p.y,-p.yaw};
}
void Camera::validate() const {
  if(width<=0 || height<=0 || !std::isfinite(fx) || !std::isfinite(fy) || fx<=0 || fy<=0 ||
     !std::isfinite(cx) || !std::isfinite(cy) || !rotation.allFinite() || !translation.allFinite())
    throw std::invalid_argument("Invalid camera calibration");
  for(auto d:distortion) if(!std::isfinite(d)) throw std::invalid_argument("Invalid distortion");
}
static cv::Mat intrinsic(const Camera &c) {
  return (cv::Mat_<double>(3,3)<<c.fx,0,c.cx,0,c.fy,c.cy,0,0,1);
}
static bool white(const cv::Vec3b &p,const FeatureOptions &o) {
  auto lo=std::min({p[0],p[1],p[2]}),hi=std::max({p[0],p[1],p[2]});
  return lo>=o.white_min && hi-lo<=o.max_chroma;
}
std::vector<Eigen::Vector2d> extractFeatures(const cv::Mat &rgb,const Camera &c,
  const FeatureOptions &o,const cv::Mat &depth,const Camera &d) {
  c.validate();
  if(rgb.type()!=CV_8UC3 || rgb.cols!=c.width || rgb.rows!=c.height || o.stride<1 ||
     o.max_points<1 || o.ground_tolerance<=0 || o.max_forward<=o.min_forward || o.max_lateral<=0)
    throw std::invalid_argument("Invalid feature input/options");
  const bool rgbd=!depth.empty();
  if(rgbd) {
    d.validate();
    if((depth.type()!=CV_32FC1 && depth.type()!=CV_16UC1) || depth.cols!=d.width || depth.rows!=d.height)
      throw std::invalid_argument("Depth must match its own CameraInfo (32FC1 or 16UC1)");
  }
  // Only RGB-D needs to project into the undistorted color image. No pointcloud
  // allocation: operate directly on selected pixels and bounded feature vectors.
  cv::Mat rectified=rgb;
  if(rgbd && !c.distortion.empty() && cv::norm(c.distortion)>1e-12)
    cv::undistort(rgb,rectified,intrinsic(c),c.distortion);
  const Camera &source=rgbd?d:c;
  struct RayCache {Camera camera;int stride=0;std::vector<cv::Point2f> pixels,rays;};
  thread_local RayCache cache;
  const auto &old=cache.camera;
  if(cache.stride!=o.stride || old.width!=source.width || old.height!=source.height ||
     old.fx!=source.fx || old.fy!=source.fy || old.cx!=source.cx || old.cy!=source.cy || old.distortion!=source.distortion) {
    cache.camera=source;cache.stride=o.stride;cache.pixels.clear();
    for(int y=0;y<source.height;y+=o.stride)for(int x=0;x<source.width;x+=o.stride)cache.pixels.emplace_back(x,y);
    cv::undistortPoints(cache.pixels,cache.rays,intrinsic(source),source.distortion);
  }
  const auto &pixels=cache.pixels;const auto &rays=cache.rays;
  std::vector<Eigen::Vector2d> result;
  result.reserve(pixels.size());
  for(size_t i=0;i<pixels.size();++i) {
    if(!rgbd && !white(rgb.at<cv::Vec3b>(static_cast<int>(pixels[i].y),static_cast<int>(pixels[i].x)),o)) continue;
    Vec3 ray(rays[i].x,rays[i].y,1),p;
    if(rgbd) {
      int x=static_cast<int>(pixels[i].x),y=static_cast<int>(pixels[i].y);
      double z=depth.type()==CV_32FC1?depth.at<float>(y,x):depth.at<uint16_t>(y,x)*0.001;
      if(!std::isfinite(z) || z<=0) continue;
      p=d.rotation*(ray*z)+d.translation;
      if(std::abs(p.z()-o.ground_z)>o.ground_tolerance) continue;
      Vec3 pc=c.rotation.transpose()*(p-c.translation);
      if(pc.z()<=0) continue;
      int u=std::lround(c.fx*pc.x()/pc.z()+c.cx),v=std::lround(c.fy*pc.y()/pc.z()+c.cy);
      if(u<0 || u>=c.width || v<0 || v>=c.height || !white(rectified.at<cv::Vec3b>(v,u),o)) continue;
    } else {
      ray=c.rotation*ray;
      if(ray.z()>=-1e-6) continue;
      double t=(o.ground_z-c.translation.z())/ray.z();
      if(t<=0) continue;
      p=c.translation+t*ray;
    }
    if(p.x()<o.min_forward || p.x()>o.max_forward || std::abs(p.y())>o.max_lateral) continue;
    result.emplace_back(p.x(),p.y());
  }
  if(result.size()>static_cast<size_t>(o.max_points)) {
    std::vector<Eigen::Vector2d> sampled; sampled.reserve(o.max_points);
    for(int i=0;i<o.max_points;++i) sampled.push_back(result[i*result.size()/o.max_points]);
    return sampled;
  }
  return result;
}
MarkingMap MarkingMap::fromTexture(const std::string &path,double width,double height,double resolution) {
  if(width<=0 || height<=0 || resolution<=0 || !std::isfinite(width+height+resolution) ||
     width/resolution<2 || height/resolution<2 || width*height/(resolution*resolution)>4000000)
    throw std::invalid_argument("Invalid/beyond bounded map dimensions");
  cv::Mat texture=cv::imread(path,cv::IMREAD_GRAYSCALE),small,mask;
  if(texture.empty()) throw std::runtime_error("Cannot read marking map: "+path);
  cv::resize(texture,small,cv::Size(std::lround(width/resolution),std::lround(height/resolution)),0,0,cv::INTER_AREA);
  cv::threshold(small,mask,150,255,cv::THRESH_BINARY);
  return MarkingMap(mask,resolution);
}
MarkingMap::MarkingMap(const cv::Mat &mask,double r,double ox,double oy):resolution_(r),ox_(ox),oy_(oy) {
  if(mask.empty() || mask.type()!=CV_8UC1 || !std::isfinite(r) || r<=0 || !std::isfinite(ox+oy) || cv::countNonZero(mask>127)==0)
    throw std::invalid_argument("Invalid/empty marking map");
  cv::Mat inverted,approx;
  cv::threshold(mask,inverted,127,255,cv::THRESH_BINARY_INV);
  cv::distanceTransform(inverted,distance_,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  distance_*=r;
  cv::distanceTransform(inverted,approx,labels_,cv::DIST_L2,5,cv::DIST_LABEL_PIXEL);
  double max_label;cv::minMaxLoc(labels_,nullptr,&max_label);
  nearest_.resize(static_cast<size_t>(max_label)+1);
  for(int y=0;y<mask.rows;++y) for(int x=0;x<mask.cols;++x)
    if(inverted.at<uint8_t>(y,x)==0)
      nearest_[labels_.at<int>(y,x)]={ox+(x+.5)*r,oy+(mask.rows-y-.5)*r};
}
bool MarkingMap::pixel(const Eigen::Vector2d &p,double &u,double &v) const {
  u=(p.x()-ox_)/resolution_-.5;v=distance_.rows-.5-(p.y()-oy_)/resolution_;
  return p.allFinite() && u>=0 && v>=0 && u<distance_.cols-1 && v<distance_.rows-1;
}
double MarkingMap::distance(const Eigen::Vector2d &p) const {
  double u,v;if(!pixel(p,u,v)) return 1e3;
  int x=static_cast<int>(u),y=static_cast<int>(v);double a=u-x,b=v-y;
  return (1-a)*(1-b)*distance_.at<float>(y,x)+a*(1-b)*distance_.at<float>(y,x+1)+
         (1-a)*b*distance_.at<float>(y+1,x)+a*b*distance_.at<float>(y+1,x+1);
}
GlobalMatch MarkingMap::search(const std::vector<Eigen::Vector2d> &points,const SearchOptions &o) const {
  if(!std::isfinite(o.xy_step+o.yaw_step+o.score_margin+o.max_score+o.min_eigenvalue) ||
     o.xy_step<.05 || o.yaw_step<.05 || o.yaw_step>1 || o.score_margin<=0 ||
     o.max_score<=0 || o.min_eigenvalue<=0 || o.coarse_points<10 || o.coarse_points>500 ||
     o.candidates<2 || o.candidates>128)
    throw std::invalid_argument("Invalid bounded global search options");
  const double lattice=std::ceil(distance_.cols*resolution_/o.xy_step)*
    std::ceil(distance_.rows*resolution_/o.xy_step)*std::ceil(2*M_PI/o.yaw_step);
  if(lattice>1000000)throw std::invalid_argument("Global search exceeds one million coarse poses");
  GlobalMatch result;
  if(points.size()<40) return result;
  for(const auto &p:points)if(!p.allFinite())return result;
  auto score=[&](const Pose &pose,int count) {
    double cost=0;
    for(int i=0;i<count;++i) {
      double d=std::min(.15,distance(pose.apply(points[static_cast<size_t>(i)*points.size()/count])));
      cost+=d*d;
    }
    return std::sqrt(cost/count);
  };
  auto close=[](const Pose &a,const Pose &b,double xy,double yaw) {
    return std::hypot(a.x-b.x,a.y-b.y)<xy && std::abs(wrap(a.yaw-b.yaw))<yaw;
  };
  struct Seed {Pose pose;double score;};
  std::vector<Seed> seeds;
  const int count=std::min(static_cast<int>(points.size()),o.coarse_points);
  // ponytail: bounded deterministic beam, not a full particle filter. If the
  // coarse basin is missed, reduce steps/expand the beam after CPU profiling.
  for(double yaw=-M_PI;yaw<M_PI;yaw+=o.yaw_step)
    for(double y=oy_+o.xy_step/2;y<oy_+distance_.rows*resolution_;y+=o.xy_step)
      for(double x=ox_+o.xy_step/2;x<ox_+distance_.cols*resolution_;x+=o.xy_step) {
        Seed seed{{x,y,yaw},score({x,y,yaw},count)};
        if(seed.score>.10)continue;
        auto nearby=std::find_if(seeds.begin(),seeds.end(),[&](const Seed &s){return close(s.pose,seed.pose,.3,.35);});
        if(nearby!=seeds.end()) {if(seed.score<nearby->score)*nearby=seed;continue;}
        if(static_cast<int>(seeds.size())<o.candidates)seeds.push_back(seed);
        else {
          auto worst=std::max_element(seeds.begin(),seeds.end(),[](const Seed &a,const Seed &b){return a.score<b.score;});
          if(seed.score<worst->score)*worst=seed;
        }
      }
  MatchOptions refine;refine.iterations=30;refine.max_distance=.3;refine.max_translation=.5;
  refine.max_yaw=.45;refine.max_rmse=.04;refine.min_inlier_ratio=.8;
  for(const auto &seed:seeds) {
    auto match=align(points,seed.pose,"distance",refine);
    if(!match.accepted)continue;
    Hypothesis candidate{match,score(match.pose,std::min(static_cast<int>(points.size()),800))};
    auto nearby=std::find_if(result.candidates.begin(),result.candidates.end(),[&](const Hypothesis &h){
      return close(h.match.pose,match.pose,.20,.15);});
    if(nearby==result.candidates.end())result.candidates.push_back(candidate);
    else if(candidate.score<nearby->score)*nearby=candidate;
  }
  std::sort(result.candidates.begin(),result.candidates.end(),[](const Hypothesis &a,const Hypothesis &b){return a.score<b.score;});
  if(result.candidates.empty())return result;
  result.margin=result.candidates.size()>1?result.candidates[1].score-result.candidates[0].score:.15-result.candidates[0].score;
  const auto &best=result.candidates.front();
  result.resolved=best.score<=o.max_score && best.match.min_eigenvalue>=o.min_eigenvalue && result.margin>=o.score_margin;
  return result;
}
bool PoseConfirmation::observe(const Pose &pose,const Pose &motion,double time,bool unique) {
  if(!unique || !std::isfinite(pose.x+pose.y+pose.yaw+motion.x+motion.y+motion.yaw+time)) {reset();return false;}
  if(count_ && time<=time_)return false;
  auto expected=compose(pose_,compose(inverse(motion_),motion));
  if(!count_ || time-time_>2.0 || std::hypot(pose.x-expected.x,pose.y-expected.y)>.10 ||
     std::abs(wrap(pose.yaw-expected.yaw))>.12)count_=0;
  // Adjacent callbacks of the same exposure must not establish confidence.
  if(count_ && time-time_<.1)return false;
  ++count_;pose_=pose;motion_=motion;time_=time;
  return count_>=required;
}
bool MarkingMap::nearest(const Eigen::Vector2d &p,Eigen::Vector2d &q) const {
  double u,v;if(!pixel(p,u,v)) return false;
  q=nearest_[labels_.at<int>(std::lround(v),std::lround(u))];return true;
}
Match MarkingMap::align(const std::vector<Eigen::Vector2d> &points,const Pose &initial,
                        const std::string &method,const MatchOptions &o) const {
  if((method!="icp" && method!="distance") || o.iterations<1 || o.min_points<3 ||
     !std::isfinite(o.max_distance+o.max_rmse+o.min_inlier_ratio+o.max_translation+o.max_yaw) ||
     o.max_distance<=0 || o.max_rmse<=0 || o.min_inlier_ratio<=0 || o.min_inlier_ratio>1 ||
     o.max_translation<=0 || o.max_yaw<=0 || !std::isfinite(initial.x+initial.y+initial.yaw))
    throw std::invalid_argument("Invalid matcher options");
  Match result;result.pose=initial;result.points=static_cast<int>(points.size());
  if(result.points<o.min_points) return result;
  for(const auto &p:points) if(!p.allFinite()) return result;
  Mat3 information=Mat3::Zero();
  for(int iteration=0;iteration<o.iterations;++iteration) {
    Mat3 h=Mat3::Zero();Vec3 g=Vec3::Zero();int used=0;
    for(const auto &p:points) {
      auto world=result.pose.apply(p);
      double dist=distance(world);if(dist>o.max_distance) continue;
      Eigen::Matrix<double,2,3> j;
      j<<1,0,-(world.y()-result.pose.y),0,1,world.x()-result.pose.x;
      double weight=dist>.025?.025/dist:1.0;
      if(method=="icp") {
        Eigen::Vector2d target;if(!nearest(world,target)) continue;
        Eigen::Vector2d residual=world-target;
        h+=weight*j.transpose()*j;g+=weight*j.transpose()*residual;
      } else {
        Eigen::Vector2d dx(resolution_,0),dy(0,resolution_);
        double xp=distance(world+dx),xm=distance(world-dx),yp=distance(world+dy),ym=distance(world-dy);
        if(std::max({xp,xm,yp,ym})>10) continue;
        Eigen::RowVector2d gradient((xp-xm)/(2*resolution_),(yp-ym)/(2*resolution_));
        Eigen::RowVector3d jacobian=gradient*j;
        h+=weight*jacobian.transpose()*jacobian;g+=weight*jacobian.transpose()*dist;
      }
      ++used;
    }
    if(used<o.min_points) return result;
    h/=used;g/=used;
    // Geometry-derived distance Hessian also describes weak directions of ICP:
    // point-to-point correspondences alone falsely imply observability along a line.
    if(method=="distance") information=h;
    const Vec3 prior(result.pose.x-initial.x,result.pose.y-initial.y,wrap(result.pose.yaw-initial.yaw));
    Vec3 step=-(h+Mat3::Identity()*.005).ldlt().solve(g+.005*prior);
    if(!step.allFinite()) return result;
    double scale=std::max({1.0,step.head<2>().norm()/.05,std::abs(step.z())/.05});step/=scale;
    result.pose.x+=step.x();result.pose.y+=step.y();result.pose.yaw=wrap(result.pose.yaw+step.z());
    result.iterations=iteration+1;
    if(step.norm()<1e-5) break;
  }
  double squared=0;int inliers=0;information.setZero();
  for(const auto &p:points) {
    auto world=result.pose.apply(p);double d=distance(world);
    if(d>o.max_distance) continue;
    squared+=d*d;++inliers;
    Eigen::Vector2d dx(resolution_,0),dy(0,resolution_);
    double xp=distance(world+dx),xm=distance(world-dx),yp=distance(world+dy),ym=distance(world-dy);
    if(std::max({xp,xm,yp,ym})>10) continue;
    Vec3 j((xp-xm)/(2*resolution_),(yp-ym)/(2*resolution_),0);
    j.z()=-j.x()*(world.y()-result.pose.y)+j.y()*(world.x()-result.pose.x);
    information+=j*j.transpose();
  }
  result.inlier_ratio=static_cast<double>(inliers)/points.size();
  if(inliers<o.min_points) return result;
  result.rmse=std::sqrt(squared/inliers);information/=inliers;
  Eigen::SelfAdjointEigenSolver<Mat3> eig(information);
  result.min_eigenvalue=eig.eigenvalues().minCoeff();
  // Pixel correlations make N raw pixels overconfident. Cap effective support.
  const double variance=std::max(0.0001,result.rmse*result.rmse)/std::min(inliers,30);
  Vec3 variances;
  for(int i=0;i<3;++i) variances[i]=std::clamp(variance/std::max(eig.eigenvalues()[i],1e-8),1e-5,.25);
  result.covariance=eig.eigenvectors()*variances.asDiagonal()*eig.eigenvectors().transpose();
  result.accepted=result.rmse<=o.max_rmse && result.inlier_ratio>=o.min_inlier_ratio &&
    std::hypot(result.pose.x-initial.x,result.pose.y-initial.y)<=o.max_translation &&
    std::abs(wrap(result.pose.yaw-initial.yaw))<=o.max_yaw;
  return result;
}
void PoseFilter::reset(const Pose &p,const Mat3 &cov) {
  if(!std::isfinite(p.x+p.y+p.yaw) || !cov.allFinite() || !cov.isApprox(cov.transpose()) ||
     Eigen::SelfAdjointEigenSolver<Mat3>(cov).eigenvalues().minCoeff()<=0)
    throw std::invalid_argument("Initial covariance must be positive definite");
  seed_={};seed_.pose=p;seed_.covariance=cov;history_.clear();last_correction_=-1e30;
}
void PoseFilter::advance(State &s,double t,double v,double w) const {
  double dt=t-s.time,heading=s.pose.yaw+.5*w*dt;
  Mat3 f=Mat3::Identity();f(0,2)=-v*dt*std::sin(heading);f(1,2)=v*dt*std::cos(heading);
  Eigen::Matrix<double,3,2> b;
  b<<dt*std::cos(heading),-.5*v*dt*dt*std::sin(heading),
     dt*std::sin(heading), .5*v*dt*dt*std::cos(heading),0,dt;
  Eigen::Matrix2d q=Eigen::Matrix2d::Zero();q(0,0)=velocity_noise*velocity_noise;q(1,1)=yaw_rate_noise*yaw_rate_noise;
  // Continuous random walk floor accounts for unmodelled slip while stationary.
  s.covariance=f*s.covariance*f.transpose()+b*q*b.transpose()+Mat3::Identity()*1e-5*dt;
  s.pose.x+=v*dt*std::cos(heading);s.pose.y+=v*dt*std::sin(heading);s.pose.yaw=wrap(s.pose.yaw+w*dt);
  s.time=t;s.velocity=v;s.yaw_rate=w;
}
void PoseFilter::predict(double t,double v,double w) {
  if(!std::isfinite(t+v+w)) return;
  if(history_.empty()) {seed_.time=t;history_.push_back(seed_);return;}
  if(t<=history_.back().time) return;
  State s=history_.back();advance(s,t,v,w);history_.push_back(s);
  while(history_.size()>2 && t-history_.front().time>5) history_.pop_front();
}
bool PoseFilter::at(double t,State &s) const {
  if(history_.empty() || t<history_.front().time || t>history_.back().time) return false;
  auto it=std::lower_bound(history_.begin(),history_.end(),t,[](const State &a,double b){return a.time<b;});
  if(it==history_.begin() || std::abs(it->time-t)<1e-9) {s=*it;return true;}
  auto prev=std::prev(it);s=*prev;advance(s,t,it->velocity,it->yaw_rate);return true;
}
bool PoseFilter::correct(double t,const Pose &measurement,const Mat3 &r) {
  State s;if(t<=last_correction_ || !at(t,s) || !r.allFinite() || !r.isApprox(r.transpose()) ||
    Eigen::SelfAdjointEigenSolver<Mat3>(r).eigenvalues().minCoeff()<=0 ||
    !std::isfinite(measurement.x+measurement.y+measurement.yaw)) return false;
  Vec3 innovation(measurement.x-s.pose.x,measurement.y-s.pose.y,wrap(measurement.yaw-s.pose.yaw));
  Mat3 sum=s.covariance+r;
  if(innovation.dot(sum.ldlt().solve(innovation))>16.27) return false;  // chi-square(3), 99.9%
  Mat3 k=s.covariance*sum.inverse();Vec3 delta=k*innovation;
  s.pose.x+=delta.x();s.pose.y+=delta.y();s.pose.yaw=wrap(s.pose.yaw+delta.z());
  Mat3 a=Mat3::Identity()-k;s.covariance=a*s.covariance*a.transpose()+k*r*k.transpose();
  replay(s);return true;
}
bool PoseFilter::reanchor(double t,const Pose &pose,const Mat3 &covariance) {
  State state;if(!at(t,state) || !std::isfinite(pose.x+pose.y+pose.yaw) || !covariance.allFinite() ||
    !covariance.isApprox(covariance.transpose()) || Eigen::SelfAdjointEigenSolver<Mat3>(covariance).eigenvalues().minCoeff()<=0)return false;
  state.pose=pose;state.covariance=covariance;replay(state);return true;
}
void PoseFilter::replay(const State &state) {
  State s=state;const double t=s.time;
  auto insertion=std::lower_bound(history_.begin(),history_.end(),t,[](const State &h,double stamp){return h.time<stamp;});
  if(insertion==history_.end() || std::abs(insertion->time-t)>1e-9) history_.insert(insertion,s);
  for(auto &h:history_) if(h.time>=t) {
    double time=h.time,v=h.velocity,w=h.yaw_rate;
    if(time>t) advance(s,time,v,w);
    h=s;
  }
  last_correction_=t;
}
}  // namespace limo_localization
