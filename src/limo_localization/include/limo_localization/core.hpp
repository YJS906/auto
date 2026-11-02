#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <deque>
#include <string>
#include <vector>

namespace limo_localization {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
double wrap(double angle);
struct Pose {
  double x=0, y=0, yaw=0;
  Eigen::Vector2d apply(const Eigen::Vector2d &p) const;
};
Pose compose(const Pose &a, const Pose &b);
Pose inverse(const Pose &p);

struct Camera {
  int width=0, height=0;
  double fx=0, fy=0, cx=0, cy=0;
  std::vector<double> distortion;
  Mat3 rotation=Mat3::Identity();  // optical -> base
  Vec3 translation=Vec3::Zero();
  void validate() const;
};
struct FeatureOptions {
  int white_min=160, max_chroma=90, stride=3, max_points=800;
  double ground_z=-0.045, ground_tolerance=0.025;
  double min_forward=0.15, max_forward=2.0, max_lateral=1.2;
};
// RGB is BGR8. Depth is optical-axis Z, 32FC1 metres or 16UC1 millimetres.
// RGB-D uses calibrated reprojection: matching resolutions/optical frames are NOT assumed.
std::vector<Eigen::Vector2d> extractFeatures(
  const cv::Mat &rgb, const Camera &color, const FeatureOptions &options,
  const cv::Mat &depth={}, const Camera &depth_camera={});

struct MatchOptions {
  int iterations=15, min_points=40;
  double max_distance=0.15, max_rmse=0.035, min_inlier_ratio=0.5;
  double max_translation=0.2, max_yaw=0.25;
};
struct Match {
  Pose pose;
  Mat3 covariance=Mat3::Identity();
  double rmse=1e9, inlier_ratio=0, min_eigenvalue=0;
  int points=0, iterations=0;
  bool accepted=false;
};
struct SearchOptions {
  double xy_step=0.20, yaw_step=0.2617993877991494;
  double score_margin=0.004, max_score=0.025, min_eigenvalue=0.002;
  int coarse_points=80, candidates=32;
};
struct Hypothesis {Match match;double score=1e3;};
struct GlobalMatch {
  std::vector<Hypothesis> candidates;
  double margin=0;
  bool resolved=false;
};
class MarkingMap {
public:
  MarkingMap(const cv::Mat &mask, double resolution, double origin_x=0, double origin_y=0);
  static MarkingMap fromTexture(const std::string &path, double width, double height, double resolution);
  Match align(const std::vector<Eigen::Vector2d> &points, const Pose &initial,
              const std::string &method, const MatchOptions &options={}) const;
  double distance(const Eigen::Vector2d &p) const;
  GlobalMatch search(const std::vector<Eigen::Vector2d> &points,
                     const SearchOptions &options={}) const;
private:
  double resolution_, ox_, oy_;
  cv::Mat distance_, labels_;
  std::vector<Eigen::Vector2d> nearest_;
  bool pixel(const Eigen::Vector2d &p, double &u, double &v) const;
  bool nearest(const Eigen::Vector2d &p, Eigen::Vector2d &q) const;
};

// Require independent, time-ordered confirmations consistent with wheel/gyro motion.
// Ambiguous observations never count as confirmation.
class PoseConfirmation {
public:
  int required=3;
  bool observe(const Pose &pose,const Pose &motion,double time,bool unique);
  void reset() {count_=0;}
  int count() const {return count_;}
private:
  int count_=0;
  double time_=0;
  Pose pose_,motion_;
};

struct State {
  double time=0, velocity=0, yaw_rate=0;
  Pose pose;
  Mat3 covariance=Mat3::Identity()*0.01;
};
// Three-state planar EKF. Camera updates are applied at capture time, then
// wheel/gyro predictions are replayed through the bounded history.
class PoseFilter {
public:
  double velocity_noise=0.03, yaw_rate_noise=0.03;
  void reset(const Pose &pose, const Mat3 &covariance);
  void predict(double time, double velocity, double yaw_rate);
  bool at(double time, State &state) const;
  bool correct(double time, const Pose &measurement, const Mat3 &covariance);
  // Used ONLY after confirmed global localization/manual initialization.
  bool reanchor(double time,const Pose &pose,const Mat3 &covariance);
  const State &latest() const {return history_.back();}
  bool empty() const {return history_.empty();}
private:
  State seed_;
  double last_correction_=-1e30;
  std::deque<State> history_;
  void advance(State &state, double time, double velocity, double yaw_rate) const;
  void replay(const State &state);
};
}  // namespace limo_localization
