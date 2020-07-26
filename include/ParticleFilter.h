/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: ParticleFilter.h
 *
 *          Created On: Wed Jul 15 23:09:07 2020
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <iostream>
#include <memory>
#include <set>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "misc.h"

struct Pose {
 public:
  Pose() : position_(Eigen::Vector2d::Zero()), angle_(0.0) {}
  Pose(const Eigen::Vector2d& p, double a) : position_(p), angle_(a) {}

  Pose(double x, double y, double a)
      : position_(Eigen::Vector2d(x, y)), angle_(a) {}

  const Eigen::Vector2d& position() const { return position_; }
  Eigen::Vector2d& position() { return position_; }

  double& angle() { return angle_; }
  const double& angle() const { return angle_; }

  Pose inverse() const {
    const Eigen::Rotation2Dd R = Eigen::Rotation2Dd(angle_).inverse();
    const Eigen::Vector2d t = -(R * position_);
    return Pose(t, R.angle());
  }

  friend std::ostream& operator<<(std::ostream& os, const Pose& p) {
    os << "position: (" << p.position()[0] << ", " << p.position()[1]
       << "), angle: " << p.angle();
    return os;
  }

  friend Eigen::Vector2d operator*(const Pose& p, const Eigen::Vector2d& point) {
    return Eigen::Rotation2Dd(p.angle()) * point + p.position();
  }

  Pose& operator=(const Pose& p) {
    position_ = p.position();
    angle_ = p.angle();
    return *this;
  }

 private:
  Eigen::Vector2d position_; // particle pos in world coordinate
  double angle_;        // particle angle in world coordinate
};

struct Particle {
 public:
  Particle(const Pose& pose, double w) : pose_(pose), weight_(w) {}
  Particle(const Eigen::Vector2d& p, double a, double w)
      : pose_(p, a), weight_(w) {}

  Particle(double x, double y, double a, double w)
      : pose_(Eigen::Vector2d(x, y), a), weight_(w) {}

  const Pose& pose() const { return pose_; }
  Pose& pose() { return pose_; }

  const Eigen::Vector2d& position() const { return pose_.position(); }
  Eigen::Vector2d& position() { return pose_.position(); }

  double& angle() { return pose_.angle(); }
  const double& angle() const { return pose_.angle(); }

  double& weight() { return weight_; }
  const double& weight() const { return weight_; }

  friend std::ostream& operator<<(std::ostream& os, const Particle& p) {
    os << "particle position: (" << p.pose().position()[0] << ", "
       << p.pose().position()[1] << "), angle: " << p.pose().angle()
       << ", weight: " << p.weight();
    return os;
  }

  Particle& operator=(const Particle& p) {
    pose_ = p.pose();
    weight_ = p.weight();
    return *this;
  }

 private:
  Pose pose_; // particle pos in world coordinate
  double weight_;       // particle weight
};

typedef std::shared_ptr<Particle> ParticlePtr;

class ParticleFilter {
 public:
  ParticleFilter(const int num, const bool display = false);

  ~ParticleFilter();

  void Run();

  void InitParticles();

  void UpdateParticlesPose(const Eigen::Vector2d& v, const double va);
  void UpdatePose(const Eigen::Vector2d& v, const double va, Pose& pose);

  void UpdateParticlesWeight(const std::vector<Eigen::Vector2d>& observations);

  void Resample();

  std::vector<Eigen::Vector2d> GetObservations(const Pose& pose);

  const std::vector<ParticlePtr>& particles() const { return particles_; }

  std::vector<ParticlePtr>& particles() { return particles_; }

 private:
  void NormalizePartilesWeight();

  void DrawHalfField();

  void DrawParticles(cv::Mat& image);

  void DrawRobot(cv::Mat& image, const Pose& pose, const cv::Scalar& color);
  void DrawRobot(cv::Mat& image, const Eigen::Vector2d& pos, const double angle,
                 const cv::Scalar& color);

 private:
  cv::Mat half_field_;

  std::vector<Eigen::Vector2d> objects_;
  std::vector<ParticlePtr> particles_;
  int n_;

  bool display_;
};

#endif
