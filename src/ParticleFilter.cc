/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: ParticleFilter.cc
 *
 *          Created On: Wed Jul 15 23:08:58 2020
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#include "ParticleFilter.h"
#include <glog/logging.h>

const cv::Scalar kRed = cv::Scalar(0, 0, 255);
const cv::Scalar kGreen = cv::Scalar(0, 255, 0);
const cv::Scalar kBlue = cv::Scalar(255, 0, 0);

constexpr double kDegreeToRad = M_PI / 180.0;
constexpr double kRadToDegree = 180.0 / M_PI;

constexpr double kRSigma = 0.04;
constexpr double kAngleSigma = 2 * kDegreeToRad;
constexpr double kObservationSigma = 0.1;

ParticleFilter::ParticleFilter(const int num, const bool display)
    : n_(num),
      display_(display) {
  objects_.resize(2);
  // left/right goalpost at field (4.5m, +/-1.5m)
  objects_[0] = Eigen::Vector2d(4.5, 1.5); // left goalpost
  objects_[1] = Eigen::Vector2d(4.5, -1.5);  // right goalpost

  if (display_)
    DrawHalfField();
  particles_.reserve(n_);
  InitParticles();
}

ParticleFilter::~ParticleFilter() {}

/**
 *  Main loop function, update all particles with control command
 *  "w", "s" will control robot move forward, backword +-0.1m
 *  "a", "d" will control robot turn left, right +-5 degrees
 *  ideal pose means command without white noise
 *  real pose means robot pose with white noise
 */
void ParticleFilter::Run() {
  Pose ideal_pose(0., 0., 0.); // ideal pose without noise
  Pose real_pose(0., 0., 0.);  // real pose with noise added

  double ideal_linear_velocity = 0.0;
  double real_linear_velocity = 0.0;

  double ideal_angle_velocity = 0.0;
  double real_angle_velocity = 0.0;

  while (true) {
    cv::Mat display_mat = half_field_.clone();
    DrawParticles(display_mat);
    DrawRobot(display_mat, ideal_pose, kRed);
    DrawRobot(display_mat, real_pose, kBlue);

    /****/
    ParticlePtr max_particle;
    double max_weight = std::numeric_limits<double>::min();
    for (const auto& p : particles_) {
      if (p->weight() > max_weight) {
        max_particle = p;
        max_weight = p->weight();
      }
    }

    std::cout << *max_particle << std::endl;
    DrawRobot(display_mat, max_particle->pose(), kGreen);
    /****/

    cv::imshow("particles", display_mat);
    int key = cv::waitKey(0);
    if (key == int('w')) {
      ideal_linear_velocity = 0.10;   // ideal linear velocity at 0.10 m/s
      real_linear_velocity = ideal_linear_velocity + WhiteNoise(0.0, kRSigma);

      ideal_angle_velocity = 0.0;
      real_angle_velocity = 0.0;

    } else if (key == int('s')) {
      ideal_linear_velocity = -0.10;   // ideal linear velocity at -0.10 m/s
      real_linear_velocity = ideal_linear_velocity + WhiteNoise(0.0, kRSigma);

      ideal_angle_velocity = 0.0;
      real_angle_velocity = 0.0;

    } else if (key == int('a')) {
      ideal_angle_velocity = 5. * kDegreeToRad; // ideal angle velocity at 5 degree/s
      real_angle_velocity = ideal_angle_velocity + WhiteNoise(0.0, kAngleSigma);

      ideal_linear_velocity = 0.0;
      real_linear_velocity = 0.0;

    } else if (key == int('d')) {
      ideal_angle_velocity = -5. * kDegreeToRad; // ideal angle velocity at -5 degree/s
      real_angle_velocity = ideal_angle_velocity + WhiteNoise(0.0, kAngleSigma);

      ideal_linear_velocity = 0.0;
      real_linear_velocity = 0.0;

    } else if (key == int('q')) {
      break;
    }
    Eigen::Vector2d ideal_velocity(ideal_linear_velocity, 0.0);
    Eigen::Vector2d real_velocity(real_linear_velocity, 0.0);

    UpdateParticlesPose(real_velocity, real_angle_velocity);
    UpdatePose(ideal_velocity, ideal_angle_velocity, ideal_pose);
    UpdatePose(real_velocity, real_angle_velocity, real_pose);

    std::vector<Eigen::Vector2d> observations = GetObservations(real_pose);
    UpdateParticlesWeight(observations);
    Resample();
  }
}

/**
 *  Init all particles with uniform distribution
 */
void ParticleFilter::InitParticles() {
  for (int i = 0; i < n_; i++) {
    double x = UniformRandom(0.0, 4.5);
    double y = UniformRandom(-3.0, 3.0);
    double angle = UniformRandom(-M_PI, M_PI);
    double weight = 1.0 / n_;

    ParticlePtr p = std::make_shared<Particle>(x, y, angle, weight);
    particles_.push_back(p);
  }
  return;
}

void ParticleFilter::UpdateParticlesPose(const Eigen::Vector2d& v,
                                         const double va) {
  for (auto& p : particles_)
    UpdatePose(v, va, p->pose());
  return;
}

/**
 * v: velocity in robot coordinate (m/s)
 * va: angular velocity in robot coordinate (rad/s)
 */
void ParticleFilter::UpdatePose(const Eigen::Vector2d& v, const double va,
                                Pose& pose) {
   // Fill here
  return;
}

/**
 *  Update particles weight with observations
 */
void ParticleFilter::UpdateParticlesWeight(
    const std::vector<Eigen::Vector2d>& observations) {
  // Fill here
  return;
}

/**
 *  Resample particles, delete low weight particles and
 *  add some new particles
 */
void ParticleFilter::Resample() {
  // Fill here
}

/**
 *  Get observations from data
 *  We suppose robot has 60 degrees viewing angle and 3.5m viewing distance
 *  so we can get all observations within 3.5m and 60 degrees.
 *  Here we assume observation has 10% error
 *  return observations' pose in robot coordinate
 */
std::vector<Eigen::Vector2d> ParticleFilter::GetObservations(const Pose& pose) {
  std::vector<Eigen::Vector2d> observations;
  for (size_t i = 0; i < objects_.size(); i++) {
    Eigen::Vector2d pose_relative = pose.inverse() * objects_[i];
    double distance = pose_relative.norm();
    // distance greater than 3.5m, robot cannot observe it
    if (distance > 3.5)
      continue;

    double angle = atan2(pose_relative.y(), pose_relative.x());
    if (angle > 30 * M_PI or angle < -30 * M_PI)
      continue;
    pose_relative.x() += WhiteNoise(0., pose_relative.x() * 0.1);
    pose_relative.y() += WhiteNoise(0., pose_relative.y() * 0.1);
    observations.push_back(pose_relative);
  }
  return observations;
}

/**
 *  Normalize all particle weights
 */
void ParticleFilter::NormalizePartilesWeight() {
  double sum = 0.0;
  for (auto& p : particles_)
    sum += p->weight();

  for (auto& p : particles_)
    p->weight() = p->weight() / sum;
}

/**
 *  Draw half football field in image
 */
void ParticleFilter::DrawHalfField() {
  int height = 600;   // 600 pixel, 1 pixel = 1cm
  int width = 550;    // 550 pixel, 1 pixel = 1cm

  // create fields
  half_field_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 100, 0));
  // draw goalposts
  for (size_t i = 0; i < objects_.size(); i++) {
    cv::circle(half_field_, ConvertToPixelPos(objects_[i]), 5, cv::Scalar(0, 155, 255), 1);
  }
}

/**
 *  Draw particles on field
 */
void ParticleFilter::DrawParticles(cv::Mat& image) {
  Eigen::Vector2d arrow_length(0.1, 0.0);
  for (const auto& p : particles_) {
    Eigen::Rotation2Dd R(p->angle());
    Eigen::Vector2d pt = p->position() + R * arrow_length;
    cv::arrowedLine(image, ConvertToPixelPos(p->position()),
                    ConvertToPixelPos(pt), cv::Scalar(0, 0, 0), 1, 8, 0, 0.5);
  }
}

/**
 *  Draw robot pose on image
 */
void ParticleFilter::DrawRobot(cv::Mat& image, const Pose& pose,
                               const cv::Scalar& color) {
  DrawRobot(image, pose.position(), pose.angle(), color);
}

void ParticleFilter::DrawRobot(cv::Mat& image, const Eigen::Vector2d& position,
                               const double angle, const cv::Scalar& color) {
  Eigen::Vector2d arrow_length(0.1, 0.0);
  Eigen::Rotation2Dd R(angle);
  Eigen::Vector2d pt = position + R * arrow_length;
  cv::arrowedLine(image, ConvertToPixelPos(position),
                  ConvertToPixelPos(pt), color, 1, 8, 0, 0.5);
}
