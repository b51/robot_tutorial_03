/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: misc.h
 *
 *          Created On: Thu Jul 16 22:53:28 2020
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#ifndef MISC_H_
#define MISC_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <Eigen/Geometry>

// Adjust angle to keep it within -pi<=angle<=+pi
static double ModAngle(double angle) {
  while (angle >= M_PI)
    angle -= 2 * M_PI;
  while (angle <= -M_PI)
    angle += 2* M_PI;

  return angle;
}

// generate random number in [l, r)
static double UniformRandom(double l, double r) {
  std::random_device rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dist(l, r);
  return dist(gen);
}

static double WhiteNoise(double mu, double sigma) {
  std::random_device rd{};
  std::mt19937 gen{rd()};
  // values near the mean are the most likely
  // standard deviation affects the dispersion of generated values from the mean
  std::normal_distribution<> distribute{mu, sigma};
  return distribute(gen);
}

static cv::Point2f ConvertToPixelPos(const Eigen::Vector2d& pos) {
  double scale = 100;
  Eigen::Vector2d scaled_pos = pos * scale;

  // field y positive axis is negative direction in pixel coodrinate
  scaled_pos.y() *= -1.0;

  // t_pw: translation from world to pixel coordinate
  Eigen::Vector2d t_pw = Eigen::Vector2d(0, 300);
  Eigen::Vector2d pixel_pos = scaled_pos + t_pw;

  return cv::Point2f(pixel_pos.x(), pixel_pos.y());
}

#endif
