/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: main.cc
*
*          Created On: Wed Jul 15 23:08:45 2020
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include <iostream>
#include <glog/logging.h>

#include "ParticleFilter.h"

DEFINE_bool(display, true, "Do display particles for debug?");
DEFINE_int32(num, 50, "number of particles");

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = google::WARNING;
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::cout << "Usage: " << argv[0]
            << " -num [particles number] -display [true/false]" << std::endl;

  int num = FLAGS_num;
  bool display = FLAGS_display;

  std::shared_ptr<ParticleFilter> particle_filter = std::make_shared<ParticleFilter>(num, display);

  particle_filter->Run();
  return 0;
}
