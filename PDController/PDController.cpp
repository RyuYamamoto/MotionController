#ifndef _PD_CONTROLLER_
#define _PD_CONTROLLER_

#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/Link>
#include <cnoid/SimpleController>
#include <iostream>
#include <string>

class PDController : public cnoid::SimpleController
{
public:
  virtual bool initialize(cnoid::SimpleControllerIO * io) override
  {
    this->io = io;
    body_ = io->body();
    dt_ = io->timeStep();
    std::cout << body_->joint(0)->q() << std::endl;

    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
      cnoid::Link * joint = body_->joint(idx);
      joint->setActuationMode(cnoid::Link::JOINT_TORQUE);
      io->enableIO(joint);
      q_ref_.emplace_back(joint->q());
    }
    q_prev_ = q_ref_;

    loadGain("/usr/local/lib/choreonoid-1.8/simplecontroller/jaxon_pd_gain.csv");

    return true;
  }

  void loadGain(const std::string filename)
  {
    std::ifstream ifs(filename);
    std::string line;
    std::string delimeter;

    while (getline(ifs, line)) {
      std::istringstream i_stream(line);

      std::vector<double> pd_line;
      while (getline(i_stream, delimeter, ',')) { pd_line.push_back(std::atof(delimeter.c_str())); }
      pd_gain_.push_back(std::make_pair(pd_line[0], pd_line[1]));
    }
    for (const auto p : pd_gain_) {}
  }
  virtual bool control() override
  {
    std::cout << body_->numJoints() << std::endl;
    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
      cnoid::Link * joint = body_->joint(idx);
      const double q = joint->q();
      const double dq = (q - q_prev_[idx]) / dt_;
      const double u = (q_ref_[idx] - q) * pd_gain_[idx].first + (0 - dq) * pd_gain_[idx].second;
      q_prev_[idx] = q;
      joint->u() = u;
    }
    return true;
  }

public:
  cnoid::Body * body_;
  cnoid::SimpleControllerIO * io;

  std::vector<double> q_ref_;
  std::vector<double> q_prev_;
  std::vector<std::pair<double, double>> pd_gain_;

  double dt_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PDController)

#endif
