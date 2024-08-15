#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include "g2o_optimization/edge_project_line.h"

// monocular line
EdgeSE3ProjectLine::EdgeSE3ProjectLine()
    : g2o::BaseBinaryEdge<2, Eigen::Vector4d, VertexLine3D, VertexVIPose>() {}

bool EdgeSE3ProjectLine::read(std::istream &is) {
  g2o::internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeSE3ProjectLine::write(std::ostream &os) const {
  g2o::internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE3ProjectLine::computeError() {
  const VertexVIPose *v1 =
      static_cast<const VertexVIPose *>(_vertices[1]);
  const VertexLine3D *v2 = static_cast<const VertexLine3D *>(_vertices[0]);

  Eigen::Vector4d obs(_measurement);
  Eigen::Vector3d line_2d = cam_project(g2o::internal::fromSE3Quat(v1->estimate().Tcw) * v2->estimate());

  double line_2d_norm = line_2d.head(2).norm();
  Eigen::Vector2d error;
  error(0) = obs(0) * line_2d(0) + obs(1) * line_2d(1) + line_2d(2);
  error(1) = obs(2) * line_2d(0) + obs(3) * line_2d(1) + line_2d(2);
  _error = error / line_2d_norm;
}

Eigen::Vector3d EdgeSE3ProjectLine::cam_project(const g2o::Line3D& line) const {
  Eigen::Vector3d w = line.w();
  Eigen::Vector3d line_2d;
  line_2d(0) = fy * w(0);
  line_2d(1) = fx * w(1);
  line_2d(2) = Kv.transpose() * w;

  return line_2d;
}


// stereo line
EdgeStereoSE3ProjectLine::EdgeStereoSE3ProjectLine()
    : g2o::BaseBinaryEdge<4, Vector8d, VertexLine3D, VertexVIPose>() {}

bool EdgeStereoSE3ProjectLine::read(std::istream &is) {
  g2o::internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeStereoSE3ProjectLine::write(std::ostream &os) const {
  g2o::internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeStereoSE3ProjectLine::computeError() {
  const VertexVIPose *v1 =
      static_cast<const VertexVIPose *>(_vertices[1]);
  const VertexLine3D *v2 = static_cast<const VertexLine3D *>(_vertices[0]);
  Vector8d obs(_measurement);
  Eigen::Vector4d error;
  g2o::Isometry3 T_left = g2o::internal::fromSE3Quat(v1->estimate().Tcw);
  Eigen::Vector3d line_2d_left = cam_project(T_left * v2->estimate());
  double line_2d_norm_left = line_2d_left.head(2).norm();
  error(0) = (obs(0) * line_2d_left(0) + obs(1) * line_2d_left(1) + line_2d_left(2)) / line_2d_norm_left;
  error(1) = (obs(2) * line_2d_left(0) + obs(3) * line_2d_left(1) + line_2d_left(2)) / line_2d_norm_left;

  g2o::Isometry3 T_right(T_left);
  T_right(0, 3) -= b;

  Eigen::Vector3d line_2d_right = cam_project(T_right * v2->estimate());
  double line_2d_norm_right = line_2d_right.head(2).norm();
  error(2) = (obs(4) * line_2d_right(0) + obs(5) * line_2d_right(1) + line_2d_right(2)) / line_2d_norm_right;
  error(3) = (obs(6) * line_2d_right(0) + obs(7) * line_2d_right(1) + line_2d_right(2)) / line_2d_norm_right;
  _error = error;
}

Eigen::Vector3d EdgeStereoSE3ProjectLine::cam_project(const g2o::Line3D& line) const {
  Eigen::Vector3d w = line.w();
  Eigen::Vector3d line_2d;
  line_2d(0) = fy * w(0);
  line_2d(1) = fx * w(1);
  line_2d(2) = Kv.transpose() * w;
  return line_2d;
}