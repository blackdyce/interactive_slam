#pragma once

#include <hdl_graph_slam/robust_kernels.hpp>

namespace hdl_graph_slam {
struct EdgeInfo {
public:
  EdgeInfo(g2o::EdgeSE3* edge);

  bool operator<(const EdgeInfo& rhs) const;

  bool operator>(const EdgeInfo& rhs) const;

  double error() const;

  double score() const;

  void update();

public:
  g2o::EdgeSE3* edge;

  int begin;
  int end;
  int num_evaluations;
  bool isGps;
};
}