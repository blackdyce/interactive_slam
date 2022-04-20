#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <random>
#include <boost/optional.hpp>

#include <imgui.h>
#include <hdl_graph_slam/robust_kernels.hpp>
#include <hdl_graph_slam/registration_methods.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>
#include <hdl_graph_slam/edge_data.hpp>

namespace g2o {
class EdgeSE3;
}

namespace hdl_graph_slam {

class InformationUpdateWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InformationUpdateWindow(std::shared_ptr<InteractiveGraphView>& graph);
  ~InformationUpdateWindow();

  void draw_ui();

  void draw_gl(glk::GLSLShader& shader);

  void show();

  void start();

  void close();

private:
  void apply_robust_kernel();
  void apply_information();

  void refinement();
  void refinement_task();

private:
  bool show_window;
  std::shared_ptr<InteractiveGraphView>& graph;
  std::vector<EdgeInfo> edges;

  int apply_method; // all, odom, gps

  float position_inf;
  float rotation_inf;

  std::mt19937 mt;
  std::atomic_bool running;
  std::thread refinement_thread;

  std::mutex edges_mutex;
  g2o::EdgeSE3* inspected_edge;

  RegistrationMethods registration_method;
  RobustKernels robust_kernel;

  int optimization_cycle;
  int optimization_count;
};

}  // namespace hdl_graph_slam