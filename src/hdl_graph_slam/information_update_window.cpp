#include <hdl_graph_slam/information_update_window.hpp>

#include <functional>
#include <g2o/types/slam3d/edge_se3.h>
#include <glk/primitives/primitives.hpp>

#include <hdl_graph_slam/information_matrix_calculator.hpp>

namespace hdl_graph_slam {

InformationUpdateWindow::InformationUpdateWindow(std::shared_ptr<InteractiveGraphView>& graph) 
  : show_window(false)
  , apply_method(0)
  , graph(graph)
  , running(false)
  , inspected_edge(nullptr)
  , optimization_cycle(10)
  , optimization_count(0) 
  , position_inf(-1) 
  , rotation_inf(-1) 
  {}

InformationUpdateWindow::~InformationUpdateWindow() {
  running = false;
  if(refinement_thread.joinable()) {
    refinement_thread.join();
  }
}

void InformationUpdateWindow::draw_ui() {
  if(!show_window) {
    running = false;
    return;
  }

  ImGui::Begin("information update", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("Loop detection");
  const char* apply_methods[] = {"All", "Odom", "Gps", "Loop closure"};
  ImGui::Combo("Search method", &apply_method, apply_methods, IM_ARRAYSIZE(apply_methods));

  ImGui::Text("Scan matching");
  ImGui::DragFloat("Position information", &position_inf, 10, -1, 1e6f, "%.6f");
  ImGui::DragFloat("Rotation information", &rotation_inf, 10, -1, 1e6f, "%.6f");

  if(ImGui::Button("Apply to all SE3 edges")) {
    apply_information();
  }

  std::stringstream format;
  format << (optimization_count % optimization_cycle) << "/%d";
  ImGui::DragInt("Optimization cycle", &optimization_cycle, 1, 1, 1024, format.str().c_str());

  if(ImGui::Button("Start")) {
    if(!running) {
      running = true;
      refinement_thread = std::thread([this]() { refinement_task(); });
    }
  }

  ImGui::SameLine();
  if(ImGui::Button("Stop")) {
    if(running) {
      running = false;
      refinement_thread.join();
    }
  }

  if(running) {
    ImGui::SameLine();
    ImGui::Text("%c Running", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
  }

  ImGui::End();
}

void InformationUpdateWindow::apply_information() {
  if(running) {
    running = false;
    refinement_thread.join();
  }

  if (position_inf < 0 || rotation_inf < 0)
  {
    return;
  }

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  inf.block<3, 3>(0, 0) *= position_inf;
  inf.block<3, 3>(3, 3) *= rotation_inf;

  int appliedCount = 0;

  for(const auto& edge : edges) 
  {
    if (apply_method == 1 || apply_method == 3)
    {
      if (edge.isGps){
        continue;
      }

      g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge.edge);
      if(e == nullptr) {
        continue;
      }

      auto isLoopClosure = std::abs(e->vertex(0)->id() - e->vertex(1)->id()) > 1;

      if ((apply_method == 1 && !isLoopClosure) || (apply_method == 3 && isLoopClosure))
      {
        std::cout << e->vertex(0)->id() << " - " <<  e->vertex(1)->id() << ": " << isLoopClosure << std::endl;
        e->setInformation(inf);
        ++appliedCount;
      }
    }
    else if (apply_method == 0 || (edge.isGps && apply_method == 2))
    {
      edge.edge->setInformation(inf);
      ++appliedCount;
    }
  }

  std::cout << "appliedCount = " << appliedCount << std::endl;

}

void InformationUpdateWindow::refinement() {
  std::vector<double> accumulated_error(edges.size());
  accumulated_error[0] = edges[0].error();

  for(int i = 1; i < edges.size(); i++) {
    accumulated_error[i] = accumulated_error[i - 1] + edges[i].error();
  }

  std::uniform_real_distribution<> udist(0.0, accumulated_error.back());
  double roulette = udist(mt);

  auto loc = std::upper_bound(accumulated_error.begin(), accumulated_error.end(), roulette);
  size_t index = std::distance(accumulated_error.begin(), loc);

  auto& edge = edges[index];

  auto v1 = graph->keyframes.find(edge.begin);
  auto v2 = graph->keyframes.find(edge.end);

  if(v1 == graph->keyframes.end() || v2 == graph->keyframes.end()) {
    return;
  }

  double fitness_score_before = InformationMatrixCalculator::calc_fitness_score(v1->second->cloud, v2->second->cloud, edge.edge->measurement(), 2.0);

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration = registration_method.method();

  Eigen::Isometry3d relative = v1->second->estimate().inverse() * v2->second->estimate();

  registration->setInputTarget(v1->second->cloud);
  registration->setInputSource(v2->second->cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
  registration->align(*aligned, relative.matrix().cast<float>());

  relative.matrix() = registration->getFinalTransformation().cast<double>();
  double fitness_score_after = InformationMatrixCalculator::calc_fitness_score(v1->second->cloud, v2->second->cloud, relative, 2.0);

  if(fitness_score_after < fitness_score_before) {
    std::lock_guard<std::mutex> lock(graph->optimization_mutex);
    edge.edge->setMeasurement(relative);
    graph->apply_robust_kernel(edge.edge, robust_kernel.type(), robust_kernel.delta());

    if(((++optimization_count) % optimization_cycle) == 0) {
      graph->optimize();
    }
  }

  {
    std::lock_guard<std::mutex> lock(edges_mutex);
    std::sort(edges.begin(), edges.end(), std::greater<EdgeInfo>());
    edge.num_evaluations++;
    inspected_edge = edge.edge;
  }
}

void InformationUpdateWindow::refinement_task() {
  while(running) {
    refinement();
  }
}

void InformationUpdateWindow::draw_gl(glk::GLSLShader& shader) {
  if(!running) {
    return;
  }

  g2o::HyperGraph::Edge* edge = nullptr;
  {
    std::lock_guard<std::mutex> lock(edges_mutex);
    edge = inspected_edge;
  }

  auto found = graph->edges_view_map.find(edge);
  if(found == graph->edges_view_map.end()) {
    return;
  }

  const Eigen::Vector3f pt = found->second->representative_point();
  const auto& cone = glk::Primitives::instance()->primitive(glk::Primitives::CONE);

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("material_color", Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));

  auto trans = Eigen::Scaling<float>(1.0f, 1.0f, 1.5f) * Eigen::Translation3f(Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
  shader.set_uniform("model_matrix", (Eigen::Translation3f(pt + Eigen::Vector3f::UnitZ() * 0.1f) * trans).matrix());
  cone.draw(shader);
}

void InformationUpdateWindow::show() {
  show_window = true;

  edges.clear();
  for(const auto& edge : graph->graph->edges()) 
  {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge);

    if(e == nullptr) {
      continue;
    }

    auto info = EdgeInfo(e);

    auto id1 = e->vertex(0)->id();
    auto isGps = graph->gpsVertices.find(id1) != graph->gpsVertices.end() && graph->gpsVertices[id1];

    if (!isGps)
    {
      auto id2 = e->vertex(1)->id();
      isGps = graph->gpsVertices.find(id2) != graph->gpsVertices.end() && graph->gpsVertices[id2];
    }

    info.isGps = isGps;

    edges.push_back(std::move(info));
  }

  std::sort(edges.begin(), edges.end(), std::greater<EdgeInfo>());
}

void InformationUpdateWindow::close() {
  if(running) {
    running = false;
  }
  show_window = false;
  edges.clear();
}

}  // namespace hdl_graph_slam