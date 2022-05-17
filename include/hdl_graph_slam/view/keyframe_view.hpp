#ifndef HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP
#define HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP

#include <memory>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>

#include <g2o/types/slam3d/vertex_se3.h>
#include <hdl_graph_slam/view/vertex_view.hpp>
#include <hdl_graph_slam/interactive_keyframe.hpp>

namespace hdl_graph_slam {

class KeyFrameView : public VertexView {
public:
  using Ptr = std::shared_ptr<KeyFrameView>;

  KeyFrameView(const InteractiveKeyFrame::Ptr& kf)
    : VertexView(kf->node)
    , is_selected(false)
  {
    keyframe = kf;

    // std::cout << kf->id() << " : ";
    pointcloud_buffer.reset(new glk::PointCloudBuffer(kf->cloud));
  }

  InteractiveKeyFrame::Ptr lock() const { return keyframe.lock(); }

  virtual bool available() const override { return !keyframe.expired(); }

  bool is_ignore_keyframe(const Eigen::Vector4f &kf_pos, const long id)
  {
    // Todo: REPLACE pointcloud_buffer, add config for LOD
    Eigen::Vector4f view_position = this->view_mat * kf_pos;
    view_position[3] = 0;
    const float dist = view_position.norm();

    if (dist > 800) {
      pointcloud_buffer->set_stride_scale(10);
      return (id % 2 == 0 || id % 3 == 0|| id % 5 == 0|| id % 7 == 0);
    }
    
    if (dist > 400) {
      pointcloud_buffer->set_stride_scale(5);
      return (id % 2 == 0 || id % 3 == 0|| id % 5 == 0|| id % 7 == 0);
    }

    if (dist > 200)
    {
      pointcloud_buffer->set_stride_scale(2);
      return (id % 2 == 0 || id % 9 == 0);
    }

    if (dist > 100)
    {
      pointcloud_buffer->set_stride_scale(1);
      return (id % 3 == 0);
    }

    pointcloud_buffer->set_stride_scale(1);
    return false;
  }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
    if (this->is_selected) {
      return;
    }

    InteractiveKeyFrame::Ptr kf = keyframe.lock();
    Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();

    if (is_ignore_keyframe(model_matrix.rightCols(1), kf->id()))
    {
      return;
    }

    shader.set_uniform("color_mode", 0);
    shader.set_uniform("model_matrix", model_matrix);

    shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));
    pointcloud_buffer->draw(shader);

    if (!flags.draw_verticies || !flags.draw_keyframe_vertices) {
      return;
    }

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("material_color", this->color);

    draw_kf(kf, shader);
  }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader, const Eigen::Vector4f& select_color) {
    if (!is_selected || !available()) {
      return;
    }

    InteractiveKeyFrame::Ptr kf = keyframe.lock();
    Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();

    draw(flags, shader, select_color, kf->estimate().matrix().cast<float>());
  }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader, const Eigen::Vector4f& select_color, 
    const Eigen::Matrix4f& model_matrix) override {
    if (!is_selected || !available()) {
      return;
    }

    InteractiveKeyFrame::Ptr kf = keyframe.lock();

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("material_color", select_color);
    shader.set_uniform("model_matrix", model_matrix);

    shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));
    pointcloud_buffer->draw(shader);

    draw_kf(kf, shader);
  }

  void draw_kf(const InteractiveKeyFrame::Ptr& kf, glk::GLSLShader& shader)
  {
    shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));
    shader.set_uniform("apply_keyframe_scale", true);

    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);

    shader.set_uniform("apply_keyframe_scale", false);
  }

  void set_view_mat(const Eigen::Matrix4f &mat)
  {
    this->view_mat = mat;
  }

  void set_selected(const bool value) {
    this->is_selected = value;
  }

private:
  std::weak_ptr<InteractiveKeyFrame> keyframe;
  std::unique_ptr<glk::PointCloudBuffer> pointcloud_buffer;
  Eigen::Matrix4f view_mat;
  std::atomic<bool> is_selected;
};

}  // namespace hdl_graph_slam

#endif