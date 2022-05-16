#include <glk/pointcloud_buffer.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

PointCloudBuffer::PointCloudBuffer(const std::string& cloud_filename) {
}

PointCloudBuffer::PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {

  stride_scale = 1;
  stride = sizeof(pcl::PointXYZI);
  num_points = cloud->size();
  // std::cout << "num_points " << num_points << ", stride " << stride << std::endl;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, cloud->size() * stride, cloud->points.data(), GL_STATIC_DRAW);
}

PointCloudBuffer::~PointCloudBuffer() {
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void PointCloudBuffer::draw(glk::GLSLShader& shader) {
  if(num_points == 0) {
    return;
  }
  static int cc = 0;

  GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, stride * stride_scale, 0);

  glDrawArrays(GL_POINTS, 0, num_points / stride_scale);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(0);
}

}  // namespace glk
