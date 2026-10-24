#pragma once

#include "PointCloud.h"
#include "RenderSettings.h"
#include <GL/glew.h>
#include <Eigen/Core>

class PointCloudRenderer {
public:
  using ColorMode = slstudio::gui::RenderSettings::ColorMode;
  PointCloudRenderer();
  ~PointCloudRenderer();
  PointCloudRenderer(const PointCloudRenderer &) = delete;
  PointCloudRenderer &operator=(const PointCloudRenderer &) = delete;
  void setCloud(PointCloudConstPtr cloud);
  GLuint render(int width, int height, const slstudio::gui::RenderSettings &settings);
  void resetView();
  void zoom(float steps);
  void pan(float deltaX, float deltaY, float viewportHeight);
  bool setRotationCenter(float imageX, float imageY);
  bool hasVisiblePixels() const;
  float yaw = 0.0f, pitch = 0.0f, distance = 3.0f;

private:
  PointCloudConstPtr displayedCloud;
  GLuint program = 0, vertices = 0, array = 0;
  GLuint gridPositions = 0, gridPixels = 0;
  GLuint framebuffer = 0, texture = 0, depth = 0;
  int width = 0, height = 0;
  GLsizei count = 0;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector2f panOffset = Eigen::Vector2f::Zero();
  Eigen::Vector3f rotationCenter = Eigen::Vector3f::Zero();
  Eigen::Matrix3f orientation = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  float radius = 1.0f, minimumDepth = 0.0f, maximumDepth = 1.0f;
};