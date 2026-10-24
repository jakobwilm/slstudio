#include "CloudRenderer.h"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
GLuint compileShader(GLenum type, const char *source) {
  const auto shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint compiled = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (!compiled) {
    char log[2048]{};
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    glDeleteShader(shader);
    throw std::runtime_error(log);
  }
  return shader;
}
}

PointCloudRenderer::PointCloudRenderer() {
  const char *vertexSource = R"(#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in ivec2 gridPixel;
uniform mat4 transform;
uniform mat3 normalTransform;
uniform sampler2D gridPositions;
uniform bool surfaceShading;
uniform vec3 center;
uniform float radius;
uniform float pointSize;
uniform int colorMode;
uniform vec3 solidColor;
uniform vec2 depthRange;
out vec3 vertexColor;
vec3 neighbor(ivec2 offset) {
  ivec2 pixel = gridPixel + offset;
  if (any(lessThan(pixel, ivec2(0))) || any(greaterThanEqual(pixel, textureSize(gridPositions, 0)))) return vec3(0);
  vec4 samplePosition = texelFetch(gridPositions, pixel, 0);
  if (samplePosition.w == 0.0 || abs(samplePosition.z - position.z) > max(abs(position.z) * 0.02, 0.0001)) return vec3(0);
  return samplePosition.xyz - position;
}
vec3 tangent(vec3 forward, vec3 backward) {
  if (dot(forward, forward) == 0.0) return -backward;
  if (dot(backward, backward) == 0.0) return forward;
  return forward - backward;
}
void main() {
  gl_Position = transform * vec4((position - center) / radius, 1.0);
  gl_PointSize = pointSize;
  float amount = clamp((position.z - depthRange.x) / max(depthRange.y - depthRange.x, 0.001), 0.0, 1.0);
  vertexColor = colorMode == 2 ? solidColor :
      colorMode == 1 ? mix(vec3(0.1, 0.75, 0.65), vec3(1.0, 0.65, 0.2), amount) : color;
  if (surfaceShading && all(greaterThanEqual(gridPixel, ivec2(0)))) {
    vec3 horizontal = tangent(neighbor(ivec2(1, 0)), neighbor(ivec2(-1, 0)));
    vec3 vertical = tangent(neighbor(ivec2(0, 1)), neighbor(ivec2(0, -1)));
    vec3 normal = normalTransform * cross(horizontal, vertical);
    if (dot(normal, normal) > 1e-20) {
      normal = normalize(normal);
      if (normal.z < 0.0) normal = -normal;
      float lighting = 0.3 + 0.7 * max(dot(normal, normalize(vec3(-0.4, 0.5, 1.0))), 0.0);
      vertexColor *= lighting;
    }
  }
})";
  const char *fragmentSource = R"(#version 330 core
in vec3 vertexColor;
out vec4 outputColor;
void main() {
  vec2 offset = gl_PointCoord - vec2(0.5);
  if (dot(offset, offset) > 0.25) discard;
  outputColor = vec4(vertexColor, 1.0);
})";
  const auto vertex = compileShader(GL_VERTEX_SHADER, vertexSource);
  GLuint fragment = 0;
  try { fragment = compileShader(GL_FRAGMENT_SHADER, fragmentSource); }
  catch (...) { glDeleteShader(vertex); throw; }
  program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, fragment);
  glLinkProgram(program);
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  GLint linked = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &linked);
  if (!linked) {
    glDeleteProgram(program);
    throw std::runtime_error("Point cloud shader program failed to link");
  }
  glGenVertexArrays(1, &array);
  glGenBuffers(1, &vertices);
  glGenBuffers(1, &gridPixels);
  glGenTextures(1, &gridPositions);
  glGenFramebuffers(1, &framebuffer);
  glGenTextures(1, &texture);
  glGenRenderbuffers(1, &depth);
}

PointCloudRenderer::~PointCloudRenderer() {
  glDeleteTextures(1, &gridPositions);
  glDeleteBuffers(1, &gridPixels);
  glDeleteRenderbuffers(1, &depth);
  glDeleteTextures(1, &texture);
  glDeleteFramebuffers(1, &framebuffer);
  glDeleteBuffers(1, &vertices);
  glDeleteVertexArrays(1, &array);
  glDeleteProgram(program);
}

void PointCloudRenderer::resetView() {
  yaw = pitch = 0.0f;
  distance = 3.0f;
  panOffset.setZero();
  rotationCenter.setZero();
}

void PointCloudRenderer::zoom(float steps) {
  distance = std::clamp(distance * std::exp(-steps * 0.1f), 0.01f, 30.0f);
}

void PointCloudRenderer::pan(float deltaX, float deltaY, float viewportHeight) {
  panOffset += Eigen::Vector2f(deltaX, -deltaY) *
      (2.0f * distance * std::tan(0.7f * 0.5f) / std::max(1.0f, viewportHeight));
}

bool PointCloudRenderer::setRotationCenter(float imageX, float imageY) {
  if (!count || width <= 0 || height <= 0 || !(imageX >= 0 && imageX < 1 && imageY >= 0 && imageY < 1)) return false;
  const int column = static_cast<int>(imageX * width);
  const int row = height - 1 - static_cast<int>(imageY * height);
  float depthValue = 1;
  glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
  glReadPixels(column, row, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depthValue);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  if (!std::isfinite(depthValue) || depthValue >= 1) return false;
  const Eigen::Vector4f clip(2.0f * (column + 0.5f) / width - 1,
      2.0f * (row + 0.5f) / height - 1, 2.0f * depthValue - 1, 1);
  const Eigen::Vector4f position = transform.inverse() * clip;
  Eigen::Vector3f selected = position.head<3>() / position.w();
  if (!selected.allFinite()) return false;
  const Eigen::Vector3f world = selected * radius + center;
  float nearest = std::numeric_limits<float>::infinity();
  for (const auto &point : displayedCloud->points) {
    const Eigen::Vector3f measured(point.x, point.y, point.z);
    const float separation = (measured - world).squaredNorm();
    if (separation < nearest) {
      nearest = separation;
      selected = (measured - center) / radius;
    }
  }
  if (!std::isfinite(nearest)) return false;
  const Eigen::Vector3f shift = orientation * (selected - rotationCenter);
  panOffset.setZero();
  distance -= shift.z();
  rotationCenter = selected;
  return true;
}

void PointCloudRenderer::setCloud(PointCloudConstPtr cloud) {
  count = 0;
  displayedCloud.reset();
  if (!cloud || cloud->points.empty()) return;
  if (cloud->points.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()))
    throw std::runtime_error("Point cloud exceeds renderer capacity");
  auto minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max()).eval();
  auto maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest()).eval();
  for (const auto &point : cloud->points) {
    const Eigen::Vector3f position(point.x, point.y, point.z);
    minimum = minimum.cwiseMin(position);
    maximum = maximum.cwiseMax(position);
  }
  center = (minimum + maximum) * 0.5f;
  radius = std::max((maximum - minimum).norm() * 0.5f, 0.001f);
  minimumDepth = minimum.z();
  maximumDepth = maximum.z();
  std::vector<cv::Vec2i> pixels(cloud->points.size(), cv::Vec2i(-1, -1));
  const auto &indices = cloud->pixelIndices;
  GLint maximumTextureSize = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximumTextureSize);
  if (indices.cols > maximumTextureSize || indices.rows > maximumTextureSize)
    throw std::runtime_error("Organized point cloud exceeds GPU texture dimensions");
  cv::Mat4f positions(std::max(1, indices.rows), std::max(1, indices.cols), cv::Vec4f(0, 0, 0, 0));
  for (int row = 0; row < indices.rows; ++row)
    for (int column = 0; column < indices.cols; ++column) {
      const int index = indices(row, column);
      if (index < 0) continue;
      if (static_cast<std::size_t>(index) >= cloud->points.size())
        throw std::runtime_error("Invalid organized point-cloud index");
      const auto &point = cloud->points[index];
      pixels[index] = cv::Vec2i(column, row);
      positions(row, column) = cv::Vec4f(point.x, point.y, point.z, 1);
    }
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, gridPositions);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, positions.cols, positions.rows, 0, GL_RGBA, GL_FLOAT, positions.data);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);
  count = static_cast<GLsizei>(cloud->points.size());
  glBindVertexArray(array);
  glBindBuffer(GL_ARRAY_BUFFER, vertices);
  glBufferData(GL_ARRAY_BUFFER, cloud->points.size() * sizeof(PointXYZRGB), cloud->points.data(), GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PointXYZRGB), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(PointXYZRGB),
                         reinterpret_cast<void *>(offsetof(PointXYZRGB, r)));
  glBindBuffer(GL_ARRAY_BUFFER, gridPixels);
  glBufferData(GL_ARRAY_BUFFER, pixels.size() * sizeof(cv::Vec2i), pixels.data(), GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(2);
  glVertexAttribIPointer(2, 2, GL_INT, sizeof(cv::Vec2i), nullptr);
  glBindVertexArray(0);
  displayedCloud = std::move(cloud);
}

GLuint PointCloudRenderer::render(int newWidth, int newHeight, const slstudio::gui::RenderSettings &settings) {
  newWidth = std::clamp(newWidth, 1, 8192);
  newHeight = std::clamp(newHeight, 1, 8192);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
  if (width != newWidth || height != newHeight) {
    width = newWidth;
    height = newHeight;
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      throw std::runtime_error("Point cloud framebuffer is incomplete");
    }
  }
  glViewport(0, 0, width, height);
  glDisable(GL_SCISSOR_TEST);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glClearColor(0.065f, 0.075f, 0.085f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  const float aspect = static_cast<float>(width) / height;
  Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
  const float focal = 1.0f / std::tan(0.7f * 0.5f);
  projection(0, 0) = focal / aspect;
  projection(1, 1) = focal;
  const float nearPlane = std::min(0.01f, distance * 0.1f);
  projection(2, 2) = -(100.0f + nearPlane) / (100.0f - nearPlane);
  projection(2, 3) = -2.0f * 100.0f * nearPlane / (100.0f - nearPlane);
  projection(3, 2) = -1.0f;
  Eigen::Affine3f view = Eigen::Affine3f::Identity();
  view.translate(Eigen::Vector3f(panOffset.x(), panOffset.y(), -distance));
  view.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));
  view.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY()));
  view.scale(Eigen::Vector3f(1, -1, -1));
  orientation = view.linear();
  view.translate(-rotationCenter);
  transform = projection * view.matrix();
  glUseProgram(program);
  glUniformMatrix4fv(glGetUniformLocation(program, "transform"), 1, GL_FALSE, transform.data());
  glUniform3fv(glGetUniformLocation(program, "center"), 1, center.data());
  glUniform1f(glGetUniformLocation(program, "radius"), radius);
  glUniform1f(glGetUniformLocation(program, "pointSize"), settings.pointSize);
  glUniform1i(glGetUniformLocation(program, "colorMode"), static_cast<int>(settings.colorMode));
  glUniform3fv(glGetUniformLocation(program, "solidColor"), 1, settings.solidColor.val);
  glUniform1i(glGetUniformLocation(program, "surfaceShading"), settings.surfaceShading);
  glUniformMatrix3fv(glGetUniformLocation(program, "normalTransform"), 1, GL_FALSE, orientation.data());
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, gridPositions);
  glUniform1i(glGetUniformLocation(program, "gridPositions"), 0);
  glUniform2f(glGetUniformLocation(program, "depthRange"), minimumDepth, maximumDepth);
  glBindVertexArray(array);
  glDrawArrays(GL_POINTS, 0, count);
  glBindVertexArray(0);
  glUseProgram(0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_DEPTH_TEST);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  return texture;
}

bool PointCloudRenderer::hasVisiblePixels() const {
  std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  std::size_t bright = 0;
  for (std::size_t index = 0; index < pixels.size(); index += 3)
    if (pixels[index] > 35 || pixels[index + 1] > 35 || pixels[index + 2] > 35) ++bright;
  return bright > 100;
}