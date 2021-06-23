// Copyright 2013 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


/*
 * \file  gazebo_ros_video.cpp
 *
 * \brief Video plugin for displaying ROS image topics on Ogre textures
 *
 * \author Piyush Khandelwal
 *
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_colour.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <memory>
#include <string>

namespace gazebo_plugins
{
/**
 * Helper class for GazeboRosVideo
 * The class deals with the image conversions required for display on Gazebo
 * Ogre texture
 **/
class ColourVisual : public gazebo::rendering::Visual
{
public:
  /// Constructor
  ColourVisual(
    const std::string & name, gazebo::rendering::VisualPtr parent,
    int height, int width);

  /// Destructor
  virtual ~ColourVisual();

  /// Resize image and copy image data into pixel buffer.
  /// \param[in] image Subscribed image
  void render(float r, float g, float b);

private:
  /// Instance of Ogre Texture manager
  Ogre::TexturePtr texture_;

  /// Height of image to be displayed
  int height_;

  /// Width of image to be displayed
  int width_;
};

ColourVisual::ColourVisual(
  const std::string & name,
  gazebo::rendering::VisualPtr parent, int height,
  int width)
: gazebo::rendering::Visual(name, parent), height_(height), width_(width)
{
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name + "__ColourTexture__",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_BYTE_BGRA,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
    name + "__ColourMaterial__", "General");
  material->getTechnique(0)->getPass(0)->createTextureUnitState(
    name + "__ColourTexture__");
  material->setReceiveShadows(false);
  material->setSelfIllumination(1.0, 1.0, 1.0);
  material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

  double factor = 1.0;

  Ogre::ManualObject mo(name + "__ColourObject__");
  mo.begin(name + "__ColourMaterial__", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  mo.position(-factor / 2, factor / 2, 0.51);
  mo.textureCoord(0, 0);

  mo.position(factor / 2, factor / 2, 0.51);
  mo.textureCoord(1, 0);

  mo.position(factor / 2, -factor / 2, 0.51);
  mo.textureCoord(1, 1);

  mo.position(-factor / 2, -factor / 2, 0.51);
  mo.textureCoord(0, 1);

  mo.triangle(0, 3, 2);
  mo.triangle(2, 1, 0);
  mo.end();

  mo.convertToMesh(name + "__ColourMesh__");

  Ogre::MovableObject * obj =
    (Ogre::MovableObject *) GetSceneNode()->getCreator()->createEntity(
    name + "__ColourEntity__", name + "__ColourMesh__");
  obj->setCastShadows(false);
  AttachObject(obj);
}

ColourVisual::~ColourVisual() {}
void ColourVisual::render(float r, float g, float b)
{
  // Get the pixel buffer
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox & pixelBox = pixelBuffer->getCurrentLock();
  auto * pDest = static_cast<uint32_t *>(pixelBox.data);
  int i = 0;
  //float bright = 0.3 * r + 0.59 * g + 0.11 * b;
  float bright = fmax(r, fmax(g, b));
  uint32_t col = ((uint32_t)(r * 255) << 16) | ((uint32_t)(g * 255) << 8) | ((uint32_t)(b * 255));
  for(int y = 0; y < height_; y++)
  {
    float dy = (y - (height_ * 0.5)) / (height_ * 0.5);
    dy *= dy;
    for(int x = 0; x < width_; x++)
    {
        float dx = (x - (width_ * 0.5)) / (width_ * 0.5);
        float alpha = (1 - (dy + dx * dx)) * bright;
        alpha = alpha < 0 ? 0 : alpha;
        uint32_t a = (uint8_t)(alpha * 255) << (uint32_t)24;
        pDest[i++] = col | a;
    }
  }
  // Unlock the pixel buffer
  pixelBuffer->unlock();
}

class GazeboRosColourPrivate
{
public:
  /// Callback when a image is received.
  /// \param[in] _msg Image command message.
  void setColour(const std_msgs::msg::ColorRGBA::ConstSharedPtr msg);

  /// Callback to be called at every simulation iteration.
  void onUpdate();

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// A shared_ptr to instance of Video Visual
  std::shared_ptr<ColourVisual> colour_visual_;

  /// Flag variable set on arrival of a new colour
  bool new_colour_available_;

  // Received colour value
  float r_, g_, b_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr rosnode_;

  /// Subscriber to images
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr colour_subscriber_;
};

// Constructor
GazeboRosColour::GazeboRosColour()
: impl_(std::make_unique<GazeboRosColourPrivate>())
{
}

// Destructor
GazeboRosColour::~GazeboRosColour()
{
  impl_->update_connection_.reset();
  impl_->rosnode_.reset();
}

void GazeboRosColour::Load(
  gazebo::rendering::VisualPtr _parent,
  sdf::ElementPtr _sdf)
{
  impl_->rosnode_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->rosnode_->get_qos();

  int height = 64;

  int width = 64;

  impl_->r_ = 0.0;
  impl_->g_ = 0.0;
  impl_->b_ = 0.0;

  impl_->colour_visual_ = std::make_shared<ColourVisual>(
    _parent->Name() + "::colour_visual::" + _sdf->Get<std::string>("name"), 
    _parent, height, width);
  _parent->GetScene()->AddVisual(impl_->colour_visual_);

  // Subscribe to the image topic
  impl_->colour_subscriber_ =
    impl_->rosnode_->create_subscription<std_msgs::msg::ColorRGBA>(
    "colour", qos.get_subscription_qos("color", rclcpp::QoS(1)),
    std::bind(&GazeboRosColourPrivate::setColour, impl_.get(), std::placeholders::_1));

  impl_->new_colour_available_ = false;

  impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
    std::bind(&GazeboRosColourPrivate::onUpdate, impl_.get()));

  RCLCPP_INFO(
    impl_->rosnode_->get_logger(),
    "GazeboRosColour has started. Subscribed to [%s]",
    impl_->colour_subscriber_->get_topic_name());
}

void GazeboRosColourPrivate::onUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosColourPrivate::onUpdate");
#endif
  //if (new_colour_available_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("render");
#endif
    colour_visual_->render(r_, g_, b_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  //}
  new_colour_available_ = false;
}

void GazeboRosColourPrivate::setColour(const std_msgs::msg::ColorRGBA::ConstSharedPtr msg)
{
  r_ = msg->r;
  g_ = msg->g;
  b_ = msg->b;
  new_colour_available_ = true;
}
GZ_REGISTER_VISUAL_PLUGIN(GazeboRosColour)
}  // namespace gazebo_plugins
