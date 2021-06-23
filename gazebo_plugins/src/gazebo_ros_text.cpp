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
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_text.hpp>
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
    class TextVisual : public gazebo::rendering::Visual
    {
    public:
        /// Constructor
        TextVisual(
            const std::string &name, gazebo::rendering::VisualPtr parent,
            int height, int width);

        /// Destructor
        virtual ~TextVisual();

        /// Resize image and copy image data into pixel buffer.
        /// \param[in] image Subscribed image
        void render(std::string text);

    private:
        /// Instance of Ogre Texture manager
        Ogre::TexturePtr texture_;

        /// Height of image to be displayed
        int height_;

        /// Width of image to be displayed
        int width_;
    };

    TextVisual::TextVisual(
        const std::string &name,
        gazebo::rendering::VisualPtr parent, int height,
        int width)
        : gazebo::rendering::Visual(name, parent), height_(height), width_(width)
    {
        texture_ = Ogre::TextureManager::getSingleton().createManual(
            name + "__TextTexture__",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_BYTE_BGRA,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
            name + "__TextMaterial__", "General");
        material->getTechnique(0)->getPass(0)->createTextureUnitState(
            name + "__TextTexture__");
        material->setReceiveShadows(false);
        //material->setSelfIllumination(1.0, 1.0, 1.0);
        material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

        double factor = 1.0;

        Ogre::ManualObject mo(name + "__TextObject__");
        mo.begin(name + "__TextMaterial__", Ogre::RenderOperation::OT_TRIANGLE_LIST);

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

        mo.convertToMesh(name + "__TextMesh__");

        Ogre::MovableObject *obj =
            (Ogre::MovableObject *)GetSceneNode()->getCreator()->createEntity(
                name + "__TextEntity__", name + "__TextMesh__");
        obj->setCastShadows(false);
        AttachObject(obj);

        gazebo::rendering::MovableText *text = new gazebo::rendering::MovableText();
        text->Load("__TEXT_OBJECT__", "testing..", "Arial", 2.0);
        text->SetShowOnTop(true);

        //Ogre::SceneNode *textNode = GetSceneNode()->createChildSceneNode("__TEXT_NODE__");
        //textNode->attachObject(text);
        //AttachObject(text);

    }

    TextVisual::~TextVisual() {}
    void TextVisual::render(std::string text)
    {
        // Get the pixel buffer
        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

        // Lock the pixel buffer and get a pixel box
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();
        auto *pDest = static_cast<uint32_t *>(pixelBox.data);
        int i = 0;
        //float bright = 1.0;
        //uint32_t col = 0xffffff;
        for (int y = 0; y < height_; y++)
        {
            for (int x = 0; x < width_; x++)
            {
                pDest[i++] = (x<<16) | y;
            }
        }
        // Unlock the pixel buffer
        pixelBuffer->unlock();
    }

    class GazeboRosTextPrivate
    {
    public:
        /// Callback when a image is received.
        /// \param[in] _msg Image command message.
        void setText(const std_msgs::msg::String::ConstSharedPtr msg);

        /// Callback to be called at every simulation iteration.
        void onUpdate();

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        /// A shared_ptr to instance of Video Visual
        std::shared_ptr<TextVisual> text_visual_;

        /// Flag variable set on arrival of new text
        bool new_text_available_;

        // Received colour value
        std::string text_;

        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr rosnode_;

        /// Subscriber to images
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_subscriber_;
    };

    // Constructor
    GazeboRosText::GazeboRosText()
        : impl_(std::make_unique<GazeboRosTextPrivate>())
    {
    }

    // Destructor
    GazeboRosText::~GazeboRosText()
    {
        impl_->update_connection_.reset();
        impl_->rosnode_.reset();
    }

    void GazeboRosText::Load(
        gazebo::rendering::VisualPtr _parent,
        sdf::ElementPtr _sdf)
    {
        impl_->rosnode_ = gazebo_ros::Node::Get(_sdf);

        // Get QoS profiles
        const gazebo_ros::QoS &qos = impl_->rosnode_->get_qos();

        int height = 256;

        int width = 256;

        impl_->text_ = "";

        impl_->text_visual_ = std::make_shared<TextVisual>(
            _parent->Name() + "::text_visual::" + _sdf->Get<std::string>("name"),
            _parent, height, width);
        _parent->GetScene()->AddVisual(impl_->text_visual_);

        // Subscribe to the image topic
        impl_->text_subscriber_ =
            impl_->rosnode_->create_subscription<std_msgs::msg::String>(
                "text", qos.get_subscription_qos("text", rclcpp::QoS(1)),
                std::bind(&GazeboRosTextPrivate::setText, impl_.get(), std::placeholders::_1));

        impl_->new_text_available_ = false;

        impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
            std::bind(&GazeboRosTextPrivate::onUpdate, impl_.get()));

        RCLCPP_INFO(
            impl_->rosnode_->get_logger(),
            "GazeboRosText has started. Subscribed to [%s]",
            impl_->text_subscriber_->get_topic_name());
    }

    void GazeboRosTextPrivate::onUpdate()
    {
        text_visual_->render(text_);
        new_text_available_ = true;
    }

    void GazeboRosTextPrivate::setText(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        text_ = msg->data;
        new_text_available_ = true;
    }
    GZ_REGISTER_VISUAL_PLUGIN(GazeboRosText)
} // namespace gazebo_plugins
