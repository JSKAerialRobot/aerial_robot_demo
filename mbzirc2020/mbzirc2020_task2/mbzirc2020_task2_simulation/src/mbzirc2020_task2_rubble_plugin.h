#pragma once

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

#if GAZEBO_MAJOR_VERSION >= 8
using namespace ignition;
#endif


namespace gazebo
{
  class GAZEBO_VISIBLE MBZIRC2020Task2RubblePlugin : public WorldPlugin
  {
    /// \brief Constructor.
  public: MBZIRC2020Task2RubblePlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
  public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
  public: virtual void Init();

#if GAZEBO_MAJOR_VERSION >= 8
  private: void MakeBox(const std::string &_name, math::Pose3d &_pose,
                        math::Vector3d &_size, math::Vector3d &_color, double _mass);
#else
  private: void MakeBox(const std::string &_name, math::Pose &_pose,
                        math::Vector3 &_size, math::Vector3 &_color, double _mass);
#endif
  private: class Obj
           {
#if GAZEBO_MAJOR_VERSION >= 8
           public: math::Pose3d pose;
           public: math::Vector3d size;
#else
           public: math::Pose pose;
           public: math::Vector3 size;
#endif
  };

  private: physics::WorldPtr world;
  };
}
