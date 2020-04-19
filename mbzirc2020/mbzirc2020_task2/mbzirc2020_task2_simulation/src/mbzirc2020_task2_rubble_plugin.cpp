#include <gazebo/physics/World.hh>
#include "mbzirc2020_task2_rubble_plugin.h"

#if GAZEBO_MAJOR_VERSION >= 8
using namespace ignition;
using namespace gazebo;
#else
#include <gazebo/math/Rand.hh>
using namespace gazebo;
#endif

GZ_REGISTER_WORLD_PLUGIN(MBZIRC2020Task2RubblePlugin)

/////////////////////////////////////////////////
MBZIRC2020Task2RubblePlugin::MBZIRC2020Task2RubblePlugin()
{
}

/////////////////////////////////////////////////
void MBZIRC2020Task2RubblePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

#if GAZEBO_MAJOR_VERSION >= 8
  math::Vector3d bottomRight = _sdf->Get<math::Vector3d>("bottom_right");
  math::Vector3d topLeft = _sdf->Get<math::Vector3d>("top_left");
  math::Vector3d brickSize = _sdf->Get<math::Vector3d>("brick_size");
  math::Vector3d brickColor = _sdf->Get<math::Vector3d>("brick_color");
#else
  math::Vector3 bottomRight = _sdf->Get<math::Vector3>("bottom_right");
  math::Vector3 topLeft = _sdf->Get<math::Vector3>("top_left");
  math::Vector3 brickSize = _sdf->Get<math::Vector3>("brick_size");
  math::Vector3 brickColor = _sdf->Get<math::Vector3>("brick_color");
#endif
  double brickMass = _sdf->Get<double>("brick_mass");

  unsigned int count = _sdf->Get<unsigned int>("count");

  for (unsigned int i = 0; i < count; ++i)
  {
    Obj obj;

#if GAZEBO_MAJOR_VERSION >= 8
    obj.pose.Pos().X() = math::Rand::DblUniform(bottomRight.X(), topLeft.X());
    obj.pose.Pos().Y() = math::Rand::DblUniform(bottomRight.Y(), topLeft.Y());
    obj.pose.Pos().Z() = math::Rand::DblUniform(bottomRight.Z(), topLeft.Z());

    obj.pose.Rot().Euler(math::Vector3d(math::Rand::DblUniform(0.0, 3.1415),
                                         math::Rand::DblUniform(-0.1, 0.1),
                                         math::Rand::DblUniform(0.0, 3.1415)));

#else
    obj.pose.pos.x = math::Rand::GetDblUniform(bottomRight.x, topLeft.x);
    obj.pose.pos.y = math::Rand::GetDblUniform(bottomRight.y, topLeft.y);
    obj.pose.pos.z = math::Rand::GetDblUniform(bottomRight.z, topLeft.z);

    obj.pose.rot.SetFromEuler(math::Vector3(math::Rand::GetDblUniform(0.0, 3.1415),
                                            math::Rand::GetDblUniform(-0.1, 0.1),
                                            math::Rand::GetDblUniform(0.0, 3.1415)));
#endif
    obj.size = brickSize;

    // Make sure the bottom of the rubble piece is above the bottomRight.z
    // This will prevent ground penetration.
#if GAZEBO_MAJOR_VERSION >= 8
    if (obj.pose.Pos().Z() - obj.size.Z() * 0.5 < bottomRight.Z())
      obj.pose.Pos().Z() = bottomRight.Z() + obj.size.Z() * 0.5;
#else
    if (obj.pose.pos.z - obj.size.z * 0.5 < bottomRight.z)
      obj.pose.pos.z = bottomRight.z + obj.size.z * 0.5;
#endif

    std::ostringstream name;
    name << "rubble_" << i;
    this->MakeBox(name.str(), obj.pose, obj.size, brickColor, brickMass);
  }
}

/////////////////////////////////////////////////
void MBZIRC2020Task2RubblePlugin::Init()
{
}

/////////////////////////////////////////////////
#if GAZEBO_MAJOR_VERSION >= 8
void MBZIRC2020Task2RubblePlugin::MakeBox(const std::string &_name, math::Pose3d &_pose,
                                          math::Vector3d &_size, math::Vector3d &_color, double _mass)
#else
void MBZIRC2020Task2RubblePlugin::MakeBox(const std::string &_name, math::Pose &_pose,
                                          math::Vector3 &_size, math::Vector3 &_color, double _mass)
#endif
{
  std::ostringstream newModelStr;

#if GAZEBO_MAJOR_VERSION >= 8
  float sx = _size.X();
  float sy = _size.Y();
  float sz = _size.Z();
#else
  float sx = _size.x;
  float sy = _size.y;
  float sz = _size.z;
#endif

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    "<model name='" << _name << "'>"
    "<allow_auto_disable>true</allow_auto_disable>"
    "<pose>" << _pose << "</pose>"
    "<link name='link'>"
      "<velocity_decay>"
        "<linear>0.01</linear>"
        "<angular>0.01</angular>"
      "</velocity_decay>"
      "<inertial><mass>" << _mass << "</mass>"
        "<inertia>"
        "<ixx>" << (1.0/12.0) * _mass * (sy*sy + sz*sz) << "</ixx>"
        "<iyy>" << (1.0/12.0) * _mass * (sz*sz + sx*sx) << "</iyy>"
        "<izz>" << (1.0/12.0) * _mass * (sx*sx + sy*sy) << "</izz>"
        "<ixy>" << 0.0 << "</ixy>"
        "<ixz>" << 0.0 << "</ixz>"
        "<iyz>" << 0.0 << "</iyz>"
        "</inertia>"
      "</inertial>"
      "<collision name='collision'>"
        "<geometry>"
          "<box><size>" << _size << "</size></box>"
        "</geometry>"
      "</collision>"
      "<visual name='visual'>"
        "<geometry>"
          "<box><size>" << _size << "</size></box>"
        "</geometry>"
        "<material>"
          "<ambient>" << _color << " 1</ambient>"
          "<diffuse>" << _color << " 1</diffuse>"
          "<specular>0.1 0.1 0.1 1</specular>"
          "<emissive>0 0 0 0</emissive>"
        "</material>"
      "</visual>"
    "</link>"
  "</model>"
  "</sdf>";

  this->world->InsertModelString(newModelStr.str());
}
