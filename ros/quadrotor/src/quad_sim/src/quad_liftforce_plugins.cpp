#include <algorithm>
#include <assert.h>
#include <math.h>
#include <ros/console.h>


#include <quad_sim/quad_liftforce_plugins.h>

namespace gazebo 
{
GZ_REGISTER_MODEL_PLUGIN(Quad_forcelift);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Quad_forcelift::Quad_forcelift()
{ 
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Quad_forcelift::~Quad_forcelift()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Quad_forcelift::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  if (_sdf->HasElement("propeller1_link"))
    this->link1_name_ = _sdf->GetElement("propeller1_link")->Get<std::string>();

  this->link1_ = _model->GetLink(this->link1_name_);

  if (_sdf->HasElement("propeller2_link"))
    this->link2_name_ = _sdf->GetElement("propeller2_link")->Get<std::string>();

  this->link2_ = _model->GetLink(this->link2_name_);

  if (_sdf->HasElement("propeller3_link"))
    this->link3_name_ = _sdf->GetElement("propeller3_link")->Get<std::string>();

  this->link3_ = _model->GetLink(this->link3_name_);


  if (_sdf->HasElement("propeller4_link"))
    this->link4_name_ = _sdf->GetElement("propeller4_link")->Get<std::string>();

  this->link4_ = _model->GetLink(this->link4_name_);

  // if (_sdf->HasElement("xyzOffset"))
  //   this->xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<std::vector>();
  // else
  //   ROS_FATAL_NAMED("force", "Please specify a xyzOffset.");

  if (!this->link1_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link1_name_.c_str());
    return;
  }

  if (!this->link2_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link2_name_.c_str());
    return;
  }

  if (!this->link3_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link3_name_.c_str());
    return;
  }

  if (!this->link4_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link4_name_.c_str());
    return;
  }

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Quad_forcelift::OnUpdate()
{
  
  std::vector<double> templiftforce;

  double link1_velo,link2_velo,link3_velo,link4_velo;

  // link1_velo = this->link1_->RelativeAngularVel()[2];
  // link2_velo = this->link2_->RelativeAngularVel()[2];
  // link3_velo = this->link3_->RelativeAngularVel()[2];
  // link4_velo = this->link4_->RelativeAngularVel()[2];

  link1_velo = 2000.0;
  link2_velo = 2000.0;
  link3_velo = 2000.0;
  link4_velo = 2000.0;

  double link_velo[] = {link1_velo,link2_velo,link3_velo,link4_velo};
  
  double velo,force;
  for(int i=0;i<4;i++){
    if (link_velo[i] == 0){
      force = 0;
    }
    else{
      force = 2.63E-05 + (6.29E-06*link_velo[i]) + (1.33E-07*(link_velo[i]*link_velo[i])) + (6.63E-13*(link_velo[i]*link_velo[i]*link_velo[i]));
    }
    ROS_FATAL_NAMED("force","force : %lf\n",force);
    // gzdbg << "force : " << force << std::endl;

    templiftforce.push_back(force);
  }
  
  ignition::math::Vector3d liftforce1(0,templiftforce[0],0);
  ignition::math::Vector3d liftforce2(0,templiftforce[1],0);
  ignition::math::Vector3d liftforce3(0,templiftforce[2],0);
  ignition::math::Vector3d liftforce4(0,templiftforce[3],0);

  this->link1_->AddRelativeForce(liftforce1);
  this->link2_->AddRelativeForce(liftforce2);
  this->link3_->AddRelativeForce(liftforce3);
  this->link4_->AddRelativeForce(liftforce4);

}

}