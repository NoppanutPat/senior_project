#include <algorithm>
#include <assert.h>
#include <math.h>

#include <quad_sim/gazebo_ros_force.h>

namespace gazebo 
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForce::GazeboRosForce()
{
  std::vector<double> temp={0,0,0,0};
  this->rotorvelo.data = temp;
  this->liftforce.data = temp;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForce::~GazeboRosForce()
{
  this->update_connection_.reset();

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("propeller1_link"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <propeller1_link>, cannot proceed");
    return;
  }
  else
    this->link1_name_ = _sdf->GetElement("propeller1_link")->Get<std::string>();

  this->link1_ = _model->GetLink(this->link1_name_);

  if (!_sdf->HasElement("propeller2_link"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <propeller2_link>, cannot proceed");
    return;
  }
  else
    this->link2_name_ = _sdf->GetElement("propeller2_link")->Get<std::string>();

  this->link2_ = _model->GetLink(this->link2_name_);

  if (!_sdf->HasElement("propeller3_link"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <propeller3_link>, cannot proceed");
    return;
  }
  else
    this->link3_name_ = _sdf->GetElement("propeller3_link")->Get<std::string>();

  this->link3_ = _model->GetLink(this->link3_name_);


  if (!_sdf->HasElement("propeller4_link"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <propeller4_link>, cannot proceed");
    return;
  }
  else
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

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions throttle = ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
    this->topic_name_,1,
    boost::bind( &GazeboRosForce::UpdateObjectForce,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(throttle);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosForce::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosForce::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateObjectForce(const std_msgs::Float64MultiArray::ConstPtr& throttle)
{
  
  std::vector<double> tempvelo;
  std::vector<double> templiftforce;
  double velo,force;
  for(int i=0;i<4;i++){
    if (throttle->data[i] >= 0){
      velo = -0.75+47.3*(throttle->data[i])+(-0.281*(throttle->data[i]*throttle->data[i]))+(9.97E-04*(throttle->data[i]*throttle->data[i]*throttle->data[i]));
      force = -5.38E-03+(0.0478*throttle->data[i])+(8.93E-04*throttle->data[i]*throttle->data[i])+(-2.34E-06*throttle->data[i]*throttle->data[i]*throttle->data[i]);
    }
    else{
      velo = -1*(-0.75-47.3*(throttle->data[i])+(-0.281*(throttle->data[i]*throttle->data[i]))-(9.97E-04*(throttle->data[i]*throttle->data[i]*throttle->data[i])));
      force = -5.38E-03-(0.0478*throttle->data[i])+(8.93E-04*throttle->data[i]*throttle->data[i])-(-2.34E-06*throttle->data[i]*throttle->data[i]*throttle->data[i]);
    }

    ROS_FATAL_NAMED("force","force : %lf , velo : %lf\n",force,velo);

    tempvelo.push_back(velo);
    templiftforce.push_back(force);
  }
  
  this->rotorvelo.data = tempvelo;
  this->liftforce.data = templiftforce;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateChild()
{
  this->lock_.lock();

  ignition::math::Vector3d velo1(0,0,this->rotorvelo.data[0]);
  ignition::math::Vector3d velo2(0,0,this->rotorvelo.data[1]);
  ignition::math::Vector3d velo3(0,0,this->rotorvelo.data[2]);
  ignition::math::Vector3d velo4(0,0,this->rotorvelo.data[3]);
  
  ignition::math::Vector3d liftforce1(0,this->liftforce.data[0],0);
  ignition::math::Vector3d liftforce2(0,this->liftforce.data[1],0);
  ignition::math::Vector3d liftforce3(0,this->liftforce.data[2],0);
  ignition::math::Vector3d liftforce4(0,this->liftforce.data[3],0);

  // this->link_->AddRelativeForce(force);
  // // this->link_->AddForceAtRelativePosition(force,ignition::math::Vector3d(0,0,0));
  // // this->link_->AddForceAtWorldPosition(this->link_->GetWorldPose().rot.RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->GetWorldCoGPose().pos);
  // this->link_->AddRelativeTorque(torque);

  this->link1_->SetAngularVel(velo1);
  this->link2_->SetAngularVel(velo2);
  this->link3_->SetAngularVel(velo3);
  this->link4_->SetAngularVel(velo4);

  this->link1_->AddRelativeForce(liftforce1);
  this->link2_->AddRelativeForce(liftforce2);
  this->link3_->AddRelativeForce(liftforce3);
  this->link4_->AddRelativeForce(liftforce4);
  this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosForce::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}