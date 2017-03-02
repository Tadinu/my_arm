#include "Gazebo/gazebo_marker_plugin.h"

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboMarkerPlugin::GazeboMarkerPlugin():
      line(nullptr)
    {

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GazeboMarkerPlugin::~GazeboMarkerPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void GazeboMarkerPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      this->visual_ = _parent;

      this->visual_namespace_ = "visual/";

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      this->rosnode_ = new ros::NodeHandle(this->visited_visual_namespace_);
      this->force_sub_ = this->rosnode_->subscribe("/some_force", 1000, &GazeboMarkerPlugin::VisualizeForceOnLink, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&GazeboMarkerPlugin::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void GazeboMarkerPlugin::UpdateChild()
    {
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void GazeboMarkerPlugin::VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
    {
      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      //TODO: Get the current link position
      link_pose = CurrentLinkPose();
      //TODO: Get the current end position
      endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

      // Add two points to a connecting line strip from link_pose to endpoint
      this->line->AddPoint(
        math::Vector3(
          link_pose.position.x,
          link_pose.position.y,
          link_pose.position.z
          )
        );
      this->line->AddPoint(math::Vector3(endpoint.x, endpoint.y, endpoint.z));
      // set the Material of the line, in this case to purple
      this->line->setMaterial("Gazebo/Purple");
      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(GazeboMarkerPlugin)
  }
}
