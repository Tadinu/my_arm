#include <QVector3D>
#include "Rviz/VMarker.h"
#include <boost/iterator/iterator_concepts.hpp>
#include "KsGlobal.h"

//#include "Voxelyze/include/VX_MeshRender.h"

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>

// Reference:
// http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started
// http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls
// https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/src/basic_controls.cpp

#define CMARKER_NAME ("ball")
#define CMARKER_BASE_FRAME (CWORLD_FRAME) //CBASE_LINK

VMarker* VMarker::_instance = nullptr;
QMutex VMarker::_markerMutex;
bool VMarker::_isInitialized = false;
bool VMarker::_isVoxelMeshUpdated = false;
std::vector<CVertex> VMarker::gbVoxelMeshVertices;
std::vector<CLine> VMarker::gbVoxelMeshEdges;
std::vector<CFacet> VMarker::gbVoxelMeshFaces;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> VMarker::_server = nullptr;
std::vector<tf::Vector3> gb_cloud_cube_positions;

void VMarker::frameCallback(const ros::TimerEvent&)
{
    // Be careful, this may be invoked on RobotThread worker thread!
    // If VMARKER_INSTANCE() is initialized on RobotThread thread, so it is okay.
    // Otherwise, a mutex installation is required!
#if 0
    static uint32_t counter = 0;

    static tf::TransformBroadcaster br;

    tf::Transform t;

    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
    t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br.sendTransform(tf::StampedTransform(t, time, CMARKER_BASE_FRAME, "moving_frame"));

    t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
    br.sendTransform(tf::StampedTransform(t, time, CMARKER_BASE_FRAME, "rotating_frame"));

    counter++;
#endif
}

void VMarker::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    // --------------------------------------------------------------------
    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            static tf::Vector3 lastPos(0,0,0);
            if(feedback->marker_name == CMARKER_NAME) {
                if(lastPos.x() != feedback->pose.position.x ||
                   lastPos.y() != feedback->pose.position.y ||
                   lastPos.z() != feedback->pose.position.z
                ) {
                    if(VMARKER_INSTANCE()->checkPosLimit(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z)))
                    {
                        lastPos.setX(feedback->pose.position.x);
                        lastPos.setY(feedback->pose.position.y);
                        lastPos.setZ(feedback->pose.position.z);

                        VMARKER_INSTANCE()->emitMarkerPosChanged(QVector3D(lastPos.x(), lastPos.y(), lastPos.z()));
                    }
                    else {
                        VMARKER_INSTANCE()->setMarkerPos(feedback->marker_name.c_str(), lastPos);
                    }
                }
            }
#if 0
            // -------------------------------------------------------------
            //compute difference vector for this cube

            tf::Vector3 fb_pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
            unsigned index = atoi( feedback->marker_name.c_str() );

            if ( index > gb_cloud_cube_positions.size() )
            {
                return;
            }
            tf::Vector3 fb_delta = fb_pos - gb_cloud_cube_positions[index];

            // move all markers in that direction
            for ( unsigned i=0; i< gb_cloud_cube_positions.size(); i++ )
            {
                float d = fb_pos.distance( gb_cloud_cube_positions[i] );
                float t = 1 / (d*5.0+1.0) - 0.2;
                if ( t < 0.0 ) t=0.0;

                gb_cloud_cube_positions[i] += t * fb_delta;

                if ( i == index ) {
                    ROS_INFO_STREAM( d );
                    gb_cloud_cube_positions[i] = fb_pos;
                }

                std::stringstream s;
                s << i;
                VMARKER_INSTANCE()->setMarkerPos(s.str(), gb_cloud_cube_positions[i]);
            }
#endif
            break;
        }
    }
    // -------------------------------------------------------------------
    // LOGGING --
#if 0
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if(feedback->mouse_point_valid)
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", "   << feedback->mouse_point.y
                       << ", "   << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type)
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
        break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
        break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM( s.str() << ": pose changed"
                << "\nposition = "
                << feedback->pose.position.x
                << ", " << feedback->pose.position.y
                << ", " << feedback->pose.position.z
                << "\norientation = "
                << feedback->pose.orientation.w
                << ", " << feedback->pose.orientation.x
                << ", " << feedback->pose.orientation.y
                << ", " << feedback->pose.orientation.z
                << "\nframe: " << feedback->header.frame_id
                << " time: " << feedback->header.stamp.sec << "sec, "
                << feedback->header.stamp.nsec << " nsec" );
        break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
        break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
        break;
    }
#endif
    _server->applyChanges();
}

VMarker* VMarker::getInstance()
{
    if(_instance == nullptr) {
        _instance = new VMarker();
    }

    return _instance;
}

void VMarker::deleteInstace()
{
    delete _instance;
    _instance = nullptr;
}

bool VMarker::checkInstance()
{
    return _instance != nullptr;
}

VMarker::VMarker():
         _pos_limit(10, 10, 100),
         _static_markers(std::vector<visualization_msgs::Marker>(MARKER_ID_TOTAL)) { // Don't use std::vector::reserve!!!
    // Don't call VMarker::initialize(); here!!!
    //_visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(CWORLD_FRAME, "/moveit_visual_markers"));
    _visual_tools.reset(new rviz_visual_tools::RvizVisualTools("rh_forearm" /* CWORLD_FRAME */, "/rviz_visual_markers"));
    ROS_INFO("Sleeping 5 seconds before running demo");
    //ros::Duration(5.0).sleep();

    // Clear messages
    _visual_tools->deleteAllMarkers();
    _visual_tools->enableBatchPublishing();
}

VMarker::~VMarker()
{
    _server.reset();
    _static_markers.clear();
}

void VMarker::setStaticMarkerProperties(int markerId,
                                        const char* header_frame,
                                        uint32_t marker_shape_type,
                                        const tf::Vector3& scale,
                                        const tf::Vector3& pos,
                                        const tf::Quaternion& orient)
{
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    _static_markers[markerId].header.frame_id = header_frame;
    _static_markers[markerId].header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    _static_markers[markerId].ns = "basic_shapes";
    _static_markers[markerId].id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    _static_markers[markerId].type = marker_shape_type;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    _static_markers[markerId].pose.position.x = pos.x();
    _static_markers[markerId].pose.position.y = pos.y();
    _static_markers[markerId].pose.position.z = pos.z();

#if 0
    // geometry_msgs::Quaternion <- tf::Quaternion
    _static_markers[markerId].pose.orientation.x = orient.x();
    _static_markers[markerId].pose.orientation.y = orient.y();
    _static_markers[markerId].pose.orientation.z = orient.z();
    _static_markers[markerId].pose.orientation.w = orient.w();
#endif

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    _static_markers[markerId].scale.x = scale.x();
    _static_markers[markerId].scale.y = scale.y();
    _static_markers[markerId].scale.z = scale.z();

    // Set the color -- be sure to set alpha to something non-zero!
    _static_markers[markerId].color.r = 0.0f;
    _static_markers[markerId].color.g = 1.0f;
    _static_markers[markerId].color.b = 0.0f;
    _static_markers[markerId].color.a = 1.0;

    _server->applyChanges();
}

void VMarker::createInteractiveMarkers()
{
    // -----------------------------------------------------------------------------------------------
    // MAKE SOME MAKERS AT INITIALIZATION --
    //
    // Make Marker Interactive Controls --
    //
    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, true, _static_markers[TARGET_BALL]);

    // Make Cube Cloud --
    //
    //makeCubeCloud();

    // Make CantileverBeam
    //
    //makeCantileverBeam();
#if 0
    tf::Vector3 position;
    position = tf::Vector3( 0, 3, 0);
    make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE,  true, makeMarker() );

    position = tf::Vector3( 3, 3, 0);
    makeRandomDofMarker(  makeMarker());
    position = tf::Vector3(-3, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D,  false,makeMarker() );
    position = tf::Vector3( 0, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,  true,makeMarker() );
    position = tf::Vector3( 3, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D,  false,makeMarker() );
    position = tf::Vector3(-3,-3, 0);
    makeViewFacingMarker( makeMarker() );
    position = tf::Vector3( 0,-3, 0);
    makeQuadrocopterMarker( makeMarker() );
    position = tf::Vector3( 3,-3, 0);
    makeChessPieceMarker( makeMarker() );
    position = tf::Vector3(-3,-6, 0);
    makePanTiltMarker( makeMarker() );
    position = tf::Vector3( 0,-6, 0);
    makeMovingMarker( makeMarker());
    position = tf::Vector3( 3,-6, 0);
    makeMenuMarker( makeMarker() );
    position = tf::Vector3( 0,-9, 0);
    makeButtonMarker( makeMarker() );
#endif
    _server->applyChanges();
}

void VMarker::initialize()
{
    VMarker::_isInitialized = true;
    // -------------------------------------------------------------------------------------------
    // Static Markers --
    //
    for(size_t i = 0; i < MARKER_ID_TOTAL; i++) {
        _static_markers[i].header.frame_id = CWORLD_FRAME;
        _static_markers[i].header.stamp    = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        _static_markers[i].ns = "basic_shapes";
        _static_markers[i].id = 0;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        _static_markers[i].action = visualization_msgs::Marker::ADD;

        _static_markers[i].lifetime = ros::Duration();
    }

    // Interactive Markers --
    //
    _server.reset(new interactive_markers::InteractiveMarkerServer("my_arm","",false));
    //_marker.header.frame_id = "/base_link";
    //
    ros::Duration(0.1).sleep();

    // create a timer to update the published transforms
    //ros::Timer frame_timer = node_handler.createTimer(ros::Duration(0.01), &VMarker::frameCallback);
    //
    _menu_handler.insert("First Entry" , &VMarker::processFeedback);
    _menu_handler.insert("Second Entry", &VMarker::processFeedback);
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = _menu_handler.insert("Submenu");
    _menu_handler.insert(sub_menu_handle, "First Entry",  &VMarker::processFeedback);
    _menu_handler.insert(sub_menu_handle, "Second Entry", &VMarker::processFeedback);
}

// Static Marker --
//
void VMarker::setStaticMarker(int markerId, const visualization_msgs::Marker& marker)
{
    if(markerId >= 0 && markerId < MARKER_ID_TOTAL) {
        _static_markers[markerId] = marker;
    }
}

visualization_msgs::Marker& VMarker::getStaticMarker(int markerId)
{
    assert(markerId >= 0 && markerId < MARKER_ID_TOTAL);
    return _static_markers[markerId];
}

tf::Vector3 VMarker::getStaticMarkerPos(int markerId)
{
    if(markerId >= 0 && markerId < MARKER_ID_TOTAL) {
        return tf::Vector3(_static_markers[markerId].pose.position.x, _static_markers[markerId].pose.position.y, _static_markers[markerId].pose.position.z);
    }
    else {
        return tf::Vector3();
    }
}

void VMarker::setStaticMarkerPos(int markerId, const tf::Vector3& pos)
{
    if(markerId >= 0 && markerId < MARKER_ID_TOTAL) {
        _static_markers[markerId].pose.position.x = pos.x();
        _static_markers[markerId].pose.position.y = pos.y();
        _static_markers[markerId].pose.position.z = pos.z();
    }
}

void VMarker::moveStaticMarkerPos(int markerId, const tf::Vector3& distance)
{
    if(markerId >= 0 && markerId < MARKER_ID_TOTAL) {
        _static_markers[markerId].pose.position.x += distance.x();
        _static_markers[markerId].pose.position.y += distance.y();
        _static_markers[markerId].pose.position.z += distance.z();
    }
}

// Interactive Marker --
//
tf::Vector3 VMarker::getInteractiveMarkerPos()
{
    visualization_msgs::InteractiveMarker int_marker;
    _server->get(CMARKER_NAME, int_marker);
    return tf::Vector3(int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z);
}

void VMarker::setInteractiveMarkerPos(const tf::Vector3& pos)
{
    visualization_msgs::InteractiveMarker int_marker;
    _server->get(CMARKER_NAME, int_marker);

    if(checkPosLimit(pos)) {
        int_marker.pose.position.x = pos.x();
        int_marker.pose.position.y = pos.y();
        int_marker.pose.position.z = pos.z();

        _server->setPose(CMARKER_NAME, int_marker.pose);
        _server->applyChanges();
    }
}

void VMarker::moveInteractiveMarkerPos(const tf::Vector3& distance)
{
    visualization_msgs::InteractiveMarker int_marker;
    _server->get(CMARKER_NAME, int_marker);

    tf::Vector3 new_pos(int_marker.pose.position.x + distance.x(),
                       int_marker.pose.position.y + distance.y(),
                       int_marker.pose.position.z + distance.z());
    if(checkPosLimit(new_pos)) {
        int_marker.pose.position.x = new_pos.x();
        int_marker.pose.position.y = new_pos.y();
        int_marker.pose.position.z = new_pos.z();

        _server->setPose(CMARKER_NAME, int_marker.pose);
        _server->applyChanges();
    }
    else {
        ROS_WARN("Marker enters restricted zone!");
    }
}

void VMarker::setMarkerPos(const std::string& marker_name, const tf::Vector3& pos)
{
    geometry_msgs::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();

    _server->setPose(marker_name, pose);
    _server->applyChanges();
}

void VMarker::alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    geometry_msgs::Pose pose = feedback->pose;

    pose.position.x = round(pose.position.x-0.5)+0.5;
    pose.position.y = round(pose.position.y-0.5)+0.5;

    ROS_INFO_STREAM( feedback->marker_name << ":"
        << " aligning position = "
        << feedback->pose.position.x
        << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z
        << " to "
        << pose.position.x
        << ", " << pose.position.y
        << ", " << pose.position.z );

    _server->setPose( feedback->marker_name, pose );
    _server->applyChanges();
}

void VMarker::saveMarker(const visualization_msgs::InteractiveMarker& int_marker)
{
    _server->insert(int_marker);
    _server->setCallback(int_marker.name, &VMarker::processFeedback);
}

void VMarker::make6DofMarker( bool fixed, unsigned int interaction_mode, bool show_6dof,
                              const visualization_msgs::Marker& marker)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = marker.header.frame_id;
    int_marker.pose.position   = marker.pose.position;

    int_marker.scale = 1;

    int_marker.name = CMARKER_NAME;
    int_marker.description = "Simple 6-DOF Control";

    // insert a box
    makeMarkerControl(int_marker, marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    visualization_msgs::InteractiveMarkerControl control;

    if ( fixed )
    {
      int_marker.description += "\n(fixed orientation)";
      control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }

    _server->insert(int_marker);
    _server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        _menu_handler.apply( *_server, int_marker.name );

    ROS_INFO_ONCE("Make6DOF Marker Done");
}
// %EndTag(6DOF)%

// %Tag(RandomDof)%
void VMarker::makeRandomDofMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "6-DOF\n(Arbitrary Axes)";

  makeMarkerControl(int_marker, marker);

  visualization_msgs::InteractiveMarkerControl control;

  for ( int i=0; i<3; i++ )
  {
    control.orientation.w = KsGlobal::rand(-1,1);
    control.orientation.x = KsGlobal::rand(-1,1);
    control.orientation.y = KsGlobal::rand(-1,1);
    control.orientation.z = KsGlobal::rand(-1,1);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(RandomDof)%


// %Tag(ViewFacing)%
void VMarker::makeViewFacingMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "View Facing 6-DOF";

  visualization_msgs::InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back(marker);
  control.always_visible = true;

  int_marker.controls.push_back(control);

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(ViewFacing)%


// %Tag(Quadrocopter)%
void VMarker::makeQuadrocopterMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "Quadrocopter";

  makeMarkerControl(int_marker, marker);

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Quadrocopter)%

// %Tag(ChessPiece)%
void VMarker::makeChessPieceMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  _server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}
// %EndTag(ChessPiece)%

// %Tag(PanTilt)%
void VMarker::makePanTiltMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "Pan / Tilt";

  makeMarkerControl(int_marker, marker);

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(PanTilt)%

// %Tag(Menu)%
void VMarker::makeMenuMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "Context Menu\n(Right Click)";

  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
  _menu_handler.apply( *_server, int_marker.name );
}
// %EndTag(Menu)%

// %Tag(Button)%
void VMarker::makeButtonMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "Button\n(Left Click)";

  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Button)%

// %Tag(Moving)%
void VMarker::makeMovingMarker(const visualization_msgs::Marker& marker)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker.header.frame_id;
  int_marker.pose.position   = marker.pose.position;
  int_marker.scale = 1;

  int_marker.name = CMARKER_NAME;
  int_marker.description = "Marker Attached to a\nMoving Frame";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

visualization_msgs::Marker VMarker::makeStaticMarker(int marker_type)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = CMARKER_BASE_FRAME;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = marker_type;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}

#if 0
visualization_msgs::Marker VMarker::makeVoxelyzeMarker(int marker_type)
{
    // Create a pure static marker
    visualization_msgs::Marker voxelyzeMarker = makeStaticMarker(marker_type);

    // Create a voxelyze
    CVoxelyze Vx(0.05); //5mm voxels
    CVX_Material* pMaterial = Vx.addMaterial(1000000, 1000); //A material with stiffness E=1MPa and density 1000Kg/m^3
    CVX_Voxel* Voxel1 = Vx.setVoxel(pMaterial, 0, 0, 0); //Voxel at index x=0, y=0. z=0
    Voxel1->external()->setFixedAll(); //Fixes all 6 degrees of freedom with an external condition
    for (int i=0; i<100; i++) Vx.doTimeStep(); //simulates 100 timesteps

    // Assign the voxelyze properties to the static marker
    voxelyzeMarker.pose.position.x = Voxel1->position().x;
    voxelyzeMarker.pose.position.y = Voxel1->position().y;
    voxelyzeMarker.pose.position.z = Voxel1->position().z;

    voxelyzeMarker.pose.orientation.x = Voxel1->orientation().x;
    voxelyzeMarker.pose.orientation.y = Voxel1->orientation().y;
    voxelyzeMarker.pose.orientation.z = Voxel1->orientation().z;
    voxelyzeMarker.pose.orientation.w = Voxel1->orientation().w;

    return voxelyzeMarker;
}
#endif

#if 0
void VMarker::makeCantileverBeam()
{
    CVoxelyze Vx(0.5); //500mm voxels
    CVX_Material* pMaterial = Vx.addMaterial(1000000, 1000); //A material with stiffness E=1MPa and density 1000Kg/m^3
    std::vector<CVX_Voxel*> voxelList; voxelList.reserve(3);
    voxelList.push_back(Vx.setVoxel(pMaterial, 0, 0, 0)); //Voxel at index x=0, y=0. z=0
    voxelList.push_back(Vx.setVoxel(pMaterial, 1, 0, 0));
    voxelList.push_back(Vx.setVoxel(pMaterial, 2, 0, 0)); //Beam extends in the +X direction

    voxelList[0]->external()->setFixedAll(); //Fixes all 6 degrees of freedom with an external condition on Voxel 1
    voxelList[2]->external()->setForce(0, 0, -1); //pulls Voxel 3 downward with 1 Newton of force.

    for (int i=0; i<100; i++) Vx.doTimeStep(); //simulate  100 timesteps.

    // Create a visual marker based on meshRender data
    //
    CVX_MeshRender voxelMeshRender(&Vx);
    voxelMeshRender.generateMesh();
    gbVoxelMeshVertices = voxelMeshRender.getVertices();
    gbVoxelMeshEdges    = voxelMeshRender.getEdges();
    // -------------------------------------------------------------------------------------
#if 0
    for(int i = 0; i < voxelList.size(); i++) {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = CMARKER_BASE_FRAME;
        int_marker.scale = 0.1;

        int_marker.pose.position.x    = voxelList[i]->position().x;
        int_marker.pose.position.y    = voxelList[i]->position().y;
        int_marker.pose.position.z    = voxelList[i]->position().z;

        int_marker.pose.orientation.x = voxelList[i]->orientation().x;
        int_marker.pose.orientation.y = voxelList[i]->orientation().y;
        int_marker.pose.orientation.z = voxelList[i]->orientation().z;
        int_marker.pose.orientation.w = voxelList[i]->orientation().w;

        std::stringstream s;
        s << i;
        int_marker.name = s.str();

        makeBoxControl(int_marker);

        _server->insert(int_marker);
        _server->setCallback(int_marker.name, &processFeedback);
    }
#endif
}
#endif

void VMarker::publishVisualArrows()
{
#if 1
    // ----------------------------------------------------------------------------------------------
    // Create pose
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

    // Publish arrow vector of pose
    //ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    _visual_tools->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
    // Don't forget to trigger the publisher!
    _visual_tools->trigger();
#endif
}

void VMarker::setVoxelMeshInfo(std::vector<CVertex> vers,
                               std::vector<CLine> edges,
                               std::vector<CFacet> faces) {
    _markerMutex.lock();
    gbVoxelMeshVertices.swap(vers);
    gbVoxelMeshEdges.swap(edges);
    gbVoxelMeshFaces.swap(faces);
    setVoxelMeshUpdated();
    _markerMutex.unlock();
}

void VMarker::publishVoxelMesh(bool isShaded)
{
    //return;
    QMutexLocker lock(&_markerMutex);
    // ----------------------------------------------------------------------------------------------
    if(!VMarker::_isVoxelMeshUpdated)
        return;
    // ----------------------------------------------------------------------------------------------
    // DELETE CURRRENT MARKERS
    _visual_tools->deleteAllMarkers();
    // LOAD INTERNAL MARKER PUBLISHER, WAITING FOR THE SUBSCRIBER
    //
    _visual_tools->loadMarkerPub(true, true); // (publisher.getNumSubscribers() > 0)
    // ----------------------------------------------------------------------------------------------
    // START PUBLISHING MARKERS
    //
    VMarker::_isVoxelMeshUpdated = false;
    //ROS_INFO("PUBLISH VOXELS %d %d %d",(int)gbVoxelMeshFaces.size(),
    //                                   (int)gbVoxelMeshEdges.size(),
    //                                   (int)gbVoxelMeshVertices.size());

//#define PUBLISH_WHOLE_MESH
    // SHADED --
#ifdef PUBLISH_WHOLE_MESH
    geometry_msgs::Pose pose;
    _visual_tools->publishMesh(pose, "Voxel.stl");
#else
    //
    isShaded = false;
    if(isShaded) {
#if 0
        // FACES --
        geometry_msgs::Polygon polygon;
        geometry_msgs::Point32 point;
        for (int i = 0; i < (int)gbVoxelMeshFaces.size(); i++) {
            for (int j = 0; j < 3; j++) {
                CVertex& CurVert = gbVoxelMeshVertices[gbVoxelMeshFaces[i].vi[j]]; //just a local reference for readability
                //(CurVert.VColor.r, CurVert.VColor.g, CurVert.VColor.b);
                //
                point.x = CurVert.v.x + CurVert.DrawOffset.x;
                point.y = CurVert.v.y + CurVert.DrawOffset.y;
                point.z = CurVert.v.z + CurVert.DrawOffset.z;
                //
                polygon.points.push_back(point);
                //
                _visual_tools->publishPolygon(polygon,  rviz_visual_tools::LIME_GREEN);
            }
            polygon.points.clear();
        }
#endif
        // EDGES --
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;
        for (int i = 0; i < (int)gbVoxelMeshEdges.size(); i++) {
            for (int j=0; j<2; j++) {
                CVertex& CurVert1 = gbVoxelMeshVertices[gbVoxelMeshEdges[i].vi[0]];
                CVertex& CurVert2 = gbVoxelMeshVertices[gbVoxelMeshEdges[i].vi[1]];

                // Point 1
                point1.x = CurVert1.v.x + CurVert1.DrawOffset.x;
                point1.y = CurVert1.v.y + CurVert1.DrawOffset.y;
                point1.z = CurVert1.v.z + CurVert1.DrawOffset.z;

                // Point 2
                point2.x = CurVert2.v.x + CurVert2.DrawOffset.x;
                point2.y = CurVert2.v.y + CurVert2.DrawOffset.y;
                point2.z = CurVert2.v.z + CurVert2.DrawOffset.z;
                //
                _visual_tools->publishLine(point1, point2, rviz_visual_tools::LIME_GREEN);
            }
        }
    }

    // WIREFRAME
    else {
        geometry_msgs::Point point;
        std::vector<geometry_msgs::Point> path;

        for (int i = 0; i < (int)gbVoxelMeshFaces.size(); i++) {
            //glNormal3d(Facets[i].n.x, Facets[i].n.y, Facets[i].n.z);
            for (int j = 0; j < 3; j++) {

                CVertex& CurVert = gbVoxelMeshVertices[gbVoxelMeshFaces[i].vi[j]]; //just a local reference for readability
                //glColor3d(CurVert.VColor.r, CurVert.VColor.g, CurVert.VColor.b);

                // Point 1
                point.x = CurVert.v.x + CurVert.DrawOffset.x;
                point.y = CurVert.v.y + CurVert.DrawOffset.y;
                point.z = CurVert.v.z + CurVert.DrawOffset.z;
                path.push_back(point);
            }
        }
        _visual_tools->publishPath(path, rviz_visual_tools::LIME_GREEN, 0.0001);
    }
#endif // #ifdef PUBLISH_WHOLE_MESH else

    // ----------------------------------------------------------------------------------------------
    // Don't forget to trigger the publisher!
    _visual_tools->trigger();
}

visualization_msgs::InteractiveMarkerControl&
VMarker::makeMarkerControl(visualization_msgs::InteractiveMarker &msg, const visualization_msgs::Marker& marker)
{
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);
    msg.controls.push_back( control );

    return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl& VMarker::makeBoxControl(visualization_msgs::InteractiveMarker &msg )
{
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.independent_marker_orientation = true;

    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale;
    marker.scale.y = msg.scale;
    marker.scale.z = msg.scale;
    marker.color.r = 0.65+0.7*msg.pose.position.x;
    marker.color.g = 0.65+0.7*msg.pose.position.y;
    marker.color.b = 0.65+0.7*msg.pose.position.z;
    marker.color.a = 1.0;

    control.markers.push_back( marker );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void VMarker::makeCubeCloud()
{
    int side_length = 10;
    float step = 1.0/ (float)side_length;
    int count = 0;

    gb_cloud_cube_positions.reserve( side_length*side_length*side_length );

    for ( double x=-0.5; x<0.5; x+=step )
    {
        for ( double y=-0.5; y<0.5; y+=step )
        {
            for ( double z=0.0; z<1.0; z+=step )
            {
                visualization_msgs::InteractiveMarker int_marker;
                int_marker.header.frame_id = CMARKER_BASE_FRAME;
                int_marker.scale = step;

                int_marker.pose.position.x = x;
                int_marker.pose.position.y = y;
                int_marker.pose.position.z = z;

                gb_cloud_cube_positions.push_back( tf::Vector3(x,y,z) );

                std::stringstream s;
                s << count;
                int_marker.name = s.str();

                makeBoxControl(int_marker);

                _server->insert(int_marker);
                _server->setCallback(int_marker.name, &processFeedback);

                count++;
            }
        }
    }
}
