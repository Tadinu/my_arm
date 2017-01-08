#ifndef ___VMARKER_H___
#define ___VMARKER_H___

#include <QtCore>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "KsGlobal.h"

class VMarker : public QObject {
    Q_OBJECT
public:
    static VMarker* getInstance();
    virtual ~VMarker();

    void initialize();
    void initializeMarker(const char* header_frame,
                          uint32_t marker_shape_type,
                          const tf::Vector3& scale = tf::Vector3(1.0f, 1.0f, 1.0f),
                          const tf::Vector3& pos = tf::Vector3(0,0,0),
                          const tf::Quaternion& orient = tf::Quaternion());
    static void frameCallback(const ros::TimerEvent&);
    static void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    visualization_msgs::Marker makeMarker();
    visualization_msgs::InteractiveMarkerControl&
        makeMarkerControl(visualization_msgs::InteractiveMarker &msg, const visualization_msgs::Marker& marker);

    void makeCubeCloud();
    visualization_msgs::InteractiveMarkerControl&
        makeBoxControl(visualization_msgs::InteractiveMarker &msg);
    void makeMovingMarker(const visualization_msgs::Marker& marker);
    void makeButtonMarker(const visualization_msgs::Marker& marker);
    void makeMenuMarker(const visualization_msgs::Marker& marker);
    void makePanTiltMarker(const visualization_msgs::Marker& marker);
    void makeChessPieceMarker(const visualization_msgs::Marker& marker);
    void makeQuadrocopterMarker(const visualization_msgs::Marker& marker);
    void makeViewFacingMarker(const visualization_msgs::Marker& marker);
    void makeRandomDofMarker(const visualization_msgs::Marker& marker);
    void make6DofMarker(bool fixed, unsigned int interaction_mode, bool show_6dof,
                        const visualization_msgs::Marker& marker);
    void saveMarker(const visualization_msgs::InteractiveMarker& int_marker);
    static void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    // -----------------------------------------------------------------------------------------------------------------
    // Static Marker --
    //
    void setStaticMarker(const visualization_msgs::Marker& marker);
    visualization_msgs::Marker& getStaticMarker();
    tf::Vector3 getStaticMarkerPos();
    void setStaticMarkerPos(const tf::Vector3& pos);
    void moveStaticMarkerPos(const tf::Vector3& distance);

    // Interactive Marker --
    //
    tf::Vector3 getInteractiveMarkerPos();
    void setInteractiveMarkerPos(const tf::Vector3& pos);
    void moveInteractiveMarkerPos(const tf::Vector3& distance);

    void setMarkerPos(const std::string& marker_name, const tf::Vector3& pos);

    // Called by _instance
    void emitMarkerPosChanged(const QVector3D& pos) {
        emit markerPosChanged(pos);
    }

    bool checkPosLimit(const tf::Vector3 & pos) {
#if 0
        return true;
#else
        return (pos.x() <= _pos_limit.x() &&
                pos.y() <= _pos_limit.y() &&
                pos.z() <= _pos_limit.z() &&
                pos.z() >= 0);
#endif
    }

signals:
    void markerPosChanged(const QVector3D& pos);

private:
    VMarker();
    static VMarker* _instance;
    static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;

    interactive_markers::MenuHandler _menu_handler;

    visualization_msgs::Marker _static_marker;
    tf::Vector3 _pos_limit;
};
#endif

