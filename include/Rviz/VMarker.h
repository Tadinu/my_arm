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
#define RVizMarker              (visualization_msgs::Marker)
#define RVizIntMarker           (visualization_msgs::InteractiveMarker)

#define CRVIZ_MARKER_TOPIC_NAME ("visualization_marker")

class VMarker : public QObject {
    Q_OBJECT

public:
    enum STATIC_MARKER_ID {
        TARGET_BALL,
        ARROW_TOOL,
        MARKER_ID_TOTAL
    };

public:
    static VMarker* getInstance();
    static bool checkInstance();
    static void deleteInstace();
    virtual ~VMarker();

    void initialize();
    void setStaticMarkerProperties(int markerId,
                          const char* header_frame,
                          uint32_t marker_shape_type,
                          const tf::Vector3& scale = tf::Vector3(1.0f, 1.0f, 1.0f),
                          const tf::Vector3& pos   = tf::Vector3(0,0,0),
                          const tf::Quaternion& orient = tf::Quaternion());
    static void frameCallback(const ros::TimerEvent&);
    static void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    // Common Marker --
    void createInteractiveMarkers();
    visualization_msgs::Marker makeStaticMarker(int marker_type = visualization_msgs::Marker::CUBE);
    void saveMarker(const visualization_msgs::InteractiveMarker& int_marker);
    static void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    visualization_msgs::InteractiveMarkerControl&
        makeMarkerControl(visualization_msgs::InteractiveMarker &msg, const visualization_msgs::Marker& marker);

    // Custom Markers
    void makeCubeCloud();
    // -----------------------------------------------------------------------------------------------------------------
    // Static Marker --
    //
    void setStaticMarker(int markerId, const visualization_msgs::Marker& marker);
    visualization_msgs::Marker& getStaticMarker(int markerId);
    tf::Vector3 getStaticMarkerPos(int markerId);
    void setStaticMarkerPos(int markerId, const tf::Vector3& pos);
    void moveStaticMarkerPos(int markerId, const tf::Vector3& distance);

    // Interactive Marker --
    //
    tf::Vector3 getInteractiveMarkerPos();
    void setInteractiveMarkerPos(const tf::Vector3& pos);
    void moveInteractiveMarkerPos(const tf::Vector3& distance);

    void setMarkerPos(const std::string& marker_name, const tf::Vector3& pos);

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


    // Called by _instance
    void emitMarkerPosChanged(const QVector3D& pos) {
        emit markerPosChanged(pos);
    }

    void setInteractiveMarkerPosLimit(const tf::Vector3& limit) {
        _pos_limit = limit;
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

    std::vector<visualization_msgs::Marker> _static_markers;

    tf::Vector3 _pos_limit;
};
#endif

