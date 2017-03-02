#ifndef ___VMARKER_H___
#define ___VMARKER_H___

#include <QtCore>
#include <QMutex>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Actually, moveit is built based on rviz visual tools, so already include it!
// https://github.com/davetcoleman/moveit_visual_tools
//#include <moveit_visual_tools/moveit_visual_tools.h>
// https://github.com/davetcoleman/rviz_visual_tools
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "KsGlobal.h"
#include "ros_vox_cad/Voxelyze/Utils/Mesh.h"

//#include "Voxelyze/include/Voxelyze.h"

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
    // Actually though singleton, however this technially means At least one VMarker instance being initialized
    static bool isInitialized() { return _isInitialized; } // NOT called through VMarker::getInstace()
    static void setVoxelMeshUpdated() { _isVoxelMeshUpdated = true; }
    static void setVoxelMeshInfo(std::vector<CVertex> vers,
                                 std::vector<CLine> edges,
                                 std::vector<CFacet> faces);
    //
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
    void saveMarker(const visualization_msgs::InteractiveMarker& int_marker);
    static void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    visualization_msgs::InteractiveMarkerControl&
        makeMarkerControl(visualization_msgs::InteractiveMarker &msg, const visualization_msgs::Marker& marker);
    void setMarkerPos(const std::string& marker_name, const tf::Vector3& pos);

    // Custom Markers
    void makeCubeCloud();
    // -----------------------------------------------------------------------------------------------------------------
    // Static Marker --
    //
    visualization_msgs::Marker makeStaticMarker(int marker_type = visualization_msgs::Marker::CUBE);
    void setStaticMarker(int markerId, const visualization_msgs::Marker& marker);
    visualization_msgs::Marker& getStaticMarker(int markerId);
    tf::Vector3 getStaticMarkerPos(int markerId);
    void setStaticMarkerPos(int markerId, const tf::Vector3& pos);
    void moveStaticMarkerPos(int markerId, const tf::Vector3& distance);

    // -----------------------------------------------------------------------------------------------------------------
    // Voxelyze Marker --
    //
#if 0
    visualization_msgs::Marker makeVoxelyzeMarker(int marker_type = visualization_msgs::Marker::CUBE);
    void makeCantileverBeam();
#endif
    void publishVisualArrows();
    void publishVoxelMesh(bool isShaded = true);

    // Interactive Marker --
    //
    void createInteractiveMarkers();
    tf::Vector3 getInteractiveMarkerPos();
    void setInteractiveMarkerPos(const tf::Vector3& pos);
    void moveInteractiveMarkerPos(const tf::Vector3& distance);

    // -----------------------------------------------------------------------------------------------------------------
    // Various Interactive Markers --
    //
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
    static QMutex _markerMutex;
    static VMarker* _instance;
    static bool _isInitialized;
    static bool _isVoxelMeshUpdated;

    static std::vector<CFacet> gbVoxelMeshFaces;
    static std::vector<CVertex> gbVoxelMeshVertices;
    static std::vector<CLine> gbVoxelMeshEdges;

    static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
    interactive_markers::MenuHandler _menu_handler;

    // For visualizing things in rviz
    //moveit_visual_tools::MoveItVisualToolsPtr _visual_tools;
    rviz_visual_tools::RvizVisualToolsPtr _visual_tools;

    std::vector<visualization_msgs::Marker> _static_markers;

    tf::Vector3 _pos_limit;
};
#endif

