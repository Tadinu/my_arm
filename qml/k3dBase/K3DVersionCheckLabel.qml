import QtQuick 2.3

// http://qt-project.org/doc/qt-5/qtquickcontrols-index.html
import QtQuick.Controls 1.2 // Provide Qt control widgets
import QtQuick.Controls.Styles 1.2

import com.k3d.qmladapter 1.0
import MainGBSingletonObject 1.0

/* Use Rectangle instead of ToolButton for freedom in scaling the button icon */
import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL

Rectangle {
    id : thisVersionCheckLabel
    property string _CUPDATE_CONTENT

    width : updateStateIcon.width * 1.4 + updateInfoLabel.width
    height: updateStateIcon.height
    color : K3DRC.COLOR.TRANSPARENT

    signal updateLinkActivated(var url)
    signal updateLinkHoverChanged(var link)

    // UPDATE STATUS --
    //
    property int _CUPDATE_STATUS_NONE      : 0x01
    property int _CUPDATE_STATUS_UPDATING  : 0x02
    property int _CUPDATE_STATUS_UPDATED   : 0x04
    property int _CUPDATE_STATUS_OUTOFDATE : 0x08

    property string _CUPDATE_TEXT_NONE
    property string _CUPDATE_TEXT_UPDATING
    property string _CUPDATE_TEXT_UPDATED
    property string _CUPDATE_TEXT_OUTOFDATE

    property int _updateStatus : _CUPDATE_STATUS_NONE // By default, in case no update action taken!
    function getUpdateStatus() {
        return updateAnimation.running;
    }

    function setUpdateStatus(isUpdating) {
        if(isUpdating) {
            updateAnimation.start();
            _updateStatus = _CUPDATE_STATUS_UPDATING;
        }
        else {
            updateAnimation.stop();
            if(_updateStatus != _CUPDATE_STATUS_UPDATED &&
               _updateStatus != _CUPDATE_STATUS_OUTOFDATE)
               _updateStatus = _CUPDATE_STATUS_NONE;
            // -> Wait to be updated in setUpdateResult(), else remain None!
        }
    }

    property string _version
    function setVersion(version) {
        _version = version;
    }

    property var _updateUrl : null
    property var _releaseNoteUrl : null
    function setUpdateResult(isUpdateFound, version, url, releaseNoteUrl) {
        // 1-
        updateStateIcon.rotation = 0;
        _updateStatus = isUpdateFound ? _CUPDATE_STATUS_OUTOFDATE :
                                        _CUPDATE_STATUS_UPDATED;

        // 2-
        _version = version; // version can be either New Version (isUpdateFound) or current local Material Version!

        // 3-
        _updateUrl      = url;
        _releaseNoteUrl = releaseNoteUrl;
    }

    property int _rotateDuration : 500
    NumberAnimation {
        id: updateAnimation
        running: false
        loops  : Animation.Infinite
        target : updateStateIcon; property: "rotation"; from:360; to:0 ; duration: thisVersionCheckLabel._rotateDuration
    }

    K3DImage {
        id : updateStateIcon
        anchors{
            verticalCenter  : parent.verticalCenter
            left            : parent.left
        }
        height: updateInfoLabel.height * 1.1
        width : updateInfoLabel.height * 1.1
        //_btnRotation : 0
        source : thisVersionCheckLabel._updateStatus === thisVersionCheckLabel._CUPDATE_STATUS_UPDATING  ? K3DRC.ICON.ABOUT_UPDATING    :
                 thisVersionCheckLabel._updateStatus === thisVersionCheckLabel._CUPDATE_STATUS_OUTOFDATE ? K3DRC.ICON.ABOUT_OUT_OF_DATE :
                 thisVersionCheckLabel._updateStatus === thisVersionCheckLabel._CUPDATE_STATUS_UPDATED   ? K3DRC.ICON.ABOUT_UP_TO_DATE  : ""
    }

    K3DText {
        id : updateInfoLabel
        anchors{
            verticalCenter  : parent.verticalCenter
            right           : parent.right
        }
        font.bold: true
        _fontSizeScale : 1
        color : "#1A8BD8"

        textFormat: Text.RichText
        text : thisVersionCheckLabel._updateStatus === thisVersionCheckLabel._CUPDATE_STATUS_UPDATING  ? thisVersionCheckLabel._CUPDATE_TEXT_UPDATING  :
               thisVersionCheckLabel._updateStatus === thisVersionCheckLabel._CUPDATE_STATUS_OUTOFDATE ? thisVersionCheckLabel._CUPDATE_TEXT_OUTOFDATE :
               thisVersionCheckLabel._updateStatus === thisVersionCheckLabel._CUPDATE_STATUS_UPDATED   ? thisVersionCheckLabel._CUPDATE_TEXT_UPDATED   :
               thisVersionCheckLabel._CUPDATE_TEXT_NONE

        //linkColor : "#06B2D6"
        onLinkActivated : {
            //print('Link Activated:', link);
            thisVersionCheckLabel.updateLinkActivated(thisVersionCheckLabel._updateUrl);
        }

        //!!! VERY IMPORTANT
        onHoveredLinkChanged: {
            print('HOVER LINK   CHANGED:', hoveredLink);
            thisVersionCheckLabel.updateLinkHoverChanged(hoveredLink);
        }
    }
}
