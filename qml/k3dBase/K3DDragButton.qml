import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4  // Provide Qt control widgets
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Private 1.0
import QtQuick.Layouts  1.1  // Provide Qt layouts
import QtQuick.Dialogs  1.2  // StandardButton

import com.k3d.qmladapter 1.0
import MainGBSingletonObject 1.0
import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL
//import "qrc:///qml/k3dBase"

K3DRectangle {
    id: thisDragButton
    z: 65535

    //width:
    height: width
//    border.color: "gray"
//    border.width: 2
    radius: width*0.5

    _isDraggable : true
    _isPropagateComposedEvents : false
    property bool  _isFreeDrifting : false
    property bool  _isInteractiveOn: true
    property real  _tossTime
    property real  _CTOSS_THRESHOLD: 100
    property real  _lastX
    property real  _lastY

    property real _currentWidth : width // Initially
    property real _currentHeight: height
    signal clicked(var mouse)
    signal rightClicked(var mouse)
    signal pressed(var mouse)
    signal released(var mouse)
    signal mouseIn()
    signal mouseOut()
    signal tossed // casted

    // Property Animation --

    // -- Signals

    // -- Methods

    property real _msecTick: 0
    Timer  {
         id: elapsedTimer
         interval: 1000;
         running: true;
         repeat: true;
         onTriggered: {
             _msecTick += 1000;
             if(_msecTick > 5000) _msecTick = 0;

             // ---------------------------------
//             if(thisDragButton._isFreeDrifting){
//                 thisDragButton.x = Math.random() * (thisDragButton._curDragMaximumX - thisDragButton._curDragMinimumX) + thisDragButton._curDragMinimumX;
//                 thisDragButton.y = Math.random() * (thisDragButton._curDragMaximumY - thisDragButton._curDragMinimumY) + thisDragButton._curDragMinimumY;
//             }
         }
     }

//    Behavior on x { SmoothedAnimation { duration: 10000 } }

//    Behavior on y { SmoothedAnimation { duration: 10000 } }


    // Mouse Area --
    // _dragTarget by default is thisDragButton upon [_isDraggable] as true
    _origDragMinimumX : MAINGB._CGBDRAG_ZONE_MIN_X + (thisDragButton._currentWidth - thisDragButton.width)/2
    _origDragMaximumX : MAINGB._CGBDRAG_ZONE_MAX_X - (thisDragButton._currentWidth + thisDragButton.width)/2
    _origDragMinimumY : MAINGB._CGBDRAG_ZONE_MIN_Y + (thisDragButton._currentHeight - thisDragButton.height)/2
    _origDragMaximumY : MAINGB._CGBDRAG_ZONE_MAX_Y - (thisDragButton._currentHeight + thisDragButton.height)/2

    _backgroundImageZ : 1
    _baseMouseAreaZ   : 2

    _baseMouseArea.enabled      : _isInteractiveOn
    _baseMouseArea.hoverEnabled : _isInteractiveOn

    onBaseClicked: {
        thisDragButton._isFreeDrifting = false;
        thisDragButton.clicked(mouse);
    }

    onBaseRightClicked: {
        thisDragButton._isFreeDrifting = false;
        thisDragButton.rightClicked(mouse);
    }

    _hoverEnabled: true
    onBaseEntered: {
        thisDragButton.mouseIn();
    }

    onBaseExited: {
        thisDragButton.mouseOut();
    }

    // Cursor Shape --
    _cursorShape : thisDragButton._pressed ?
                   Qt.ClosedHandCursor : Qt.PointingHandCursor
    // ========================================================================================
    property real _timeStamp

    onBasePressed: {
        thisDragButton._isFreeDrifting = false;
        thisDragButton.pressed(mouse);
    }
    onBasePressAndHold: {
        thisDragButton._isFreeDrifting = false;
        thisDragButton._lastX = thisDragButton.x;
        thisDragButton._lastY = thisDragButton.y;
        _timeStamp = thisDragButton._msecTick;
    }

    onBaseReleased: {
        //1 - SIGNAL [released] first:
        thisDragButton.released(mouse);
        // ---------------------------------------------------------------------------------------------
        //2 - SIGNAL [tossed] later:
        var deltaX = Math.abs(thisDragButton.x - thisDragButton._lastX);
        var deltaY = Math.abs(thisDragButton.y - thisDragButton._lastY);

        var currentTime = new Date().getTime();
        if ( (deltaX >= thisDragButton._CTOSS_THRESHOLD || deltaY >= thisDragButton._CTOSS_THRESHOLD) &&
             thisDragButton._msecTick - _timeStamp <= 1000
           ) {
            thisDragButton._isFreeDrifting = true;
            thisDragButton.tossed();
        }
    }
}
