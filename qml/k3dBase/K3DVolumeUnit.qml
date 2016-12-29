import QtQuick 2.3
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Layouts  1.1  // Provide Qt layouts

import "qrc:///javascript/qmlcommonresource.js" as K3DRC
// Volume Circle List
Rectangle {
    id: thisVolumeUnit

    property int  _posId
    property real _deltaRadiant: Math.PI/10
    property real _rad: _posId * _deltaRadiant
    property real _csDIM_OPACITY: 0.3

    width: 12
    height: width
    radius: width*0.5
    color: "white"
    opacity: _posId == 0 ? 1:_csDIM_OPACITY // This dependency will be invalidated upon assignment to _posId !

    x: parent._centerX + parent._posRadius* Math.cos(_rad)-5
    y: parent._centerX + parent._posRadius* Math.sin(_rad)-5

    // -- Mouse Area
    K3DMouseArea {
        id:mouseArea
        hoverEnabled: true // FOR USING onEntered & onExited

        anchors.fill: parent

        onEntered: {
            if (thisVolumeUnit._posId != 0)
            {
                if (thisVolumeUnit.opacity == thisVolumeUnit._csDIM_OPACITY) {
                    goShine();
                }
                else {
                    goDim();
                }
            }
        }
        onExited: {
            //volumeCircle.opacity = 1;
        }
    }

    function goShine() { // opacity: 1
        if (thisVolumeUnit._posId == parent._currentVolPos + 1)
        {
            // Current vol pos:
            parent._currentVolPos = thisVolumeUnit._posId;
            // Opacity:
            thisVolumeUnit.opacity = 1;

        }
    }

    function goDim() {   // opacity: _csDIM_OPACITY
        if ( (thisVolumeUnit._posId == parent._topCircleId) || // The top is the last one to be shined, so let it just be dimmed if To be dimmed!
             (thisVolumeUnit._posId == parent._currentVolPos)
            )
        {
            // Current vol pos:
            parent._currentVolPos = thisVolumeUnit._posId - 1;
            // Opacity:
            thisVolumeUnit.opacity  = thisVolumeUnit._csDIM_OPACITY;
        }
    }
}
