import QtQuick 2.0
import QtQuick.Controls 1.0

import com.k3d.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

//import "qrc:///javascript/qmlcommonresource.js" as GEORC
//import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL

import "qrc:///qml/k3dBase"

Item {
    id: joyPad
    width: 250
    height: 250

    Image {
        id: joyBase
        anchors.fill: parent
        source: "qrc:///res/images/joystick_background.png"

        Image {
            id: joyThumb
            source: "qrc:///res/images/joystick_thumb.png"
            x: cX
            y: cY

            readonly property int cX: (joyBase.width - width) / 2
            readonly property int cY: (joyBase.height - height) / 2

            property var twist: {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}

            //function publishTwist() {
            //    var header = {seq: 0, stamp: ros.now(), frame_id: ''}
            //
            //    twist.linear.x = (joyThumb.x - joyThumb.cX) * scaleX.value
            //    twist.angular.x = (joyThumb.y - joyThumb.cY) * scaleY.value
            //
            //    pubJoy.publish({header: header, twist: joyThumb.twist})
            //}

            onXChanged: {
                if (mouseArea.drag.active) {
                    //publishTwist()
                }
            }

            onYChanged: {
                if (mouseArea.drag.active) {
                    //publishTwist()
                }
            }

            MouseArea {
                id: mouseArea
                anchors.fill: parent
                drag.target: joyThumb
                drag.minimumX: 0
                drag.minimumY: 0
                drag.maximumX: joyBase.width - joyThumb.width
                drag.maximumY: joyBase.height - joyThumb.height

                onReleased: {
                    drag.target.x = drag.target.cX
                    drag.target.y = drag.target.cY
                }
            }
        }
    }
}
