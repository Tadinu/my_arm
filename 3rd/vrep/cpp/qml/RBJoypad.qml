import QtQuick 2.0
import QtQuick.Controls 1.0

import com.rb.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

import "qrc:///javascript/qmlcommonresource.js" as RBRC
import "qrc:///javascript/qmlcommonutil.js" as RBUTIL

//import "qrc:///qml/qmlBase"

Item {
    id: joyPad
    width: 250
    height: 250

    signal posMoved(var distance)
    Image {
        id: joyBase
        anchors.fill: parent
        source: "qrc:///res/images/joystick_background.png"

        Image {
            id: joyThumb
            source: "qrc:///res/images/joystick_thumb.png"
            x: (joyBase.width - joyThumb.width)/2
            y: (joyBase.height - joyThumb.height)/2

            property real cX: 0
            property real cY: 0

            property var twist: {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}

            function publishTwist() {
                //var header = {seq: 0, stamp: ros.now(), frame_id: ''}
                //
                //twist.linear.x = (joyThumb.x - joyThumb.cX) * scaleX.value
                //twist.angular.x = (joyThumb.y - joyThumb.cY) * scaleY.value
                //
                //pubJoy.publish({header: header, twist: joyThumb.twist})
                // -------------------------------------------------------
                joyPad.posMoved(Qt.vector3d((joyThumb.x - joyThumb.cX)/100, (joyThumb.y - joyThumb.cY)/100, 0));
                updateThumbPos();
            }

            function updateThumbPos() {
                if(joyThumb.x !== joyThumb.cX ||
                   joyThumb.y !== joyThumb.cY) {
                    joyThumb.cX = joyThumb.x;
                    joyThumb.cY = joyThumb.y;
                }
            }

            onXChanged: publishTwist()
            onYChanged: publishTwist()

            MouseArea {
                id: mouseArea
                anchors.fill: parent
                drag.target: joyThumb
                drag.minimumX: 0
                drag.minimumY: 0
                drag.maximumX: joyBase.width - joyThumb.width
                drag.maximumY: joyBase.height - joyThumb.height

                onPressed: {
                    joyThumb.updateThumbPos();
                }
            }
        }
    }
}
