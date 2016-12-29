import QtQuick 2.0
import QtQuick.Controls 1.0

import com.k3d.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

//import "qrc:///javascript/qmlcommonresource.js" as GEORC
//import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import "qrc:///qml/k3dBase"
import "qrc:///qml"

//import Ros 1.0

ApplicationWindow {
    id: mainWindow
    visible: true
    width  : 700
    height : 700

    // JOYPAD ===============================================
    //
    GeoJoypad {
        id: joyPad
        anchors.centerIn: parent
    }

    // SEND BUTTON ===============================================
    //
    Button {
        text: "Move"
        anchors.horizontalCenter: mainWindow.horizontalCenter
        anchors.top: joyPad.bottom
        anchors.topMargin: 20
        onClicked: {
            _geopadMainWindow.rotateElement(0, 3.14/4);
        }
    }

    // SLIDERS ==============================================
    //
    Slider {
        id: scaleX
        tickmarksEnabled: true
        anchors.left: joyPad.left
        anchors.top: joyPad.bottom
        anchors.topMargin: 5
        anchors.right: joyPad.right
    }

    Slider {
        id: scaleY
        tickmarksEnabled: true
        anchors.top: joyPad.top
        anchors.right: joyPad.left
        anchors.rightMargin: 5
        anchors.bottom: joyPad.bottom
        orientation: Qt.Vertical
    }

    Slider {
        id: scaleZ
        tickmarksEnabled: true
        anchors.top: mainWindow.top
        anchors.horizontalCenter: mainWindow.horizontalCenter
        orientation: Qt.Horizontal
    }

    Component.onCompleted: {
        scaleX.value = 1.0
        scaleY.value = 1.0
        scaleZ.value = 1.0
    }
}
