import QtQuick 2.0
import QtQuick.Controls 1.0

import com.rb.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

import "qrc:///javascript/qmlcommonresource.js" as RBRC


import "qrc:///qml"

//import Ros 1.0

ApplicationWindow {
    id: mainWindow
    visible: true
    width  : 700
    height : 700

    // JOYPAD ===============================================
    RBJoypad {
        id: joyPad
        anchors.right : parent.right
        anchors.verticalCenter : parent.verticalCenter
        anchors.rightMargin : 80
        onPosMoved : {
            //_rbMainWindowAgent.moveTarget(distance);
        }
    }

    // SLIDERS ==============================================
    //
//    Slider {
//        id: moveZ
//        tickmarksEnabled: true
//        anchors.left: joyPad.right
//        anchors.leftMargin: width
//        anchors.verticalCenter: joyPad.verticalCenter
//        orientation: Qt.Vertical

//        stepSize: 0.01
//        minimumValue: -100
//        maximumValue: 100

//        value: 0.0
//        property real _value: 0.0
//        Component.onCompleted: {
//            _value = value;
//        }

//        onValueChanged: {
//            //_rbMainWindowAgent.moveTarget(Qt.vector3d(0,0, (value - _value)/10));
//            _value = value;
//        }
//    }

    Button {
        text: "Center"
        anchors.horizontalCenter: joyPad.horizontalCenter
        anchors.top: joyPad.bottom
        anchors.topMargin: 20
        onClicked: {
            //_rbMainWindowAgent.onRobotPostureReset();
        }
    }

    // SLIDER VALUES ===============================================
    //
    /*
    Row {
        id: sliderGroup
        anchors.verticalCenter : parent.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 80

        property real _CPOS_LIMIT: 2*Math.PI/3
        spacing: 5
        Slider {
            id: moveBaseJoint
            tickmarksEnabled: true
            orientation: Qt.Vertical
            anchors.verticalCenter: parent.verticalCenter

            stepSize: 0.01
            value : 0
            minimumValue: -sliderGroup._CPOS_LIMIT
            maximumValue: sliderGroup._CPOS_LIMIT
            onValueChanged: {
                _rbMainWindowAgent.setRobotJointPos(K3DQMLAdapter.MYARM_BASE_JOINT, value);
            }
        }

        Slider {
            id: moveJoint10
            tickmarksEnabled: true
            orientation: Qt.Vertical
            anchors.verticalCenter: parent.verticalCenter

            stepSize: 0.01
            value : 0
            minimumValue: -sliderGroup._CPOS_LIMIT
            maximumValue: sliderGroup._CPOS_LIMIT
            onValueChanged: {

            }
        }

        Slider {
            id: moveJoint20
            tickmarksEnabled: true
            orientation: Qt.Vertical
            anchors.verticalCenter: parent.verticalCenter

            stepSize: 0.01
            value : 0
            minimumValue: -sliderGroup._CPOS_LIMIT
            maximumValue: sliderGroup._CPOS_LIMIT
            onValueChanged: {
            }
        }

        Slider {
            id: moveJoint3 // Wrist Joint : Revolute Z
            tickmarksEnabled: true
            orientation: Qt.Vertical
            anchors.verticalCenter: parent.verticalCenter

            stepSize: 0.01
            value : 0
            minimumValue: -sliderGroup._CPOS_LIMIT
            maximumValue: sliderGroup._CPOS_LIMIT
            onValueChanged: {
            }
        }
    }
    */
}
