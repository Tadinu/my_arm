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
    id: thisProgressBar

    //width:
    height: width
    color : K3DRC.COLOR.TRANSPARENT

    // RESET -----------------------------
    property bool _isResetAllowed: false
    MouseArea {
        z:65535
        anchors.fill: thisProgressBar
        enabled: _isResetAllowed
        onClicked: {
            thisProgressBar.stop();
            thisProgressBar.start();
        }
    }
    // ------------------------------------

    property int _minValue: 0
    property int _maxValue: 100
    property int _value   : 0
    property int _percentValue : (_maxValue > _minValue && _value >= _minValue && _value <= _maxValue) ? (_value - _minValue)*100/(_maxValue - _minValue) : 0

    property int _partRightRotation : _percentValue <= 50 ? (180 + (360*_percentValue/100))     : 360
    property int _partLeftRotation  : (_percentValue<=100 && _percentValue >  50) ? (180 + 360*(_percentValue-50)/100) : 180

    property int _CPROGRESS_ANIMATION_DURATION: 10
    Behavior on _percentValue {
        PropertyAnimation {
            duration: thisProgressBar._CPROGRESS_ANIMATION_DURATION
            /* It is only triggered for top-level, standalone animations.
               It will not be triggered for animations in a Behavior or Transition,
               or animations that are part of an animation group.
            onStopped: {
                progressText.text = _percentValue + '%';
            }
            */
        }
    }

    function updateProgressRange(min, max) {
        _minValue = min;
        _maxValue = max;
    }
    function updateProgressValue(val) {
        _value = val;
    }

    property color _color : "steelblue"
    property int   _barWidth : 15

    property int    _CGLOBAL_BORDER_WIDTH        : 0
    property string _CGLOBAL_BORDER_GRANDPA_COLOR: "blue"
    property string _CGLOBAL_BORDER_PARENT_COLOR : "black"

    function start(){
    }

    function stop(){
    }

    Component.onCompleted: {
    }

    // PERCENTAGE TEXT [%]
    K3DText {
        id: progressText

        anchors.centerIn: parent
        text: _percentValue + '%'
        color: thisProgressBar._color
        font.bold: true
        _fontSizeScale : 3.5
    }

    // PROGRESS FRAMES
    Row {
        width : thisProgressBar.width
        height: width

        Timer {
            id: timer
            interval: 1000; running: true; repeat: true;
            //onTriggered: thisProgressBar.seconds++;
        }

        // LEFT HALF-CIRCLE PART
        Rectangle {
            width : thisProgressBar.width/2
            height: thisProgressBar.height
            clip: true
            color: K3DRC.COLOR.TRANSPARENT
            border.color: thisProgressBar._CGLOBAL_BORDER_GRANDPA_COLOR
            border.width: thisProgressBar._CGLOBAL_BORDER_WIDTH

            Rectangle {
                id: partLeft
                anchors.fill: parent
                clip: true
                rotation: partRight.rotation < 360 ? 180: thisProgressBar._partLeftRotation //!!! PART LEFT ROTATE AFTER PART RIGHT
                transformOrigin: Item.Right
                border.color: thisProgressBar._CGLOBAL_BORDER_PARENT_COLOR
                border.width: thisProgressBar._CGLOBAL_BORDER_WIDTH
                color: K3DRC.COLOR.TRANSPARENT

                Behavior on rotation {
                    enabled: thisProgressBar._partLeftRotation > rotation
                    NumberAnimation { duration: thisProgressBar._CPROGRESS_ANIMATION_DURATION }
                }

                Rectangle {
                    width: thisProgressBar.width//-(thisProgressBar._barWidth*2)
                    height: thisProgressBar.height//-(thisProgressBar._barWidth*2)
                    radius: width/2
                    x: 0 // thisProgressBar._barWidth
                    y: 0 // thisProgressBar._barWidth
                    color: K3DRC.COLOR.TRANSPARENT
                    border.color: thisProgressBar._color
                    border.width: thisProgressBar._barWidth
                    smooth: true
                }
            }
        }

        // RIGHT HALF-CIRCLE PART
        Rectangle {
            width: thisProgressBar.width/2
            height: thisProgressBar.height
            clip: true
            color: K3DRC.COLOR.TRANSPARENT

            border.color: thisProgressBar._CGLOBAL_BORDER_GRANDPA_COLOR
            border.width: thisProgressBar._CGLOBAL_BORDER_WIDTH

            Rectangle {
                id: partRight
                anchors.fill: parent
                clip: true
                color: K3DRC.COLOR.TRANSPARENT

                rotation: partLeft.rotation > 180 ? 360 : thisProgressBar._partRightRotation
                //print('PART RIGHT:', thisProgressBar._value, thisProgressBar._minValue, thisProgressBar._maxValue, thisProgressBar._partRightRotation);
                transformOrigin: Item.Left
                border.color: thisProgressBar._CGLOBAL_BORDER_PARENT_COLOR
                border.width: thisProgressBar._CGLOBAL_BORDER_WIDTH

                Behavior on rotation {
                    enabled: thisProgressBar._partRightRotation > rotation
                    NumberAnimation { duration: thisProgressBar._CPROGRESS_ANIMATION_DURATION }
                }

                Rectangle {
                    width : thisProgressBar.width//-(thisProgressBar._barWidth*2)
                    height: thisProgressBar.height//-(thisProgressBar._barWidth*2)
                    radius: width/2
                    x: - radius
                    y: 0 // thisProgressBar._barWidth
                    color: K3DRC.COLOR.TRANSPARENT
                    border.color: thisProgressBar._color
                    border.width: thisProgressBar._barWidth
                    smooth: true
                }
            }
        }
    } // End Row - PROGRESS FRAMES
}
