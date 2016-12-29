import QtQuick 2.3
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import "qrc:///javascript/qmlcommonresource.js" as GEORC
//import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import MainGBSingletonObject 1.0
import com.k3d.qmladapter 1.0
import "qrc:///qml/k3dBase"

Slider {
    id: thisSlider

    property string _sliderHandleIcon : GEORC.ICON.HANDLE_SLIDER_ICON
    property real   _sliderGrooveImplicitHeight : MAINGB.getRefSize(10)
    property real   _sliderHandleImpliciHeight  : MAINGB.getRefSize(20)
    property real   _sliderGrooveRadius : MAINGB.getRefSize(3)
    property color  _sliderLeftColor    : GEORC.COLOR.GRAY
    property color  _sliderRightColor   : GEORC.COLOR.GRAY
    //tickmarksEnabled: true
    orientation: Qt.Horizontal

    property bool _hovered: false
    signal clicked()
    style: SliderStyle {
        id: sliderStyle
        groove: K3DRectangle {
            implicitWidth : thisSlider.width
            implicitHeight: _sliderGrooveImplicitHeight
            radius: _sliderGrooveRadius
            clip  : true
            _hoverEnabled: true
            on_ContainsMouseChanged: thisSlider._hovered = _containsMouse;
            Rectangle {
                id: handleRect
                x : Math.abs(value - minimumValue) / Math.abs(maximumValue - minimumValue) * parent.implicitWidth
            }

            Rectangle {
                anchors {
                    top: parent.top
                    bottom: parent.bottom
                    left: parent.left
                    right: handleRect.horizontalCenter
                }
                color: _sliderLeftColor
                radius: _sliderGrooveRadius
            }

            Rectangle {
                anchors {
                    top: parent.top
                    bottom: parent.bottom
                    right: parent.right
                    left: handleRect.horizontalCenter
                }
                color: _sliderRightColor
                radius: _sliderGrooveRadius
            }

            onBaseClicked      : thisSlider.clicked()
        }
        handle: K3DRectangle {
            //anchors.centerIn: parent
            implicitWidth : implicitHeight
            implicitHeight: _sliderHandleImpliciHeight
            radius        : implicitHeight * 3 / 4
            _backgroundSource: thisSlider._sliderHandleIcon
        }
    }
}
