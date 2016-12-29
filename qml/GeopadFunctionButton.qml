import QtQuick 2.3
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import MainGBSingletonObject 1.0
//import "qrc:///javascript/qmlcommonresource.js" as GEORC
////import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import "qrc:///qml/k3dBase"
K3DRectangle {
    id: thisFunctionButton
    _hoverEnabled: true
    border.color: GEORC.COLOR.GRAY // gray in omny.qml
    border.width: MAINGB.getRefSize(1)
    radius: MAINGB.getRefSize(3)
    clip  : true

    property real _normalIconSize : 0.4
    property real _hoverIconSize  : 0.5
    property string _ICON_SOURCE          : GEORC.ICON.GREEN_ARROW
    property real   _CBUTTON_WIDTH        : thisFunctionButton.width / 4
    property bool _hoveredOn: thisFunctionButton._containsMouse || downButton._containsMouse || upButton._containsMouse

    signal up()
    signal down()
    property bool _isFuncBtnEnabled    : true
    property alias _value       : labelText.text
    Rectangle {
        id: labelRect
        z : 20
        anchors {
            left        : downButton.right
            leftMargin  : - downButton._themeRadius
            right       : upButton.left
            rightMargin : - upButton._themeRadius
            top         : thisFunctionButton.top
            bottom      : thisFunctionButton.bottom
            topMargin   : thisFunctionButton.border.width
            bottomMargin: thisFunctionButton.border.width
        }
        color: _hoveredOn ? GEORC.COLOR.GRAY : GEORC.COLOR.WHITE
        K3DText {
            id: labelText
            font.bold: true
            font.letterSpacing: 0
            anchors.centerIn: parent
            color: GEORC.COLOR.BLACK
        }
    }

    K3DButton {
        id: downButton
        z : 10
        anchors {
            left    : thisFunctionButton.left
            top     : thisFunctionButton.top
            bottom  : thisFunctionButton.bottom
            margins : thisFunctionButton.border.width
        }
        width            : _CBUTTON_WIDTH
        enabled          : _isFuncBtnEnabled
        _themeColor      : /*_containsMouse ? GEORC.COLOR.GRAY : */GEORC.COLOR.LIGHTGRAY
        _themeRadius     : thisFunctionButton.radius
        _btnType         : _CBUTTON_TYPE_ICON_TEXT
        _touchButtonImage.sourceSize.height: height * (_containsMouse ? _hoverIconSize : _normalIconSize)
        _btnRotation     : 180
        _btnNormalSource : _ICON_SOURCE
        _btnHoveredSource: _ICON_SOURCE
        _btnPressedSource: _ICON_SOURCE
        //on_ContainsMouseChanged: mainMouseArea.enabled != _containsMouse
        onClicked: {
            thisFunctionButton.down();
        }
    }

    K3DButton {
        id: upButton
        z: 10
        anchors {
            right  : thisFunctionButton.right
            top    : thisFunctionButton.top
            bottom : thisFunctionButton.bottom
            margins: thisFunctionButton.border.width
        }
        width            : _CBUTTON_WIDTH
        enabled          : _isFuncBtnEnabled
        _themeColor      : /*_containsMouse ? GEORC.COLOR.GRAY :*/ GEORC.COLOR.LIGHTGRAY
        _themeRadius     : thisFunctionButton.radius
        _btnType         : _CBUTTON_TYPE_ICON_TEXT
        _touchButtonImage.sourceSize.height: height * (_containsMouse ? _hoverIconSize : _normalIconSize)
        _btnNormalSource : _ICON_SOURCE
        _btnHoveredSource: _ICON_SOURCE
        _btnPressedSource: _ICON_SOURCE
        //on_ContainsMouseChanged: mainMouseArea.enabled != _containsMouse
        onClicked: {
            thisFunctionButton.up();
        }
    }
}
