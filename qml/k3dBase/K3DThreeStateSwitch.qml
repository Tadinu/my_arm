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
    id: thisSwitch
    clip: true
    color:K3DRC.COLOR.TRANSPARENT

    property string _CSTATE_ON_LEFT         : "onLeft" // 0
    property string _CSTATE_ON_RIGHT        : "onRight" // 1
    property string _CSTATE_OFF             : "off" //2
    property int    _STATE_LEFT             : 0
    property int    _STATE_OFF              : 1
    property int    _STATE_RIGHT            : 2
    property real   _STATE_LEFT_POS_X       : 1
    property real   _STATE_OFF_POS_X        : thisSwitch.width / 2 - knob.width / 2
    property real   _STATE_RIGHT_POS_X      : thisSwitch.width - knob.width
    property string _backgroundSwitch       : K3DRC.BACKGROUND.SWITCH_OFF
    property string _switchKnobSource       : K3DRC.ICON.SWITCH_KNOB
    // SWITCH BACKGROUND
    _backgroundSource: _backgroundSwitch

    property int _currentState : _STATE_OFF
    property bool _isGoRight   : true
    signal toggled(int state)      // TOGGLED BY USER

    onBaseClicked    : {
        if(mouse.x > (knob.x + knob.width / 2))
            _isGoRight = true;
        else
            _isGoRight = false;
        toggleSwitchState(_isGoRight);
    }

    function toggleSwitchState(goToRight) {
        if(goToRight) {
            if(_currentState === _STATE_RIGHT)
                _currentState = _STATE_OFF;
            else
                _currentState += 1;
        }
        else { // goToLeft
            if(_currentState === _STATE_LEFT)
                _currentState = _STATE_OFF;
            else
                _currentState -= 1;
        }
        switch(_currentState) {
            case _STATE_LEFT :
                thisSwitch.state = _CSTATE_ON_LEFT;
                break;
            case _STATE_OFF:
                thisSwitch.state = _CSTATE_OFF;
                break;
            case _STATE_RIGHT:
                thisSwitch.state = _CSTATE_ON_RIGHT;
                break;
        }
        thisSwitch.toggled(_currentState);
    }

    function releaseSwitch(goToRight) {
        if (knob.x === 1) {
            if (_currentState === _STATE_LEFT) return;
        }
        else if (knob.x === thisSwitch.width / 2 - knob.width / 2) {
            if (_currentState === _CSTATE_OFF) return;
        }
        else if (knob.x === thisSwitch.width - knob.width) {
            if (_currentState === _STATE_RIGHT) return;
        }
        // ---------------------------------------------------------
        thisSwitch.toggleSwitchState(goToRight);
    }
    // KNOB
    K3DRectangle {
        id: knob
        z : 10
        x: 1; y: 2
        anchors.verticalCenter: thisSwitch.verticalCenter

        width: height
        height: thisSwitch.height

        _backgroundSource: thisSwitch._switchKnobSource

        _baseMouseArea.drag.target: knob
        _baseMouseArea.drag.axis: Drag.XAxis
        _baseMouseArea.drag.minimumX: 1
        _baseMouseArea.drag.maximumX: thisSwitch.width - knob.width
        property real _lastPosX : thisSwitch.width / 2 - knob.width / 2
        onBaseClicked : thisSwitch.baseClicked(mouse.x + knob.x)//thisSwitch.toggleSwitchState()
        onBasePressed: _lastPosX = mouse.x
        onBaseReleased: {
            if(mouse.x > _lastPosX)
                _isGoRight = true;
            else
                _isGoRight = false;
            thisSwitch.releaseSwitch(_isGoRight);
        }
    }

    Rectangle {
        id : highlightRightRect
        z : 1
        anchors.verticalCenter: thisSwitch.verticalCenter
        anchors.left          : thisSwitch.horizontalCenter
        anchors.right         : knob.horizontalCenter

        height : thisSwitch.height * 0.8
        radius : height / 3
        color  : "#26A9E0"
    }

    Rectangle {
        id : highlightLeftRect
        z  : 1
        anchors.verticalCenter: thisSwitch.verticalCenter
        anchors.left          : knob.horizontalCenter
        anchors.right         : thisSwitch.horizontalCenter

        height : thisSwitch.height * 0.8
        radius : height / 3

        color  : "#26A9E0"
    }

    state : "off"
    states: [
        State {
            name: "onLeft"
            when: thisSwitch._isOn
            PropertyChanges { target: knob; x: _STATE_LEFT_POS_X }
        },
        State {
            name: "off"
            when: !thisSwitch._isOn
            PropertyChanges { target: knob; x: _STATE_OFF_POS_X }
        },
        State {
            name: "onRight"
            when: thisSwitch._isOn
            PropertyChanges { target: knob; x: _STATE_RIGHT_POS_X}
        }
    ]

    transitions: Transition {
        NumberAnimation { properties: "x"; easing.type: Easing.InOutQuad; duration: 200 }
    }
}

