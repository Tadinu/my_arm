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

    property string _CSTATE_ON  : "on"
    property string _CSTATE_OFF : "off"
    property string _backgroundSwitchOn  : K3DRC.BACKGROUND.SWITCH_ON
    property string _backgroundSwitchOff : K3DRC.BACKGROUND.SWITCH_OFF
    property string _switchKnobSource    : K3DRC.ICON.SWITCH_KNOB
    property string _CSTATE_TEXT_ON      : ""
    property string _CSTATE_TEXT_OFF     : ""
    property real   _CSTATE_TEXT_FONT_SCALE : 1
    // SWITCH BACKGROUND
    //color: thisSwitch.state === thisSwitch._CSTATE_ON ? "lightblue" : "black"
    _backgroundSource: thisSwitch.state === thisSwitch._CSTATE_ON ? thisSwitch._backgroundSwitchOn :
                       thisSwitch.state === thisSwitch._CSTATE_OFF? thisSwitch._backgroundSwitchOff: ""

    _hoverEnabled    : true
    //Change cursor key to pointinghand
    _cursorShape     : _hoverEnabled ? Qt.PointingHandCursor : Qt.ArrowCursor
    onBaseClicked    : userToggleState()
    opacity: enabled ? 1 : 0.5
    property bool _isOn: false

    signal toggled(bool isOn)      // TOGGLED BY USER
    signal stateChanged(bool isOn) // BOTH BY USER & CODE
    function userToggleState() {
        thisSwitch._isOn = !thisSwitch._isOn;
        thisSwitch.toggled(_isOn);
    }
    on_IsOnChanged: thisSwitch.stateChanged(_isOn);

    function releaseSwitch() {
        if (knob.x === 1) {
            if (!thisSwitch._isOn) return;
        }
        else if (knob.x === thisSwitch.width - knob.width) {
            if (thisSwitch._isOn) return;
        }
        // ---------------------------------------------------------
        thisSwitch.userToggleState();
    }
    property real _CHEIGHT
    property real _WIDTH_HEIGHT_RATIO : 2
    height : _CHEIGHT > 0 ? _CHEIGHT : stateText.width === 0 ? 0 : stateText.height > 0 ? stateText.height * 2.5  : 15
    width  : ((stateText.width + knob.width * 1.3) > height * _WIDTH_HEIGHT_RATIO) ? (stateText.width + knob.width * 1.3)
                                                                                   :  height * _WIDTH_HEIGHT_RATIO

    K3DText{
        id: stateText
        anchors.verticalCenter:  knob.verticalCenter
        x : thisSwitch.state === thisSwitch._CSTATE_ON ? thisSwitch.width / 10 : thisSwitch.width - thisSwitch.width / 10 - width
        text : thisSwitch.state === thisSwitch._CSTATE_ON ? _CSTATE_TEXT_ON : _CSTATE_TEXT_OFF
        font.bold: true
        _fontSizeScale : thisSwitch._CSTATE_TEXT_FONT_SCALE //thisSwitch.height / 30 // 2 Times base switch height, that meaan font size = 6 when this.height = 15
    }

    property real _disableOpacity : 0.5
    // KNOB
    K3DRectangle {
        id: knob
        x: 1; y: 2
        anchors.verticalCenter: thisSwitch.verticalCenter

        width: height//thisSwitch.width/2
        height: thisSwitch.height

        _hoverEnabled : true
        _backgroundSource: thisSwitch._switchKnobSource
        opacity: thisSwitch.enabled ? 1 : thisSwitch._disableOpacity
        _baseMouseArea.drag.target: knob
        _baseMouseArea.drag.axis: Drag.XAxis
        _baseMouseArea.drag.minimumX: 1
        _baseMouseArea.drag.maximumX: thisSwitch.width - knob.width

        onBaseClicked : thisSwitch.userToggleState()
        onBaseReleased: thisSwitch.releaseSwitch()
    }

    states: [
        State {
            name: "on"
            when: thisSwitch._isOn
            PropertyChanges { target: knob; x: thisSwitch.width - knob.width }
        },
        State {
            name: "off"
            when: !thisSwitch._isOn
            PropertyChanges { target: knob; x: 1 }
        }
    ]

    transitions: Transition {
        NumberAnimation { properties: "x"; easing.type: Easing.InOutQuad; duration: 200 }
    }
}
