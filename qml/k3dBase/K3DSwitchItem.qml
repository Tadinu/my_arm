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

Item {
    id: thisSwitchItem
    //clip: true

    width: switchX.width + switchName.width + K3DRC.MARGIN.BUTTON_ICON_TEXT_HORIZONTAL
    height: switchX.height

    property alias _buttonWidth     : switchX.width
    property alias _buttonHeight    : switchX._CHEIGHT
    property alias _isOn            : switchX._isOn
    property int   _textLeftMargin  : K3DRC.MARGIN.BUTTON_ICON_TEXT_HORIZONTAL
    property int   _textRightMargin : K3DRC.MARGIN.BUTTON_ICON_TEXT_HORIZONTAL

    signal toggled(bool isOn)      // TOGGLED BY USER, NOT BY CODE [on_IsOnChanged:]
    signal stateChanged(bool isOn) // BOTH BY USER & CODE
    signal clicked()
    property alias _CSTATE_TEXT_ON : switchX._CSTATE_TEXT_ON
    property alias _CSTATE_TEXT_OFF : switchX._CSTATE_TEXT_OFF
    property alias _CSTATE_TEXT_FONT_SCALE : switchX._CSTATE_TEXT_FONT_SCALE
    property alias _backgroundSwitchOn  : switchX._backgroundSwitchOn
    property alias _backgroundSwitchOff : switchX._backgroundSwitchOff
    property alias _enabled             : switchX.enabled

    K3DSwitch {
        id: switchX
        //width: height * 2 // By default
        onToggled     : thisSwitchItem.toggled(isOn)
        onStateChanged: thisSwitchItem.stateChanged(isOn)
        onBaseClicked : { thisSwitchItem.clicked(); thisSwitchItem.forceActiveFocus(); }
    }

    // TEXT
    property int _switchTextPost: K3DRC.SWITCH_TEXT_POS.RIGHT
    // Switch Name
    property alias _text: switchName.text
    property alias _textWidth : switchName.width
    property alias _switchName: switchName
    K3DText {
        id: switchName
        Component.onCompleted: {
            anchors.verticalCenter = switchX.verticalCenter;
            switch(_switchTextPost) {
            case 1: // RIGHT
                anchors.left = switchX.right;
                anchors.leftMargin = thisSwitchItem._textLeftMargin;
                break;
            case 2: // LEFT
                anchors.right = switchX.left;
                anchors.rightMargin = thisSwitchItem._textRightMargin;
                break;
            }
        }
    }
}
