import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4  // Provide Qt control widgets
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Private 1.0
import QtQuick.Layouts  1.1  // Provide Qt layouts
import QtQuick.Dialogs  1.2  // StandardButton

import com.k3d.qmladapter 1.0
import MainGBSingletonObject 1.0

/* Use Rectangle instead of ToolButton for freedom in scaling the button icon */
import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL

// Tooltip --
Rectangle {
    id: tooltipFrame
    width  : toolTipText.contentWidth + MAINGB._CDIALOG_BUTTON_HEIGHT
    height : MAINGB._CDIALOG_BUTTON_HEIGHT * toolTipText.lineCount
    z: 65535
    border.width: MAINGB.getRefSize(1.5) // tooltipFrame.activeFocus ? 2 : 1
    border.color: "#434240"
    radius: height / 5
    property bool _setToolTipOpacity : true
    //color: _setToolTipOpacity ? "#80282828" : "#282828" // K3DRC.COLOR.TRANSPARENT
    color: "#282828"
//        gradient: Gradient {
//            GradientStop { position: 0 ; color: tooltip.pressed ? "#ccc" : "#eee" }
//            GradientStop { position: 1 ; color: tooltip.pressed ? "#aaa" : "#ccc" }
//        }
    //visible: true
    property bool _isShown: (tooltipFrame._tooltipText !== K3DRC.TEXT.EMPTY)
    function show() {
        showAnimation.start();
    }

    SequentialAnimation {
        id: showAnimation
        loops: 1

        NumberAnimation   { target: tooltipFrame; properties: "opacity"; from:0   ; to: 0.9; duration: 100}
        NumberAnimation   { target: tooltipFrame; properties: "opacity"; from:0.9 ; to: 1  ; duration: 1000}
        NumberAnimation   { target: tooltipFrame; properties: "opacity"; from:1   ; to: 0  ; duration: 100}
    }

    property string _tooltipText
    //K3DRectangle{
    //    width: toolTipText.width + 10; height: 20
    //    color: "#282828"
    //}

    property string _tooltipColor : "white"
    property alias _tooltip : toolTipText
    K3DText {
        id: toolTipText
        anchors.centerIn: tooltipFrame
        text : tooltipFrame._tooltipText
        //color: "#B7B7B7"
        font.letterSpacing : 1
        color              : _tooltipColor
        style              : Text.Normal
        styleColor         : "white"
        wrapMode           : Text.WordWrap
        verticalAlignment  : Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        lineHeightMode     : Text.ProportionalHeight
        //font.weight        : Font.Black
        _styleText         : Text.Outline
        _styleColor        : "#282828"
        //_styleColor        : _setToolTipOpacity ? "#80282828" : "#282828"
        //font.family: "MS Reference Sans Serif"//"Mangal"
        //font.capitalization: Font.Bold
    }
//    Timer {
//            interval: 500; running: true; repeat: true
//            onTriggered: time.text = Date().toString()
//        }
    function delay(delayTime, cb) {
        Timer.triggeredOnStart(cb)
        Timer.interval = delayTime;
        Timer.repeat = false;
        Timer.start();
    }
    property real _tooltipTopMargin : MAINGB._SCREEN_HEIGHT / 107.073
    property real _tooltipLeftMargin : MAINGB._SCREEN_HEIGHT / 72;
    property int _tooltipPos
    property Item _target
    property real _toolTipHorizontalOffset : 0
    Component.onCompleted: {
        switch(tooltipFrame._tooltipPos)
        {
        case K3DRC.TOOLTIP_POS.TOP:
            anchors.bottom           = _target.top;
            anchors.bottomMargin     = MAINGB._SCREEN_HEIGHT / 213.940 ;
            anchors.horizontalCenter = _target.horizontalCenter;
            anchors.horizontalCenterOffset = _toolTipHorizontalOffset;
            break;
        case K3DRC.TOOLTIP_POS.LEFT:
            anchors.right            = _target.left;
            anchors.rightMargin      = MAINGB._SCREEN_HEIGHT / 72;
            anchors.verticalCenter   = _target.verticalCenter;
            break;
        case K3DRC.TOOLTIP_POS.BOTTOM:
            anchors.top              = _target.bottom;
            anchors.topMargin        = tooltipFrame._tooltipTopMargin;
            anchors.horizontalCenter = _target.horizontalCenter;
            break;
        case K3DRC.TOOLTIP_POS.BOTTOM_LEFT:
            anchors.top              = _target.bottom;
            anchors.topMargin        = tooltipFrame._tooltipTopMargin;
            anchors.left             = _target.left;
            break;
        case K3DRC.TOOLTIP_POS.RIGHT:
            anchors.left             = _target.right;
            anchors.leftMargin       = tooltipFrame._tooltipLeftMargin;
            anchors.verticalCenter   = _target.verticalCenter;
            break;
        case K3DRC.TOOLTIP_POS.TOPCORNER:
            anchors.top              = _target.top;
            anchors.topMargin        = MAINGB._SCREEN_HEIGHT / 30.577;
            anchors.left             = _target.left;
            anchors.leftMargin       = tooltipFrame._tooltipLeftMargin;
            break;
        case K3DRC.TOOLTIP_POS.BOTTOMCORNER:
            anchors.bottom           = _target.bottom;
            //anchors.bottomMargin   = 35;
            anchors.right            = _target.right;
            anchors.rightMargin      = - tooltipFrame.width*2/3;
            break;
        default:
            anchors.centerIn         = _target;
            break;
        }
    }


    state : K3DRC.STATE.HIDDEN
    states: [
                 State {
                     name: K3DRC.STATE.SHOWN
                     when: tooltipFrame._isShown // && !btnMouseArea.pressed

                     PropertyChanges {
                         target :tooltipFrame
                         opacity:1.0
                     }
                     StateChangeScript {
                         name: "ShownScript"
                         script: {
                         }
                     }
                 },
                 State {
                     name: K3DRC.STATE.HIDDEN
                     when: !tooltipFrame._isShown
                     PropertyChanges {
                         target :tooltipFrame
                         opacity:0
                     }
                     StateChangeScript {
                     }
                 }
            ]
    // End STATES

    // -- TRANSITIONS
    transitions:[
                     Transition {
                         from : K3DRC.STATE.HIDDEN
                         to   : K3DRC.STATE.SHOWN
                         //sleep: 5
                         NumberAnimation   { properties: "opacity"; duration:100}
//                         NumberAnimation   { properties: "width"; duration:100}
//                         NumberAnimation   { properties: "height"; duration:100}
                         ScriptAction      { scriptName: "ShownScript" }
                     },
                     Transition {
                         //from: K3DRC.STATE.SHOWN
                         to: K3DRC.STATE.HIDDEN
                         NumberAnimation   { properties: "opacity"; duration:100}
                     }
                ]
    // End TRANSITIONS
} // End Tooltip
