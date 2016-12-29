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

SpinBox {
    id: thisSpinBox // control
    Layout.fillWidth: true
    objectName : K3DRC.TEXT.K3D_SPINBOX_OBJECT_NAME

    property int _implicitWidth  : 100
    property int _implicitHeight : MAINGB._CDIALOG_SPINBOX_HEIGHT

    property color _backgroundColor : K3DRC.COLOR.TRANSPARENT
    property color _backgroundFocusColor : _backgroundColor
    property color _textColor    : "white"
    property color _backgroundBorderColor: "#67686A"
    property color _backgroundBorderHoveredColor : "#898989"
    property color _backgroundBorderFocusColor : "#E3E3E3"
    property color _selectionColor: "#3399FF"
    //decimals: 0
    //stepSize: 1
    horizontalAlignment: Qt.AlignLeft
    property int _paddingLeft   : horizontalAlignment === Qt.AlignLeft  ? font.pixelSize / 2 : 0
    property int _paddingRight  : horizontalAlignment === Qt.AlignRight ? font.pixelSize / 2 : 0
    property int _paddingTop    : 0//(implicitHeight - font.pixelSize) / 2
    property int _paddingBottom : 0//(implicitHeight - font.pixelSize) / 2
    property int _borderFocusWidth : _borderWidth * 2
    property int _borderWidth      : 1

    //property alias _cursorPosition : input.cursorPosition

    // Set font----------------------------------------------------------------
    property real _fontSizeScale  : 1.0
    property var  _fontSource : K3DRC.FONT.MAVEN_PRO_REGULAR
    FontLoader { id: customFont; source: _fontSource }
    font.pixelSize : MAINGB._BASE_FONT_SIZE * _fontSizeScale
    font.family    : customFont.name

    // ------------------------------------------------------------------------
    property var _topParent   : null
    property int _spinBoxType : K3DRC.SPINBOX_TYPE.NONE
    property bool _isInSpinBoxItem : false
    onValueChanged : {
        if(_spinBoxType === K3DRC.SPINBOX_TYPE.METRIC && !_isInSpinBoxItem)
            MAINGB.validateSpinBoxParamsSingleStep(thisSpinBox);

        if(_topParent !== null) {
            _topParent.paramsChanged();
        }
    }

    // ------------------------------------------------------------------------
    style: SpinBoxStyle {
            textColor     : thisSpinBox._textColor
            decrementControl : Item { visible: false }
            incrementControl : Item { visible: false }

            padding { top: _paddingTop ; left: thisSpinBox._paddingLeft ; right: _paddingRight ; bottom: _paddingBottom }
            selectionColor : _selectionColor
            renderType: Text.QtRendering
            background: Rectangle {
                implicitHeight: Math.max(thisSpinBox._implicitHeight, Math.round(styleData.contentHeight))
                implicitWidth : Math.max(thisSpinBox._implicitWidth, styleData.contentWidth)
                //baselineOffset: control.__baselineOffset
                border.color  : control.hovered? control.focus? thisSpinBox._backgroundBorderHoveredColor : thisSpinBox._backgroundBorderFocusColor : thisSpinBox._backgroundBorderColor
                border.width  : control.focus? _borderFocusWidth : _borderWidth
                //radius: 2
                color         : (control.hovered ||  control.focus) ? thisSpinBox._backgroundFocusColor : thisSpinBox._backgroundColor

                smooth        : true
                antialiasing  : true
            }
        }

    onHoveredChanged: {
        if(hovered)
            tooltipFrame.startTime = 0
        else
            tooltipFrame.startTime = -100
    }
    property alias  _spinBoxToolTip             : tooltipFrame._tooltipText

    K3DTooltip {
        id: tooltipFrame
        z: 65535
        property double startTime: -10
        property double secondsElapsed: 0
        property bool result: (_spinBoxToolTip !== K3DRC.TEXT.EMPTY) && thisSpinBox.hovered && (tooltipFrame.secondsElapsed >=2);

        function timeChanged(){
            if (tooltipFrame.startTime ==0){
                secondsElapsed++ //+= elapsedTimer.interval/500
            }
        }
        Timer  {
            id: elapsedTimer
            interval: 500;
            running: true;
            repeat: true;
            onTriggered: tooltipFrame.timeChanged()
        }
        _target    : thisSpinBox
        _tooltipPos: K3DRC.TOOLTIP_POS.TOP
        _isShown   : result
    } // End Tooltip
}
