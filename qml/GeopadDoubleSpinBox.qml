import QtQuick 2.0
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import MainGBSingletonObject 1.0
//import "qrc:///javascript/qmlcommonresource.js" as GEORC
////import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import "qrc:///qml/k3dBase"
Rectangle {
    id: thisDoubleSpinBoxItem
    z : 10
    border.color: GEORC.COLOR.GRAY // gray in omny.qml
    border.width: MAINGB.getRefSize(1)
    radius: MAINGB.getRefSize(4)
    color : GEORC.COLOR.TRANSPARENT
    clip  : true

    property alias _text : labelText.text
    property alias _labelWidth: labelRect.width
    Rectangle {
        id: labelRect
        z : 1
        anchors {
            left  : thisDoubleSpinBoxItem.left
            top   : thisDoubleSpinBoxItem.top
            bottom: thisDoubleSpinBoxItem.bottom
            margins: thisDoubleSpinBoxItem.border.width
        }
        color: GEORC.COLOR.GREEN
        width: labelText.contentWidth + height
        radius: thisDoubleSpinBoxItem.radius - thisDoubleSpinBoxItem.border.width
        clip : true
        K3DText {
            id: labelText
            anchors {
                verticalCenter: labelRect.verticalCenter
                left          : labelRect.left
                leftMargin    : height / 3
            }
            font.letterSpacing : 0
        }
    }

    // SpinBox
    property alias _spinBox : spinBox
    signal editingFinished()

    property alias _topParent       : spinBox._topParent
    property alias _spinBoxType     : spinBox._spinBoxType

    property alias _hovered         : spinBox.hovered
    property alias _focus           : spinBox.focus
    property alias _value           : spinBox.value
    property alias _decimals        : spinBox.decimals
    property alias _stepSize        : spinBox.stepSize
    property alias _minValue        : spinBox.minimumValue
    property alias _maxValue        : spinBox.maximumValue
    property int   _unitRightMargin : MAINGB._CDIALOG_SPINBOX_HEIGHT / 4
    property alias _spinBoxToolTip  : spinBox._spinBoxToolTip

    K3DSpinBox {
        id: spinBox
        z: 5
        focus: false
        _backgroundBorderColor       : GEORC.COLOR.GRAY // gray in omny.qml
        _backgroundBorderHoveredColor: GEORC.COLOR.GRAY
        _backgroundBorderFocusColor  : GEORC.COLOR.GRAY
        _backgroundColor             : GEORC.COLOR.WHITE
        _backgroundFocusColor        : GEORC.COLOR.LIGHTGRAY
        _textColor                   : GEORC.COLOR.BLACK
        _selectionColor              : GEORC.COLOR.GRAY
        horizontalAlignment : Qt.AlignHCenter
        maximumValue: 999999999
        minimumValue: 0
        // width: To be set outside
        anchors {
            left       : labelRect.right
            leftMargin : - labelRect.radius
            right      : defaultLabel.left
            rightMargin: - defaultLabel.radius
            top        : thisDoubleSpinBoxItem.top
            bottom     : thisDoubleSpinBoxItem.bottom
        }
        _borderFocusWidth: _borderWidth

        onEditingFinished : thisDoubleSpinBoxItem.editingFinished()
    }

    property alias _defaulttext : defaultValue.value
    Rectangle {
        id: defaultLabel
        z : 1
        anchors {
            right : thisDoubleSpinBoxItem.right
            top   : thisDoubleSpinBoxItem.top
            bottom: thisDoubleSpinBoxItem.bottom
            margins: thisDoubleSpinBoxItem.border.width
        }
        radius: thisDoubleSpinBoxItem.radius - thisDoubleSpinBoxItem.border.width
        color: GEORC.COLOR.GRAY
        width: (thisDoubleSpinBoxItem.width - labelRect.width) / 2 + radius
        K3DSpinBox {
            id: defaultValue
            z: defaultLabel.z + 10
            focus: false
            decimals        : spinBox.decimals
            stepSize        : spinBox.stepSize
            minimumValue    : spinBox.minimumValue
            maximumValue    : spinBox.maximumValue
            enabled         : false

            _backgroundBorderColor       : GEORC.COLOR.TRANSPARENT // gray in omny.qml
            _backgroundBorderHoveredColor: GEORC.COLOR.TRANSPARENT
            _backgroundBorderFocusColor  : GEORC.COLOR.TRANSPARENT
            _backgroundColor             : GEORC.COLOR.GRAY
            _backgroundFocusColor        : GEORC.COLOR.GRAY
            _textColor                   : GEORC.COLOR.WHITE
            _selectionColor              : GEORC.COLOR.GRAY
            horizontalAlignment : Qt.AlignHCenter
            anchors.fill: parent
            _borderWidth : 0
            _borderFocusWidth: _borderWidth
            font.bold: true
        }
    }
}
