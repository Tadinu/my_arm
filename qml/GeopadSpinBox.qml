import QtQuick 2.0
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import MainGBSingletonObject 1.0
import "qrc:///javascript/qmlcommonresource.js" as GEORC
//import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import "qrc:///qml/k3dBase"
Rectangle {
    id: thisOmnySpinBox
    z: 10
    border.color: GEORC.COLOR.GRAY // gray in omny.qml
    border.width: MAINGB.getRefSize(1)
    radius: MAINGB.getRefSize(3)
    color : GEORC.COLOR.TRANSPARENT
    clip  : true

    property string _labelColor : GEORC.COLOR.GREEN
    property alias  _text       : labelText.text
    property alias  _labelWidth : labelRect.width
    property bool   _isLabelHorizontal: true
    property real   _leftMargin       : thisOmnySpinBox.height / 3
    Rectangle {
        id: labelRect
        z : 1
        anchors {
            left  : thisOmnySpinBox.left
            top   : thisOmnySpinBox.top
            bottom: thisOmnySpinBox.bottom
            margins: thisOmnySpinBox.border.width
        }
        clip: true
        radius: thisOmnySpinBox.radius - 1
        color: _labelColor
        width: labelText.contentWidth + labelText.height + radius * 2
        K3DText {
            id: labelText
            font.bold: true
            font.letterSpacing: 0
            anchors {
                verticalCenter: parent.verticalCenter
                left          : parent.left
                leftMargin    : _isLabelHorizontal ? ((labelRect.width - labelText.width) / 2) : _leftMargin
            }
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
        anchors {
            left        : labelRect.right
            leftMargin  : - labelRect.radius
            right       : thisOmnySpinBox.right
            rightMargin : thisOmnySpinBox.border.width
            top         : thisOmnySpinBox.top
            topMargin   : thisOmnySpinBox.border.width
            bottom      : thisOmnySpinBox.bottom
            bottomMargin: thisOmnySpinBox.border.width
        }
        focus: false
        minimumValue: 0
        maximumValue: 99999
        stepSize    : 1
        decimals    : 0
        _backgroundBorderColor       : GEORC.COLOR.GRAY // gray in omny.qml
        _backgroundBorderHoveredColor: GEORC.COLOR.GRAY
        _backgroundBorderFocusColor  : GEORC.COLOR.GRAY
        _backgroundColor             : GEORC.COLOR.WHITE
        _backgroundFocusColor        : GEORC.COLOR.LIGHTGRAY
        _textColor                   : GEORC.COLOR.BLACK
        _selectionColor              : GEORC.COLOR.GRAY
        horizontalAlignment : Qt.AlignHCenter

        _borderWidth     : 0
        _borderFocusWidth: _borderWidth

        onHoveredChanged   : mainMouseArea.enabled = !hovered // Do not know why not error
        onEditingFinished : thisOmnySpinBox.editingFinished()
    }
}
