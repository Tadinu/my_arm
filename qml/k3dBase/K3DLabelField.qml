import QtQuick 2.3
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import MainGBSingletonObject 1.0
import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL
import "qrc:///../K3DStudio/qml/k3dBase"
Rectangle {
    id: thisLabelField
    z: 10
    border.color: K3DRC.COLOR.GRAY // gray in omny.qml
    border.width: MAINGB.getRefSize(1)
    radius: MAINGB.getRefSize(3)
    color : K3DRC.COLOR.TRANSPARENT
    clip  : true

    property string _labelColor : K3DRC.COLOR.GREEN
    property alias  _text       : labelText.text
    property alias  _labelWidth : labelRect.width
    property alias  _labelText  : labelText
    Rectangle {
        id: labelRect
        z : 1
        anchors {
            left  : thisLabelField.left
            top   : thisLabelField.top
            bottom: thisLabelField.bottom
            margins: thisLabelField.border.width
        }
        radius: thisLabelField.radius - 1
        color: _labelColor
        width: labelText.contentWidth + height / 2 + radius
        K3DText {
            id: labelText
            font.bold: true
            font.letterSpacing: 0
            anchors.centerIn: parent
        }
    }

    property alias _value : textField.text
    property bool  _isReadOnly : true
    property string _unit : ""
    property alias _textField: textField
    property alias _valueColor: textField._textColor
    signal fieldHovered(bool hovered)
    K3DTextField {
        id: textField
        z : 5
        anchors {
            left        : labelRect.right
            leftMargin  : - labelRect.radius
            right       : thisLabelField.right
            rightMargin : labelRect.radius
            top         : thisLabelField.top
            topMargin   : thisLabelField.border.width
            bottom      : thisLabelField.bottom
            bottomMargin: thisLabelField.border.width
        }
        _backgroundColor   : K3DRC.COLOR.WHITE
        horizontalAlignment: Text.AlignHCenter;
        _implicitHeight    : height
        _implicitWidth     : width
        readOnly           : _isReadOnly
        _borderWidth       : 0
        _leftMargin        : 0
        _borderColor       : K3DRC.COLOR.TRANSPARENT
        _textColor         : K3DRC.COLOR.BLACK
        _selectedTextColor : K3DRC.COLOR.BLACK
        _selectionColor    : K3DRC.COLOR.LIGHTGRAY
        _fontSizeScale     : labelText._fontSizeScale

        onHoveredChanged : thisLabelField.fieldHovered(hovered);

//        onTextChanged: {
//            var index = text.indexOf(_unit);
//            if(index === -1) {
//                text = text + _unit;
//            }
//            else if(index !== text.length - 1) {
//                text = text = text.slice(0, index) + text.slice(index + 1) + _unit;
//            }
//        }
    }
}
