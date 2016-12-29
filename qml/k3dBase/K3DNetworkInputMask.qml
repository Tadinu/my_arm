import QtQuick 2.3
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import MainGBSingletonObject 1.0
import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL

Item {
    id: inputMaskItem

    property int _numberSpinboxs  : 4
    property real _minValue       : 0
    property real _maxValue       : 255
    property real _firstLastMinValue: _minValue
    property real _firstMaxValue: _maxValue
    property int  _maxItemText    : 3
    property real _fontSizeScale  : 1.5
    property string _seperatorText: "."
    property string _backgroundBorderColor        : K3DRC.COLOR.TRANSPARENT//"#67686A"
    property string _backgroundBorderHoveredColor : K3DRC.COLOR.TRANSPARENT//"#898989"
    property string _backgroundBorderFocusColor   : K3DRC.COLOR.TRANSPARENT//"#E3E3E3"
    property string _backgroundColor              : K3DRC.COLOR.TRANSPARENT//"#1E1D1C"
    property string _backgroundSource             : ""
    property bool   _isCorrectFormat              : false // (1.1.1.1)
    signal mouseIn()
    signal mouseOut()
    signal textChanged(var text)

    function setText(text) {
        if(K3DUTIL.TEXT.isEmpty(text))
            return;
        var itemText;
        for(var i = 0; i < _numberSpinboxs; i++) {
            itemText = text.split(_seperatorText)[i];
            inputRepeater.itemAt(i)._text = itemText === undefined ? "" : itemText;
        }
    }

    function setNextFocus(index) {
        if(index < (inputRepeater.model -1) ) {
            inputRepeater.itemAt(index + 1).setFocus();
        }
    }

    function setPreFocus(index) {
        if(index >= 1) {
            inputRepeater.itemAt(index -1 ).setFocus();
        }
    }

    Row {
        id: inputRow
        anchors.fill: inputMaskItem
        spacing: 0
        Repeater {
            id: inputRepeater
            model: _numberSpinboxs

            K3DRectangle {
                id: inputItem
                width               : inputMaskItem.width / inputRepeater.model
                height              : inputMaskItem.height
                color               : K3DRC.COLOR.TRANSPARENT
                _hoverEnabled       : true
                onBaseClicked       : itemText.forceActiveFocus()
                onBaseDoubleClicked : itemText.forceActiveFocus()
                onBaseEntered       : inputMaskItem.mouseIn()
                onBaseExited        : inputMaskItem.mouseOut()
                function setFocus() {
                    itemText.forceActiveFocus();
                }

                property alias _text: itemText.text
                K3DTextInput {
                    id: itemText
                    anchors.centerIn: parent
                    _implicitHeight : MAINGB._CDIALOG_SPINBOX_HEIGHT
                    _implicitWidth  : _implicitHeight * 3

                    _fontSizeScale      : inputMaskItem._fontSizeScale
                    _backgroundColor    : inputMaskItem._backgroundColor
                    horizontalAlignment : Qt.AlignHCenter
                    maximumLength       : inputMaskItem._maxItemText
                    //inputMask: "000;9"
                    inputMethodHints    : Qt.ImhDigitsOnly
                    validator: IntValidator{bottom: (index === 0 || index === _numberSpinboxs) ? _firstLastMinValue : _minValue;
                                            top: (index === 0) ? _firstMaxValue : _maxValue;}
                    onTextChanged: {
                        if(text.length === inputMaskItem._maxItemText) {
                            setNextFocus(index);
                        }
                        if(text.length === 0) {
                            setPreFocus(index);
                        }
                        var countError = 0;
                        var textTotal = "";
                        for(var i = 0; i < inputRepeater.model; i++) {
                            var itemText = (inputRepeater.itemAt(i)._text + "");
                            textTotal += itemText.trim();
                            if(itemText.trim().length === 0 || itemText.trim().length > 3)
                                countError += 1;
                            if(i !== (inputRepeater.model - 1))
                                textTotal += inputMaskItem._seperatorText;
                        }
                        _isCorrectFormat = countError === 0;
                        //print("TEXT CHANGE: ", textTotal);
                        inputMaskItem.textChanged(textTotal.trim());
                    }

                    Keys.onPressed: {
                        //print("CURSOR POISION: ", itemText.cursorPosition )
                        if( (event.key === Qt.Key_Tab || event.key === Qt.Key_Period) ||
                            (itemText.cursorPosition === itemText.length &&
                             event.key === Qt.Key_Right)                                ) {
                            setNextFocus(index);
                        }
                        else if( (event.key === Qt.Key_Backspace && itemText.text.length === 0) ||
                                 (itemText.cursorPosition === 0 && event.key === Qt.Key_Left)      ) {
                            setPreFocus(index);
                        }
                    }
                }

                K3DText {
                    anchors {
                        bottom        : itemText.bottom
                        right         : parent.right
                    }

                    visible: index !== (inputRepeater.model -1)
                    text: inputMaskItem._seperatorText
                }
            }
        }
    }
    K3DImage {
        id: backgroundImage
        z: parent.z -1
        anchors.fill: parent
        source: _backgroundSource
    }
}
