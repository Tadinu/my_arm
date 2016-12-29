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
    id: thisSpinBoxItem
    objectName : K3DRC.TEXT.K3D_SPINBOX_ITEM_OBJECT_NAME
    property string _name : ""

    // width To be set outside
    height: spinBox.height

    // [Spinbox Item -> Spin Volume Control]
    signal increVal()
    signal decreVal()
    property Item _spinControl
    onIncreVal: {
        //print("IncreaVal signal emit!");
        if(_spinControl != null) {
            _spinControl.increVol(); // -> [SIGNAL] valueUpdated(..) -> thisSpinBoxItem.setValue(..)
        }
    }

    onDecreVal: {
        if(_spinControl != null) {
            _spinControl.decreVol();
        }
    }

    function setVisible(isVisible) {
        visible = isVisible;
    }

    // [Spin Volume Control -> Spinbox Item]
    function setValue(value) {
        // !NOTE: [SIGNAL] thisSpinBoxItem.increVal(),decreVal() anyway comes back here!
        // BUT IF ONLY CHANGE => thisSpinBoxItem.increVal(),decreVal()
        // THEREFORE, NOT CAUSE A BINDING LOOP!
        _value = value;
    }

    // Field Name
    property alias _spinBoxName: spinBoxName.text
    property alias _spinBoxLabel: spinBoxName
    K3DText {
        id: spinBoxName
        anchors {
            left: thisSpinBoxItem.left
            verticalCenter: thisSpinBoxItem.verticalCenter
        }
        opacity: thisSpinBoxItem.enabled ? 1 : 0.5
    }

    // Field Unit
    property alias _spinBoxUnit: spinBoxUnit.text
    property alias _spinBoxUnitWidth : spinBoxUnit.width
    property real _spinBoxUnitMargin : 0
    K3DText {
        id: spinBoxUnit
        anchors {
            right: thisSpinBoxItem.right
            rightMargin: thisSpinBoxItem._focus ? 0 : _spinBoxUnitMargin
            verticalCenter: thisSpinBoxItem.verticalCenter
        }
        opacity: thisSpinBoxItem.enabled ? 1 : 0.5
    }

    // SpinBox
    property alias _spinBox : spinBox
    signal editingFinished()
    signal keysPressed(var event)

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
    property Item  _parentDialog    : null

    function setName(name) {
        _name = name;
    }

    function getName() {
        return _name;
    }

    function setSingleStep(singleStep) {
        _stepSize = singleStep;
    }

    function getDecimals() {
        return _decimals;
    }

    function setSpinboxFocus(focus) {
        spinBox.focus = focus;
        if(focus) {
            spinBox.forceActiveFocus();
        }
    }

    K3DSpinBox {
        id: spinBox
        _isInSpinBoxItem : true

        focus: false
        _backgroundBorderColor: "#67686A"
        _backgroundBorderHoveredColor: "#898989"
        _backgroundBorderFocusColor : "#E3E3E3"
        _backgroundColor    : "#1E1D1C"
        horizontalAlignment : Qt.AlignHCenter

        // width: To be set outside
        anchors {
            right: spinBoxUnit.left
            rightMargin: thisSpinBoxItem._unitRightMargin
        }

        Keys.onPressed: {
            thisSpinBoxItem.keysPressed(event);
            if(_parentDialog !== null) {
                switch(event.key) {
                case Qt.Key_Enter:                               // [ENTER/RETURN] --- //16777221
                    _parentDialog.applied();
                    break;

                case Qt.Key_Return: //16777220 - Main Enter Button on Keyboard
                    _parentDialog.applied();
                    break;

                case Qt.Key_F4:                                 // [F4] --------------
                    if (event.modifiers & Qt.AltModifier) {     // [ALT_F4]
                        _parentDialog.closed(); // -> MAINGB.focusMainWindow();
                    }
                    break;

                case Qt.Key_Escape:                             // [ESC] -------------
                    _parentDialog.closed();
                    break;

                default:
                    break;
                }
            }
        }

        property real _trackingValue : 0
        // !!!NOTE: THIS HELPS PREVENT CYCLIC HANDLING [BINDING LOOP]
        onValueChanged: {
            if(_spinBoxType === K3DRC.SPINBOX_TYPE.METRIC)
                MAINGB.validateSpinBoxItemParamsSingleStep(thisSpinBoxItem);

            //print('Value Change', _trackingValue, value, spinBox.value);
            //
            if(_trackingValue !== value) {
                if(value === _trackingValue + stepSize) {
                    thisSpinBoxItem.increVal();
                }
                else if(value === _trackingValue - stepSize) {
                    thisSpinBoxItem.decreVal();
                }

                _trackingValue = value;
            }
        }

        onEditingFinished : thisSpinBoxItem.editingFinished()

        onHoveredChanged: {
            if(_parentDialog !== null) {
                if(hovered) {
                    _parentDialog.activate(false);
                }
                if(_spinControl !== null) {
                    _spinControl.setHoverTargetItem(thisSpinBoxItem, hovered);
                    if(!hovered && focus) {
                        _spinControl.setFocusTargetItem(thisSpinBoxItem, focus);
                    }
                }
            }
        }
        onFocusChanged: {
            if(_parentDialog !== null && _spinControl !== null) {
                if(focus)
                    delaySetfocusTarget.start();
                else
                    _spinControl.setFocusTargetItem(thisSpinBoxItem, focus);
            }
        }

        // when we change focus from a spinbox to another one, signal focus = true of new spinbox focused comes earlier than signal focus = fasle of old one
        Timer{
            id: delaySetfocusTarget
            repeat: false
            triggeredOnStart: false
            running: false
            interval: 10
            onTriggered: {
                _spinControl.setFocusTargetItem(thisSpinBoxItem, true);
            }
        }
    }

}
