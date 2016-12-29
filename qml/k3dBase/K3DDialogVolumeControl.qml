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

K3DVolumeControl {
    id: thisDialogVolumeControl

    anchors {
        horizontalCenter : _targetDialog.horizontalCenter
        top              : _targetDialog.bottom
        topMargin        : thisDialogVolumeControl.height / 7.31
    }

    width : 0.9 * _targetDialog.width
    height: width
    radius: width/2
    enabled: _targetDialogItem !== null
    // --------------------------------------------------------------------------------------
    // [TARGET DIALOG]
    //
    property Item _targetDialog
    property Item _targetDialogItem // of the _targetDialog
    property Item _focusTargetDialogItem: null

    function setTargetDialogItem(item) {
        _focusTargetDialogItem = _targetDialogItem = item;
    }

    function setHoverTargetItem(spinbox, isHovered) {
        //print("Set hover target item: ", spinbox, isHovered, _focusTargetDialogItem);
        _targetDialogItem = isHovered ? spinbox : _focusTargetDialogItem;
    }

    function setFocusTargetItem(spinbox, isFocus) {
        //print("Set focus target item: ", spinbox, isFocus);
        setTargetDialogItem(isFocus ? spinbox : null);
    }

    _targetRefValue  : (_targetDialogItem !== null) ?
                        _targetDialogItem._value : 0

    _CVOLUME_MINVALUE : (_targetDialogItem !== null) ?
                        _targetDialogItem._minValue : 0
    _CVOLUME_MAXVALUE : (_targetDialogItem !== null) ?
                        _targetDialogItem._maxValue : 0
    _CVOLUME_STEP_SIZE: (_targetDialogItem !== null) ?
                        _targetDialogItem._stepSize : 0

    /* !NOTE: _targetDialogItem : ACTIVATED UPON [FOCUS || HOVERED]
      - FOCUS   : FOR KEEPING THE LATEST(CURRENTLY) FOCUSED CONTROL ITEM, WHICH IS UNIQUE.
      - HOVERED : FOR UPDATING THE CURRENTLY HOVERED CONTROL ITEM, WHICH CAN BE NULL OR UNIQUE.
    */

    // [Spin Volume Control -> Spinbox Item] ----------------------------------------------
    // BaseWheel -> cause Value Updated!
    //
    onValueUpdated: {
        //print('DIALOG CONTROL VALUE UPDATED', value);
        if(_targetDialogItem != null) {
            _targetDialogItem.setValue(value); // Update _targetRefValue
        }
    }

    onBaseWheel : {
        if(_targetDialogItem != null) {
            if(_targetDialog.opacity !== 1)
                _targetDialog.opacity = 1;
        }
    }

    // Swinging Ropes ---------------------------------------------------------------------
    K3DRectangle {
        id: leftSwingRope
        z: -1
        _backgroundSource : K3DRC.BACKGROUND.DIALOG_SWING_ROPE_LEFT
        width : thisDialogVolumeControl.width / 10.6 ; height : thisDialogVolumeControl.width / 3.5
        anchors {
            top       : thisDialogVolumeControl.top
            topMargin : -thisDialogVolumeControl.height / 7.31
            left      : thisDialogVolumeControl.left
            leftMargin: thisDialogVolumeControl.width / 6.5
        }
    }

    K3DRectangle {
        id: rightSwingRope
        z: -1
        _backgroundSource : K3DRC.BACKGROUND.DIALOG_SWING_ROPE_RIGHT
        width : thisDialogVolumeControl.width / 10.6 ; height : thisDialogVolumeControl.width / 3.5
        anchors {
            top        : thisDialogVolumeControl.top
            topMargin  : -thisDialogVolumeControl.height / 7.31
            right      : thisDialogVolumeControl.right
            rightMargin: thisDialogVolumeControl.width / 6.5
        }
    }

    // States ----------------------------------------------------------------------
    state : K3DRC.STATE.HIDDEN
    states: [
        State {
                name: K3DRC.STATE.SHOWN

                PropertyChanges {
                    target: thisDialogVolumeControl
                    height: width
                    opacity : 1
                }

                PropertyChanges {
                    target: leftSwingRope
                    height: thisDialogVolumeControl.width / 3.5
                }
                PropertyChanges {
                    target: rightSwingRope
                    height: thisDialogVolumeControl.width / 3.5
                }
        },

        State {
                name: K3DRC.STATE.HIDDEN

                PropertyChanges {
                    target: thisDialogVolumeControl
                    height: 0
                    opacity : 0
                }

                PropertyChanges {
                    target: leftSwingRope
                    height: 0
                }
                PropertyChanges {
                    target: rightSwingRope
                    height: 0
                }
        }
    ]

    transitions: [
        Transition {
            ParallelAnimation {
                PropertyAnimation { target: thisDialogVolumeControl
                                    property: opacity
                                    duration: 0
                }
                PropertyAnimation { targets: [leftSwingRope, rightSwingRope]
                                    property: "height";
                                    duration: 0
                }
            }
        }
    ]
}
