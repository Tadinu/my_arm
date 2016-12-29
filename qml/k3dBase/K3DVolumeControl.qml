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

// Volume Control
K3DRectangle {
    id: thisVolumeControl

    // To signal Target item updating its value
    signal valueUpdated(real value)

    // Target Item's value info:
    property real _targetRefValue   : 0 // Used to reference the external Control Item's value

    property real _CVOLUME_MINVALUE : 0
    property real _CVOLUME_MAXVALUE : 0
    property real _CVOLUME_STEP_SIZE : 0.1
    // ++ AS PEDRO SAID, THIS IS SET OUTSIDE & INDEPENDENT OF THE RANGE! ++
    // _CVOLUME_UNIT_VAL      = (maxValue - minValue)/_CMOVE_CONTROL_VOLUME_NUM;
    //volumeScheme._curIndex = (value - minValue)/_CVOLUME_UNIT_VAL - 1;
    // -- AS PEDRO SAID, THIS IS SET OUTSIDE & INDEPENDENT OF THE RANGE! --

    // --------------------------------------------------------------------------
    property real _centerX : thisVolumeControl.width/2
    property real _centerY : thisVolumeControl.height/2
    property real _posRadius   : 0.67*thisVolumeControl.radius
    property real _deltaRadiant: Math.PI/4
    property real _CMOVE_CONTROL_VOLUME_NUM: 60
    property real _CDELTA_VOLUME_ANGLE  : 360/_CMOVE_CONTROL_VOLUME_NUM
    property real _CDELTA_VOLUME_RADIANT: 2*Math.PI/_CMOVE_CONTROL_VOLUME_NUM
    //property real _COFFSET     : - thisMarkMenuControl._CMARK_MENU_BUTTON_SIZE/2

    smooth: true
    antialiasing: true
    _backgroundSource : _maskEnabled ? K3DRC.BACKGROUND.VOLUME_CONTROL_BACKGROUND : ""
    _maskEnabled      : thisVolumeControl.opacity !== 0
    onBaseWheel: {
        //if((wheel.buttons & Qt.MiddleButton) === Qt.MiddleButton) {
            if (wheel.angleDelta.y > 0) {
                //print('Set Value - IncreVol - go Shine:');
                thisVolumeControl.increVol();
            }
            else {
                //print('Set Value - DecreVol - go Dim:');
                thisVolumeControl.decreVol();
            }
        //}
    }

    onOpacityChanged: {
        //print('OPACITY CHANGED');
    }

    on_TargetRefValueChanged: {
        //print('RETURN', _targetRefValue);
    }

    function increVol() {
        volumeScheme.goShine(volumeScheme._curIndex + 1);
    }

    function decreVol() {
        volumeScheme.goDim(volumeScheme._curIndex);
    }

    property string _CVOID_COLOR : "gray"
    property string _CVALUE_COLOR: _CVOID_COLOR === "gray"    ? "white"   : // White
                                   _CVOID_COLOR === "white"   ? "#00D2FF" : // Blue
                                   _CVOID_COLOR === "#00D2FF" ? "orange"  : "transparent"

    function nextColorSet() {
        _CVOID_COLOR = _CVOID_COLOR === "gray"    ? "white"   : // White
                       _CVOID_COLOR === "white"   ? "#00D2FF" : // Blue
                       _CVOID_COLOR === "#00D2FF" ? "gray"    : "transparent";
        // _CVALUE_COLOR IS THEN UPDATED AT ONCE NOW...
        //print("NEXT COLOR SET: ",_CVOID_COLOR );
        for(var i = 0; i <= volumeScheme._topIndex; i++)
            volumeScheme.itemAt(i).color = thisVolumeControl._CVOID_COLOR;
    }

    function prevColorSet() {
        _CVOID_COLOR = _CVOID_COLOR === "gray"    ? "#00D2FF" : // Blue
                       _CVOID_COLOR === "#00D2FF" ? "white"   :
                       _CVOID_COLOR === "white"   ? "gray"    : "transparent";
        // _CVALUE_COLOR IS THEN UPDATED AT ONCE NOW...
        for(var i = 0; i <= volumeScheme._topIndex; i++)
            volumeScheme.itemAt(i).color = thisVolumeControl._CVALUE_COLOR;
    }

    // Volume Scheme -----------------------------------------------------------------------------
    Repeater {
        id: volumeScheme
        model: thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM

        property int _curIndex : 0
        property int _topIndex : model -1

        K3DRectangle {
            id: volumeUnit
            antialiasing: true
            smooth: true

            opacity: thisVolumeControl.opacity
            // --
            height : MAINGB._CDIALOG_TOUCH_VOLUME_UNIT_HEIGHT * ((index % 5 === 0)? 1 : 0.6)
            width: height/3.3

            rotation : index * thisVolumeControl._CDELTA_VOLUME_ANGLE // 3.6 (Degree) = 2*Math.PI/60 (Rad);
            color : (index === 0)? "orange" : thisVolumeControl._CVOID_COLOR
            property bool _isOn : color === thisVolumeControl._CVOID_COLOR // !!! BY DEFAULT AS FALSE

            x : thisVolumeControl._centerX - thisVolumeControl._posRadius* Math.cos(index * thisVolumeControl._CDELTA_VOLUME_RADIANT - Math.PI/2) - width/2;
            y : thisVolumeControl._centerY - thisVolumeControl._posRadius* Math.sin(index * thisVolumeControl._CDELTA_VOLUME_RADIANT - Math.PI/2) - height/2;

            // -- Mouse Area
            _baseMouseArea.hoverEnabled: true
            onBaseEntered: {
                //print('ENTER VOL:', index, volumeScheme._curIndex, volumeUnit._backgroundSource === K3DRC.ICON.DIALOG_VOLUME_UNIT_VOID);
                if (volumeUnit._isOn) {
                    //print('Enter Unit - go Dim:', index);
                    volumeScheme.goDim(index);
                }
                else {
                    //print('Enter Unit - go Shine:', index);
                    volumeScheme.goShine(index);
                }
            }

            onBaseExited: {
            }

            onBaseClicked: {
                volumeScheme.goShineUpto(index);
            }
        }

        function goShineUpto(index) {
            //print("Go Shine upto: ", index);
            if(volumeScheme._curIndex === index)
                return;

            if (index < thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM) {

                var newVal;
                var newIndexValue;
                if(volumeScheme._curIndex <= thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM/2) {
                    newIndexValue = index - volumeScheme._curIndex;
                }
                else {
                    if(index < thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM/2) { // '<' here instead of <= should be more appropriate!
                        newIndexValue = index + (thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM - volumeScheme._curIndex);
                        thisVolumeControl.nextColorSet();
                    }
                    else {
                        newIndexValue = index - volumeScheme._curIndex;
                    }
                }
                newVal = thisVolumeControl._targetRefValue + newIndexValue * thisVolumeControl._CVOLUME_STEP_SIZE;

                if(newVal <= thisVolumeControl._CVOLUME_MAXVALUE) {
                    // [SIGNAL]: valueUpdated(newValue)
                    thisVolumeControl.valueUpdated(newVal); // Update _targetRefValue indirectly through _targetDialogItem
                }
                else if(thisVolumeControl._targetRefValue !== thisVolumeControl._CVOLUME_MAXVALUE) {
                    thisVolumeControl.valueUpdated(thisVolumeControl._CVOLUME_MAXVALUE);
                }
                else
                    return;
                // Current vol pos:
                volumeScheme._curIndex = index;
                // Opacity:
                //volumeScheme.itemAt(index).opacity = 1;
                for(var i = 0 ; i <= volumeScheme._topIndex; i++)
                    volumeScheme.itemAt(i).color = (i <= index) ? thisVolumeControl._CVALUE_COLOR : thisVolumeControl._CVOID_COLOR;

            }
        }

        function goShine(index) {
            //print('GO SHINE', index, volumeScheme._curIndex);
            var nextIndex = volumeScheme._curIndex + 1 > volumeScheme._topIndex ? 0 : volumeScheme._curIndex + 1;
            if (index === nextIndex) {
                var increVal = thisVolumeControl._targetRefValue + thisVolumeControl._CVOLUME_STEP_SIZE;

                if(increVal <= thisVolumeControl._CVOLUME_MAXVALUE) {
                    // [SIGNAL]: valueUpdated(newValue)
                    thisVolumeControl.valueUpdated(increVal); // Update _targetRefValue indirectly through _targetDialogItem
                }
                else if(thisVolumeControl._targetRefValue !== thisVolumeControl._CVOLUME_MAXVALUE) {
                    thisVolumeControl.valueUpdated(thisVolumeControl._CVOLUME_MAXVALUE);
                }
                else
                    return;
                if(index < volumeScheme.count){
                    volumeScheme.itemAt(index).color = thisVolumeControl._CVALUE_COLOR;
                }

                // Current vol index:
                volumeScheme._curIndex = index;
                if(index === volumeScheme._topIndex) { // THIS HELPS NOT NEED TO CHECK INDEX <= index === volumeScheme._topIndex
                    volumeScheme._curIndex = 0;
                    thisVolumeControl.nextColorSet();
                }
            }
        }

        function goDim(index) {
            print('GO DIM', index, volumeScheme._curIndex);
            if (index === volumeScheme._curIndex) {
                var decreVal = thisVolumeControl._targetRefValue - thisVolumeControl._CVOLUME_STEP_SIZE;

                if(decreVal >= thisVolumeControl._CVOLUME_MINVALUE) {
                    // [SIGNAL]: valueUpdated(newValue)
                    thisVolumeControl.valueUpdated(decreVal);

                    if(index < volumeScheme.count) {
                        volumeScheme.itemAt(index).color = thisVolumeControl._CVOID_COLOR;
                    }

                    // Current vol index:
                    volumeScheme._curIndex = index - 1;
                    if(index === 0) {
                        volumeScheme._curIndex = volumeScheme._topIndex;
                        thisVolumeControl.prevColorSet();
                    }
                }
            }
        }
    } // End Repeater Volume Scheme

    K3DRectangle{
        id: volumeMask
        z : volumeScheme.z + 1
        anchors.centerIn: thisVolumeControl
        width          : _maskEnabled ? (thisVolumeControl._posRadius * 2 + MAINGB._CDIALOG_TOUCH_VOLUME_UNIT_HEIGHT * 0.8) : 0
        height         : width
        _maskEnabled   : thisVolumeControl._maskEnabled

        _hoverEnabled  : true
        //_baseMaskedMouseArea.anchors.fill : volumeMask
        //_baseMaskedMouseArea.maskSize     : Qt.size(markBrushMaskMouseRect.width, markBrushMaskMouseRect.height)

        _maskSource    : _maskEnabled ? K3DRC.BACKGROUND.VOLUME_CONTROL_MASKSOURCE : ""
        _baseMouseArea.enabled : false

        function calculateAngle(x, y) {
            x = x - width / 2;
            y = height / 2 - y;
            var angle = Math.atan(x / y) / Math.PI * 180;
            //print("ANGLE", angle);
            if( y > 0)
                 angle = angle + 180;
            else {
                if(x > 0)
                    angle = angle + 360;
                // x < 0 don't change
            }

            var index = Math.floor( angle/ 360.01 * thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM);
            //print("CALCULATED ANGLE: ", index);//, angle, thisVolumeControl._CMOVE_CONTROL_VOLUME_NUM, thisVolumeControl._posRadius);
            return index;
        }
        onBaseWheel: {
            thisVolumeControl.baseWheel(wheel);
        }

        property int _lastIndex : 0;
        //Change color of the ring following the position of mouse on wheel
        onBasePositionChanged: {
            var index = calculateAngle(mouse.x, mouse.y);
            if(index === volumeScheme._curIndex)
                return;


//            if(_lastIndex === volumeScheme._topIndex && index === 0) {
//                print('change color go shine 1');
//                volumeScheme.goShine(volumeScheme._topIndex);
//                _lastIndex = index;
//                return;
//            }

//            if (_lastIndex === 0 && index === volumeScheme._topIndex) {
//                print('change color go DIM 1');
//                volumeScheme.goDim (0);
//                _lastIndex = index;
//                return;
//            }
            //print('The last index', _lastIndex);
            var preIndex  = volumeScheme._curIndex - 1 < 0 ? volumeScheme._topIndex : volumeScheme._curIndex - 1;
            var nextIndex = volumeScheme._curIndex + 1 > volumeScheme._topIndex ? 0 : volumeScheme._curIndex + 1;

            //print("BASE POSITION CHANGED: ", index, volumeScheme._curIndex);
            if(index === preIndex && _lastIndex !== volumeScheme._topIndex) {
                //print('change color go DIM 2');
                volumeScheme.goDim(volumeScheme._curIndex);
                //print("GO DIM! : ", index, volumeScheme._curIndex, _lastIndex);
            }
            else if(index === nextIndex && _lastIndex !== 0){
                //print('change color go shine 2');
                volumeScheme.goShine(nextIndex);
                //print("GO SHINE! : ", index, volumeScheme._curIndex, _lastIndex);
            }
            _lastIndex = index;
        }

        onBasePressed: {
            volumeScheme.goShineUpto(calculateAngle(mouse.x, mouse.y));
            _lastPressedIndex = calculateAngle(mouse.x, mouse.y);
        }

        property real _lastPressedIndex: 0
        onBasePressedMoved: {
            var index = calculateAngle(mouse.x, mouse.y);

            if(index > _lastPressedIndex) {
                thisVolumeControl.increVol();
            }
            if(index < _lastPressedIndex) {
                thisVolumeControl.decreVol();
            }
            if(_lastPressedIndex !== index)
                _lastPressedIndex = index
        }
    }

    // Volume Value Label -----------------------------------------------------------------------------
    K3DText {
        anchors.centerIn: thisVolumeControl
        text: thisVolumeControl._targetRefValue

        color: "white"
        font.bold: true
        _fontSizeScale : 1.3
    }
}
