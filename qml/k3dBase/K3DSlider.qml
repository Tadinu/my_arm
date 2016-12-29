/****************************************************************************
**
** Copyright (C) 2014 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the QtQuick module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia. For licensing terms and
** conditions see http://qt.digia.com/licensing. For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights. These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

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

K3DRectangle {
    id: thisSlider
    property real _sliderValueMin          : 0    // zmin
    property real _sliderValueMax          : 100  // zmax
    //property real _sliderValueRange        : _sliderValueMax - _sliderValueMin
    property real _sliderValue             : 0
    property real _CSLIDER_MIN_TIP_POS     : sliderBar.anchors.topMargin - sliderBar.width * 1.5 // (*)
    // (*) : //use sliderBar.width * 1.5 instead of sliderHandle.height/2 because sliderHandle.height change by sliderHandle._containMouse
    property real _cSLIDER_MAX_TIP_POS     : //_CSLIDER_MIN_TIP_POS + sliderBar.height // because sliderBar.height change when thisSlider shown or hidden
        ((gbMainToolBar.y - gbMainTopQuickMenuBar.height ) - (gbMainAppWindow._CMENUBAR_SLICEVIEW_BAR_SIZE * 2/3 * 12.5)
                - gbK3DBuilderSymbolButton.width/2.5) - sliderBar.anchors.bottomMargin - gbMainAppWindow._CMENUBAR_SLICEVIEW_BAR_SIZE / 3.5 * 3 / 2;
        // thisSlider.height - sliderHandle.height/2;

    property real _CSLIDER_DELTA_TIP_NOTCH : sliderBar.width * 1.5// // (*)  (1) - sliderHandle.height/2 in (1)&(2) are irrelevant!
    property real _CSLIDER_NOTCH_POS       : _cSLIDER_MAX_TIP_POS - _CSLIDER_DELTA_TIP_NOTCH

    property real _CSLIDER_MIN_NOTCH       : _CSLIDER_MIN_TIP_POS
    property real _cSLIDER_MAX_NOTCH       : _CSLIDER_NOTCH_POS - sliderBar.width *1.5 + sliderOnOffNotch.height /2// (*)  (2) // set position which make flip change here!!
    // - sliderBar.width * 1.5 make _cSLIDER_NOTCH_RANGE shorter, flip change when center of sliderHandle cross over center of sliderOnOffNotch
    property real _cSLIDER_NOTCH_RANGE     : _cSLIDER_MAX_NOTCH - _CSLIDER_MIN_NOTCH

    //objectName: "Slider"

    signal sliceViewActivated()
    signal valueChanged(real value)
    signal sliceViewFlipped(bool isOn) // => Call MainWindow::cmdSliceView() -> gbMainAppWindow.setSliceViewActivated(true/false)
    signal valueEditingFinished()
    signal sliderDragFinished()
    //clip: true

    function recalculateSliderValue(){
        var percentValue = (thisSlider._cSLIDER_MAX_NOTCH - sliderHandle.y) / thisSlider._cSLIDER_NOTCH_RANGE;
        //print("SLIDER PARA 1 : ", _sliderValue,_sliderValueMin, _sliderValueMax, _CSLIDER_MIN_NOTCH,_cSLIDER_MAX_NOTCH, sliderHandle.y, thisSlider._cSLIDER_NOTCH_RANGE);
        thisSlider._sliderValue =  (_sliderValueMin + (_sliderValueMax - _sliderValueMin) * percentValue);
        //print("SLIDER PARA 2 : ", _sliderValue,_sliderValueMin, _sliderValueMax, _CSLIDER_MIN_NOTCH,_cSLIDER_MAX_NOTCH, sliderHandle.y, thisSlider._cSLIDER_NOTCH_RANGE);
    }

    function onSliderHandlePositionChanged() {
        // In [ON] region --
        if(sliderHandle.y <= thisSlider._cSLIDER_MAX_NOTCH) {
            // [Flip On if being Off]
            if(thisSlider._isFlippedOn === false) {
                //print('SLIDER FLIPPED ON', sliderHandle.y);
                thisSlider.sliceViewFlipped(true);
            }

            // -------------------------------------------------------
            // [Recalculate slider value]
            thisSlider.recalculateSliderValue();

            // -------------------------------------------------------
            // The updated value would be best accessed through [thisSlider._sliderValue]
            thisSlider.valueChanged(thisSlider._sliderValue);
        }

        // In [OFF] region --
        else {
            // [Flip Off if being On]
            if(thisSlider._isFlippedOn === true) {
                //print('SLIDER FLIPPED OFF', sliderHandle.y);
                thisSlider.sliceViewFlipped(false);
            }
        }
    }

    // [Slider Bar] --------------------------------------------------------------------
    //
    property alias _sliderBackground: sliderBar._backgroundSource
    K3DRectangle {
        id: sliderBar

        _backgroundSource        : "qrc:///res/sliceviewbar/sliceViewSlider.svg"

        anchors.top              : thisSlider.top
        anchors.topMargin        : 20
        anchors.bottom           : thisSlider.bottom
        anchors.bottomMargin     : 20
        anchors.horizontalCenter : thisSlider.horizontalCenter

        width                    : thisSlider.width/3.5
    }
    // [Slider On Off Notch] -----------------------------------------------------------
    //
    function setFlipOn(isOn){ _isFlippedOn = isOn; }
    property bool _isFlippedOn : false
    K3DRectangle {
        id: sliderOnOffNotch
        anchors.horizontalCenter: thisSlider.horizontalCenter

        y: thisSlider._CSLIDER_NOTCH_POS // Just to define the UI pos of sliderOnOffNotch

        width : sliderBar.width*2
        height: width/3

        _backgroundSource: K3DRC.ICON.SLICE_VIEW_ON_OFF_NOTCH
//      color : "#2D2D2D"
//      radius: 3
//      border.width: 2
//      border.color: "#4D4D4D"
    } // End sliderOnOffNotch

    // [Slider Handle] ------------------------------------------------------------------
    //
    property alias _sliderHandle : sliderHandle
    // When load new Proj, flip if off, so slider need off too
    on_IsFlippedOnChanged: {
        if(!_isFlippedOn) {
            sliderHandle.y = thisSlider._cSLIDER_MAX_TIP_POS;
            setSliderRunStop();
        }
    }
    on_CSLIDER_NOTCH_RANGEChanged: {
        //print("SLIDER MAX NOTCH CHANGE: ",_isFlippedOn, _sliderValue, _cSLIDER_NOTCH_RANGE );
        if(_isFlippedOn)
            setValue(_sliderValue);
        else
            sliderHandle.y = thisSlider._cSLIDER_MAX_TIP_POS;
    }

    property real _lastYPos //:  thisSlider._cSLIDER_MAX_NOTCH
    function resetLastYpos() {
        thisSlider._lastYPos = Qt.binding(function() { return thisSlider._cSLIDER_MAX_NOTCH}) ;
    }

    property bool _isOnTop      : false
    property bool _isOnBottom   : false
    property string _metricUnit : K3DRC.TEXT.MILLIMETER

    function setSliderSpinBoxFocusOut() {
        //print("SET SPINBOX FOCUS OUT: ", valueSpinBox._value, _sliderValue);
        // This function be called in refreshUI(). so when spinbox editing finish by press key Enter, it will be called.
        valueSpinBox._focus = false; // It make spinbox go to none editing state.
        gbMainAppWindow.forceFocus(); // !!!! TO CATCH KEY EVENT (UP,DOWN KEYS)// Just when use thisSlider in K3DStudio
    }
    property bool _startValueChange : false // just for stop error when it be compiled
    K3DSpinBoxItem {
        id: valueSpinBox
        focus                             : false
        height                            : MAINGB._CDIALOG_SPINBOX_HEIGHT
        enabled                           : thisSlider._isFlippedOn
        visible                           : thisSlider._isFlippedOn
        opacity                           : thisSlider._isFlippedOn ? 1 : 0
        _spinBox.width                    : thisSlider._isFlippedOn ? height * 3 : 0
        width                             : _spinBox.width
        anchors {
            verticalCenter                : sliderHandle.top
            verticalCenterOffset          : sliderBar.width * 1.5 //use it instead of sliderHandle.height/2 because it.height change by _containMouse
            left                          : thisSlider.right
            leftMargin                    : height / 2
        }
        _spinBox.enabled: !thisSlider._isSliderRunning
        property real _currentSliderValue : K3DUTIL.Math.round10(thisSlider._sliderValue, -_decimals)
        on_CurrentSliderValueChanged: {
            //print("CURRENT SLIDER VALUE CHANGE: ", _currentSliderValue );
            _value = _currentSliderValue;
        }

        _value                             : _currentSliderValue
        _minValue                          : (-999 < _sliderValueMin) ? -999 : _sliderValueMin //  IN QT 5.4.0, if set _minValue = 5, we cannot type 1-4
        _maxValue                          : (999 > _sliderValueMax) ? 999 : _sliderValueMax
        _stepSize                          : _decimals === 4 ? 0.0025 : 0.025
        _decimals                          : _metricUnit === K3DRC.TEXT.MILLIMETER ? 3 : 4
        property real _valueInRange        : 0

        on_ValueChanged: {
            //print("SLIDER VALUE 1: ",_currentSliderValue, _value, _valueInRange);
            if(_value === _currentSliderValue || !_startValueChange)
                return;

            _valueInRange = _value;
            if(_valueInRange < _sliderValueMin) _valueInRange = _sliderValueMin;
            if(_valueInRange > _sliderValueMax) _valueInRange = _sliderValueMax;
            //print("SET VALUE 2: ", _currentSliderValue, _value, _valueInRange);
            thisSlider.setValue(_valueInRange);
            thisSlider.valueChanged(_valueInRange);
        }

        onEditingFinished: {
            //thisSlider.valueEditingFinished();
            //print("SLIDER VALUE 3: ",_currentSliderValue, _value, _valueInRange);
            if(_value > _sliderValueMax || _value < _sliderValueMin)
                _value = _currentSliderValue;
            thisSlider.valueChanged(_valueInRange);
        }

        _spinBoxToolTip               : _metricUnit
    }

    property bool _isSliderShown : false
    property bool _isSliderPressed : false

    K3DButton {
        id: sliderHandle

//        border.width: 1
//        border.color: "white"
        anchors.horizontalCenter: thisSlider.horizontalCenter

        _btnSourceSize   : _isSliderShown ? sliderBar.width * (_containsMouse ?  3.6 : 3) : 0
        Behavior on _btnSourceSize { PropertyAnimation { duration : 100 } }
        _btnPressedSource: _btnNormalSource
        _btnHoveredSource: _btnNormalSource
        _iconSource      : thisSlider._isFlippedOn ?
                           K3DRC.ICON.SLICE_VIEW_HANDLE_ON :
                           K3DRC.ICON.SLICE_VIEW_HANDLE_OFF
        _btnType         : _CBUTTON_TYPE_ICON_ONLY | _CBUTTON_TYPE_ABNORMAL

        //_btnTooltip      : thisSlider._isFlippedOn?  Math.round(thisSlider._sliderValue *1000) / 1000 : ""
        _btnTooltipShownAllowed : false//thisSlider._isFlippedOn
        //_btnTooltipPos   : K3DRC.TOOLTIP_POS.RIGHT

//        property real _startY
//        property real _destY

        enabled: !_isSliderRunning
        y: thisSlider._cSLIDER_MAX_TIP_POS
        onYChanged: {
            if(_lastYPos !== sliderHandle.y && sliderHandle.y <= _cSLIDER_MAX_NOTCH && sliderHandle.y >= _CSLIDER_MIN_NOTCH && _isFlippedOn) {
                _lastYPos = sliderHandle.y;
            }
        }

        Component.onCompleted: {
            //sliderHandle.y = thisSlider._cSLIDER_MAX_TIP_POS;
//            _startY = thisSlider._CSLIDER_MIN_NOTCH;
//            _destY  = thisSlider._cSLIDER_MAX_NOTCH;
        }
        onMouseIn: {
        }

        onClicked: {
            thisSlider.sliceViewActivated();
        }

        // -----------------------------------------------------------------------------
        // HANDLE MOUSE AREA --
        //
        _isDraggable                      : true
        _touchButtonMouseArea.drag {
            //target   : sliderHandle // ALREADY SET INSIDE K3DButton using _isDraggable
            axis     : Drag.YAxis
            minimumY : thisSlider._CSLIDER_MIN_NOTCH
            maximumY : thisSlider._cSLIDER_MAX_TIP_POS // [thisSlider._cSLIDER_MAX_NOTCH]
        }
        onPressed: {
            thisSlider.setSliderSpinBoxFocusOut();
            _isSliderPressed = true;
            setSliderRunStop();
        }
        onReleased: {
            _isSliderPressed = false;
        }

        property bool _isDragging: false
        _touchButtonMouseArea.onPositionChanged: {
            //print("_touchButtonMouseArea.pressed : ",_touchButtonMouseArea.pressed, _sliderValue);
            if(_touchButtonMouseArea.pressed) {
                dropSliderTimer.stop();
                thisSlider.onSliderHandlePositionChanged();
                dropSliderTimer.start();
            }
        }

        Timer {
            id: dropSliderTimer
            repeat: false
            running: false
            triggeredOnStart: false
            interval: 250
            onTriggered: {
                thisSlider.sliderDragFinished();
            }
        }

        _touchButtonMouseArea.onDoubleClicked: {
            //print("SLIDER Double Clicked: ",thisSlider._lastYPos, _CSLIDER_MIN_NOTCH, _cSLIDER_MAX_NOTCH, _cSLIDER_MAX_TIP_POS );
            if(thisSlider._isFlippedOn === false) {
                sliderHandle.y = thisSlider._lastYPos;
                // Make sure that _lastYPos binding with _cSLIDER_MAX_NOTCH to correct value of its. If not, in first times, double click make y out of range -> lost sliderHande.
                thisSlider.sliceViewFlipped(true);
            }
            else if(thisSlider._isFlippedOn === true) {
                sliderHandle.y = thisSlider._cSLIDER_MAX_TIP_POS;
                thisSlider.sliceViewFlipped(false);
            }
        }
        _cursorShape : _touchButtonMouseArea.pressed       ? Qt.ClosedHandCursor : Qt.OpenHandCursor
        // -----------------------------------------------------------------------------
    } // End sliderHandle
    Component.onCompleted: {
        _startValueChange = true;
    }

    function setValue(value) {
        //console.log('Set SLice View Value C++:', value, _sliderValueMin, _sliderValueMax);
        if (value < _sliderValueMin || value > _sliderValueMax)
            return;
        if(_sliderValue !== value)
            _sliderValue = value;
        var deltaY = Math.abs(thisSlider._cSLIDER_NOTCH_RANGE);

        sliderHandle.y = _cSLIDER_MAX_NOTCH - Math.abs((_sliderValue - _sliderValueMin) / (_sliderValueMax - _sliderValueMin)) * deltaY;
        //print('K3DSlider-SetValue: sliderHandle.y:',sliderHandle.y);
        if(sliderHandle.y < thisSlider._CSLIDER_MIN_NOTCH) // <-> MUST BE EQUAL handleMouseArea.drag.MaximumY
            sliderHandle.y = thisSlider._CSLIDER_MIN_NOTCH;
        else if(sliderHandle.y > thisSlider._cSLIDER_MAX_NOTCH) // <-> MUST BE EQUAL handleMouseArea.drag.MaximumY
            sliderHandle.y = thisSlider._cSLIDER_MAX_NOTCH;
        //print("SLIDER HANDLE Y: ",sliderHandle.y ,deltaY, _cSLIDER_MAX_NOTCH, _CSLIDER_MIN_NOTCH);
        // !NOTE:
        // thisSlider._sliderValue would be updated accordingly due to binding!
    }

    //   ------------------------------------------------------------------------------------
    // SLICE RUN

    property real _thickness               : 0.025//_k3dMainWindow.getTotalSlices(false, _thickness) -> totalSlices
    property real _durationPerSlice        : 50

    property bool _sliderRunAble : false
    property bool _isRunningToTop : true
    property bool _isRunningToBot : false
    property bool _isSliderRunning : false
    property bool _isPaused        : false
    signal slicePlay(bool isPlaying)

    on_SliderValueChanged: {
        if(_isSliderRunning) {
            thisSlider.setValue(thisSlider._sliderValue)
            thisSlider.valueChanged(thisSlider._sliderValue);
        }

        _isOnTop    = thisSlider._sliderValue === thisSlider._sliderValueMax;
        if(_isOnTop) {
            setSliderRunStop();
            thisSlider._isRunningToTop = false;
            thisSlider._isRunningToBot = true;
        }

        _isOnBottom = thisSlider._sliderValue === thisSlider._sliderValueMin;
        if(_isOnBottom) {
            setSliderRunStop();
            thisSlider._isRunningToBot = false;
            thisSlider._isRunningToTop = true;
        }
    }

    on_IsSliderRunningChanged: {
        thisSlider.slicePlay(_isSliderRunning);
    }

    //property real _newValue
    //property real _origValue
    function jumpSingleSlice(goUp) {
        var newValue;
        //_origValue = _sliderValue;
        var thickness = MAINGB.getter(_thickness);
        if(goUp)
            newValue = (_sliderValue + thickness) > _sliderValueMax ? _sliderValueMax : (_sliderValue + thickness);
        else
            newValue = (_sliderValue - thickness) < _sliderValueMin ? _sliderValueMin : (_sliderValue - thickness);
        //runAnimation.start();
        thisSlider.setValue(newValue);
        thisSlider.valueChanged(newValue);
    }

    //NumberAnimation { id: runAnimation; target: thisSlider; property: "_sliderValue"; from: _origValue; to: _newValue; duration: _durationPerSlice;}

    Timer {
        id: runningTimer
        triggeredOnStart: false
        running: false
        repeat: true
        interval: _durationPerSlice
        onTriggered: {
//            if(runAnimation.running)
//                runAnimation.complete();
            jumpSingleSlice(thisSlider._isRunningToTop);
        }
    }

    function runSliderUpward() {
        thisSlider._isRunningToTop = true;
        jumpSingleSlice(thisSlider._isRunningToTop);
        runningTimer.start();
        thisSlider._isSliderRunning = true;
        thisSlider._isPaused = false;
    }

    function runSliderDownward() {
        thisSlider._isRunningToBot = true;
        jumpSingleSlice(thisSlider._isRunningToTop);
        runningTimer.start();

        thisSlider._isSliderRunning = true;
        thisSlider._isPaused = false;
    }

    function setSliderRunStop() {
        if(runningTimer.running)
            runningTimer.stop();

        thisSlider._isSliderRunning = false;
        thisSlider._isPaused = false;
    }

    function setSliderPause() {
        if(runningTimer.running)
            runningTimer.stop();
        thisSlider._isSliderRunning = false;
        thisSlider._isPaused = true;
    }

    // ---------------------------------------------------------------------------------------------------------------------
    // -- STATES
    //http://qt-project.org/doc/qt-5/qtquick-statesanimations-animations.html
    //http://qt-project.org/doc/qt-5/qtquick-usecase-animations.html
    //http://qt-project.org/doc/qt-5/qml-qtquick-scriptaction.html
    //http://qt-project.org/doc/qt-4.8/qml-propertyanimation.html#details
    //http://qt-project.org/doc/qt-4.8/qdeclarativeanimation.html
    //property variant _CSTATES : ["StartV", "DestV"]

}
