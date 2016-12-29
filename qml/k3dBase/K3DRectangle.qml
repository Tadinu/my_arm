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

Rectangle {
    id: thisRectangle
    //width: backgroundImage.width; height: backgroundImage.height
    objectName: K3DRC.TEXT.K3D_RECTANGLE_OBJECT_NAME
    antialiasing: true
    smooth      : true

    property int  _itemId: -1 // Agent ID
    property var  _itemAgent: null // Register for each K3DRectangle's Child class peculiar regItemAgent()!
    // C++ Agent for QML Item
    function regItemAgent(itemAgent) {
        //print('REGITEM AGENT', itemAgent);
        _itemAgent = itemAgent;
    }

    property int  _CK3DITEMID   : 0 // K3DQMLAdapter.K3D_ACTION_VOID

    // !NOTE: NEVER EVER EVER PUT _agentClicked directly bound to MAINGB._K3DACTIONS[] IN CHILD CLASS
    // SINCE THAT CHILD COULD RESIDE INSIDE A COMPONENT, THEREFORE CAN BE RELOADED INTO A LOADER, WHICH COULD
    // CAUSE POINTLESS _agentClickedChanged SIGNAL!!!
    //
    property bool _agentEnabled : true
    on_AgentEnabledChanged: {
        enabled = _agentEnabled;
    }
    property bool _agentClicked// : false
    on_AgentClickedChanged: {
        print('K3DRECT CLICKED', _itemId);
        if(/*thisRectangle.enabled && */_k3dMainWindow.isK3DActionValid(_CK3DITEMID)) { // Check again on valid action!
            thisRectangle.baseClicked(null);
        }
    }

    // Delete on Close
    property bool _isDeletedOnClosed : false // BY DEFAULT: false, ONLY true for Modal Dialog!

    // Inside a Loader
    property bool _isInLoader       : false

    // Modality
    property bool _isModal          : false // By default as MODELESS

    // Closed state
    property bool _isClosed         : false

    // Activated state
    property bool _isActivated      : false

    // [SIGNAL] : Write code to make use of C++ instance in onInitialized()
    signal initialized()
    //
    onInitialized: {
        thisRectangle.initializeAgentBindings();
    }

    // Agents Binding
    property bool _isAgentAlreadyBinding :false
    function initializeAgentBindings() {
        if(_CK3DITEMID > K3DQMLAdapter.K3D_ACTION_VOID && _CK3DITEMID < K3DQMLAdapter.K3D_ACTION_TOTAL &&
           MAINGB._K3DACTIONS[_CK3DITEMID] !== undefined) {
            if(_isAgentAlreadyBinding)
                return;
            _isAgentAlreadyBinding = true;

            _agentClicked = MAINGB._K3DACTIONS[_CK3DITEMID].clicked
                          = false;

            //print('K3DITEMID', _CK3DITEMID,  MAINGB._K3DACTIONS[_CK3DITEMID]);
            _agentEnabled = Qt.binding(function actionEnabled() { return MAINGB._K3DACTIONS[_CK3DITEMID].enabled; });
            _agentClicked = Qt.binding(function actionClicked() { return MAINGB._K3DACTIONS[_CK3DITEMID].clicked; });
        }
    }

    Connections {
        target: MAINGB.getMainWindow()
        ignoreUnknownSignals : true
        onDynamicComponentsInitialized: {
            thisRectangle.initialized();
        }
    }

    Component.onCompleted: {
        thisRectangle.initializeAgentBindings();
    }


    // =============================================================================================
    // COMMON METHODS ------------------------------------------------------------------------------
    //
    function show() {
        visible = _isActivated = true;
    }

    function hide() {
        visible = _isActivated = false;

    }

    function isModal() {
        return _isModal;
    }

    function isHidden() {
        return !visible;
    }

    function isVisible() {
        return visible;
    }

    function isClosed() {
        return _isClosed;
    }

    function isDeletedOnClosed() {
        return _isDeletedOnClosed;
    }

    function resize(w, h) {
        width  = w;
        height = h;
    }

    function size() {
        return Qt.size(width, height);
    }

    function move(posX, posY) {
        x = posX;
        y = posY;
    }

    function pos() {
        return Qt.point(x, y);
    }

    function isActivated() {
        return _isActivated;
    }

    function setActivated(isActivated) {
        _isActivated = isActivated;
    }

    // HIGHLIGHT BORDER ON HOVERING ================================================================
    //
    property int    _CORIG_BORDER_WIDTH : 0
    property string _CORIG_BORDER_COLOR : "#939393" // "#4D5859" //#464342"//"#F7F7F7" - "#"+ Math.floor(Math.random()*16777215).toString(16)
    border.width: _CORIG_BORDER_WIDTH
    border.color: _CORIG_BORDER_COLOR

    property bool  _isHighlightBorderOnHover     : false
    property bool  _isHighlightBorderOnSelection : false
    property int    _CHIGHLIGHT_BORDER_WIDTH : 3
    property string _CHIGHLIGHT_BORDER_COLOR : _CORIG_BORDER_COLOR

    Behavior on border.width {
        enabled: _isHighlightBorderOnHover || _isHighlightBorderOnSelection
        PropertyAnimation { duration: 200 }
    }

    // HIGHLIGHT BORDER ANIMATION ==================================================================
    //
    property alias _highLightAnimation : highLightAnimation
    SequentialAnimation {
        id: highLightAnimation

        loops: 3
        ScriptAction {
            script: {
                thisRectangle.border.color = _CHIGHLIGHT_BORDER_COLOR;
            }
        }

        NumberAnimation { target: thisRectangle; properties: "border.width"; from: 0; to: thisRectangle._CHIGHLIGHT_BORDER_WIDTH; duration: 200; }
        NumberAnimation { target: thisRectangle; properties: "border.width"; from: thisRectangle._CHIGHLIGHT_BORDER_WIDTH; to: 0; duration: 200; }

        ScriptAction {
            script: {
                thisRectangle.border.color = _CORIG_BORDER_COLOR;
                thisRectangle.border.width = _CORIG_BORDER_WIDTH;
            }
        }
    }

    // HIGHLIGHT BACKGROUND ANIMATION ==============================================================
    //
    property alias _highLightAnimation2 : highLightAnimation2
    property real _CORIG_WIDTH
    property real _CORIG_HEIGHT
    SequentialAnimation {
        id: highLightAnimation2

        loops: 3

        NumberAnimation { target: thisRectangle; properties: "opacity";  from: 1;  to: 0.7;  duration: 100; }
        NumberAnimation { target: thisRectangle; properties: "opacity";  from: 0.7;  to: 1;  duration: 100; }
    }

    // SIGNALS =====================================================================================
    //
    signal baseClicked(var mouse)
    signal baseRightClicked(var mouse)
    signal baseMiddleClicked(var mouse)

    signal baseEntered
    signal baseExited
    signal basePressed(var mouse)
    signal baseReleased(var mouse)
    signal basePressAndHold(var mouse)
    signal baseWheel(var wheel)
    signal baseDoubleClicked(var mouse)
    signal basePositionChanged(var mouse)
    signal basePressedMoved(var mouse)

    property bool _stayPut          : false

    onBasePressed: {
        MAINGB.onItemPressed(_CK3DITEMID);
        // Refresh UI
        if(!thisRectangle._stayPut)
            MAINGB.refreshMainWindowUI();

        // ----------------------------------------------------------
        // Get Support Widget info for Draggable Item.
        if(_isDraggable) {
            MAINGB.getSupportWidgetInfo();

            //if(_maskEnabled)
            //    maskedMouseArea.dragReftRectVisible = _k3dMainWindow.isSupportWidgetVisible();
        }
    }

    // BACKGROUND IMAGE ============================================================================
    //
    color: K3DRC.COLOR.TRANSPARENT
    property alias _backgroundSource : backgroundImage.source
    property alias _rotation         : backgroundImage.rotation
    property alias _backgroundImageZ : backgroundImage.z
    property alias _backgroundImage  : backgroundImage

    // NORMAL IMAGE --------------------------------------------------------------------------------
    K3DImage {
        id: backgroundImage

        anchors.fill      : thisRectangle
        anchors.margins   : thisRectangle.border.width
        fillMode          : Image.Stretch // Image.PreserveAspectFit

        _sourceWidth      : thisRectangle.width  - thisRectangle.border.width
        _sourceHeight     : thisRectangle.height - thisRectangle.border.height
        //z: 2
    }

    /* BORDER IMAGE --------------------------------------------------------------------------------
    //Background Border Image
    BorderK3DImage {
        id : backgroundImage
        border { left: 2; top: 2; right: 2; bottom: 2 }
        anchors.fill: thisRectangle
        horizontalTileMode: BorderImage.Stretch
        verticalTileMode: BorderImage.Stretch
        smooth: true
        antialiasing: true
    }
    */

    /* ANIMATED IMAGE ------------------------------------------------------------------------------
    property alias _backgroundAnimatedSource : backgroundAnimatedImage.source
    AnimatedK3DImage {
        id: backgroundAnimatedImage
        anchors.fill: thisRectangle
        visible: source !== ""
        fillMode: Image.Stretch
    }
    */

    // MOUSE AREAS =================================================================================
    //
    property bool  _mouseAreaEnabled : true // Global flag to veto the MouseArea/MaskedMouseArea
    property int   _cursorShape   : _hoverEnabled ? Qt.PointingHandCursor : Qt.ArrowCursor
    property bool  _hoverEnabled  : false // By default, no Enter or Exit Event
    property bool  _isDraggable   : false

    property bool  _pressed       : thisRectangle._maskEnabled ? maskedMouseArea.pressed :
                                    mouseArea.pressed
    property bool _isWheelPropagated: false
    property bool _isPropagateComposedEvents : true

    // Drag Target --
    property Item _dragTarget     : thisRectangle._isDraggable ? thisRectangle : null

    property real _origDragMinimumX   : 0
    property real _origDragMaximumX   : 0
    property real _origDragMinimumY   : 0
    property real _origDragMaximumY   : 0

    property real _curDragMinimumX    : _origDragMinimumX
    property real _curDragMaximumX    : _origDragMaximumX
    property real _curDragMinimumY    : _origDragMinimumY
    property real _curDragMaximumY    : _origDragMaximumY

    property real _dragMinimumX       : _curDragMinimumX
    property real _dragMaximumX       : _curDragMaximumX
    property real _dragMinimumY       : _curDragMinimumY
    property real _dragMaximumY       : _curDragMaximumY

    property real _dragThreshold      : 1

    property real _offsetMinX : 0
    property real _offsetMaxX : _dragTarget !== null ? _dragTarget.width  : 0
    property real _offsetMinY : 0
    property real _offsetMaxY : _dragTarget !== null ? _dragTarget.height : 0

    property real _CREF_RECTX       : MAINGB._CSUPPORT_WIDGET_X
    property real _CREF_RECTY       : MAINGB._CSUPPORT_WIDGET_Y
    property real _CREF_RECT_WIDTH  : MAINGB._CSUPPORT_WIDGET_WIDTH
    property real _CREF_RECT_HEIGHT : MAINGB._CSUPPORT_WIDGET_HEIGHT

    property real _cRefMinX : x - _offsetMinX
    property real _cRefMaxX : x + _offsetMaxX
    property real _cRefMinY : y - _offsetMinY
    property real _cRefMaxY : y + _offsetMaxY

    function runMoveAnimationForSliceViewBar(targetX) {
        thisRectangle._animationTargetX = targetX;
        moveLeftMarginAnimation.start();
    }

    property real _animationTargetX
    NumberAnimation {
        id: moveLeftMarginAnimation
        target: thisRectangle
        property: "x"
        from : thisRectangle.x
        to   : thisRectangle._animationTargetX +
               thisRectangle._offsetMinX
        duration : 100
    }

    onBasePositionChanged: {
        if(false === _isDraggable)
            return;

        // [Normal Mouse Area] ---------------------------------------------------------------------
        //
        if(false === _maskEnabled) {
            _curDragMinimumX = _origDragMinimumX;
            _curDragMaximumX = _origDragMaximumX;
            _curDragMinimumY = _origDragMinimumY;
            _curDragMaximumY = _origDragMaximumY;
            /*
            if(false === _k3dMainWindow.isSupportWidgetVisible()) {
                _curDragMinimumX = _origDragMinimumX;
                _curDragMaximumX = _origDragMaximumX;
                _curDragMinimumY = _origDragMinimumY;
                _curDragMaximumY = _origDragMaximumY;

                return;
            }

            var isXTotallyInside = (_dragTarget.x >= _CREF_RECTX-1 + _offsetMinX) &&
                                   (_dragTarget.x <= _CREF_RECTX+1 + _CREF_RECT_WIDTH  - _offsetMaxX);
            var isYTotallyInside = (_dragTarget.y >= _CREF_RECTY-1 + _offsetMinY) &&
                                   (_dragTarget.y <= _CREF_RECTY+1 + _CREF_RECT_HEIGHT - _offsetMaxY);

            if(isXTotallyInside && isYTotallyInside) {
                thisRectangle.move(MAINGB._QAPP_WIDTH/2, MAINGB._QAPP_HEIGHT/2);
                return;
            }

            // -------------------------------------------------------------------------------------
            //
            var isXInside = (_dragTarget.x >= _CREF_RECTX-1 - _offsetMaxX) &&
                            (_dragTarget.x <= _CREF_RECTX+1 + _CREF_RECT_WIDTH  + _offsetMinX);
            var isYInside = (_dragTarget.y >= _CREF_RECTY-1 - _offsetMaxY) &&
                            (_dragTarget.y <= _CREF_RECTY+1 + _CREF_RECT_HEIGHT + _offsetMinY);

            //print('DRAG POS:', _dragTarget.x, _dragTarget.y, _CREF_RECTX, _CREF_RECTY, _CREF_RECTX + _CREF_RECT_WIDTH, _CREF_RECTY + _CREF_RECT_HEIGHT);

            if(isXInside) {
                if(_dragTarget.y <= _CREF_RECTY - _offsetMaxY) {
                    _curDragMinimumY = MAINGB._CGBDRAG_ZONE_MIN_Y  + _offsetMinY;
                    _curDragMaximumY = _CREF_RECTY                 - _offsetMaxY;
                }
                else if(_dragTarget.y >= _CREF_RECTY-1 + _CREF_RECT_HEIGHT + _offsetMinY) {
                    _curDragMinimumY = _CREF_RECTY + _CREF_RECT_HEIGHT     + _offsetMinY;
                    _curDragMaximumY = MAINGB._CGBDRAG_ZONE_MAX_Y          - _offsetMaxY;
                }
            }
            else {
                _curDragMinimumY = _origDragMinimumY;
                _curDragMaximumY = _origDragMaximumY;
            }

            if(isYInside) {
                if(_dragTarget.x <= _CREF_RECTX - _offsetMaxX) {
                    _curDragMinimumX = MAINGB._CGBDRAG_ZONE_MIN_X + _offsetMinX;
                    _curDragMaximumX = _CREF_RECTX              - _offsetMaxX;
                }
                else if(_dragTarget.x >= _CREF_RECTX-1 + _CREF_RECT_WIDTH + _offsetMinX) {
                    _curDragMinimumX = _CREF_RECTX + _CREF_RECT_WIDTH     + _offsetMinX;
                    _curDragMaximumX = MAINGB._CGBDRAG_ZONE_MAX_X - _offsetMaxX;
                }
            }
            else {
                _curDragMinimumX = _origDragMinimumX;
                _curDragMaximumX = _origDragMaximumX;
            }
            */
        }

        // [Masked Mouse Area] ---------------------------------------------------------------------
        //
        // DONE IN C++ (K3DMaskedMouseArea)
    }

    // NORMAL MOUSE AREA ---------------------------------------------------------------------------
    //
    property bool  _containsMouse : false//thisRectangle._maskEnabled ? maskedMouseArea.containsMouse :
                                    //mouseArea.containsMouse

    property alias _baseMouseArea : mouseArea
    property alias _baseMouseAreaZ: mouseArea.z

    MouseArea { // Not use K3DMouseArea here!!!
        id: mouseArea
        //z : 1
        anchors.fill: thisRectangle // This must fill Rectangle since backgroundImage can be null!
//        enabled     : (thisRectangle.objectName === K3DRC.TEXT.K3D_RECTANGLE_OBJECT_NAME) ?
//                      hoverEnabled : false
        enabled: thisRectangle._mouseAreaEnabled &&
                 (false === thisRectangle._maskEnabled)

        propagateComposedEvents : thisRectangle._isPropagateComposedEvents
        acceptedButtons: Qt.AllButtons

        // Drag
        drag.target     : thisRectangle._maskEnabled ? undefined : thisRectangle._dragTarget

        drag.minimumX   : thisRectangle._dragMinimumX
        drag.maximumX   : thisRectangle._dragMaximumX
        drag.minimumY   : thisRectangle._dragMinimumY
        drag.maximumY   : thisRectangle._dragMaximumY

        drag.threshold  : thisRectangle._dragThreshold

        // [thisRectangle._isWheelPropagated] --
        onWheel: {
            // DISABLE MOUSE WHEEL BEING PASSED TO BENEATH ITEMS
            wheel.accepted = !thisRectangle._isWheelPropagated;

            if(thisRectangle._maskEnabled) {
                return;
            }
            //print('MOUSE AREA WHEELED');
            thisRectangle.baseWheel(wheel);
        }

        onClicked: {
            if(thisRectangle._maskEnabled)
                return;
            //print('MOUSE AREA CLICKED');
            // !!! REFRESH UI
            if(!thisRectangle._stayPut) {
                ;
                //MAINGB.refreshMainWindowUI();
            }
            // -----------------------------
            if(mouse.button === Qt.LeftButton) {
                thisRectangle.baseClicked(mouse);
            }
            else if(mouse.button === Qt.RightButton) {
                thisRectangle.baseRightClicked(mouse);
            }
            else if(mouse.button === Qt.MiddleButton) {
                thisRectangle.baseMiddleClicked(mouse);
            }
        }

        cursorShape     : thisRectangle._maskEnabled ? Qt.ArrowCursor :
                          thisRectangle._cursorShape
        hoverEnabled    : thisRectangle._maskEnabled ? false :
                          thisRectangle._hoverEnabled // By default, no Enter or Exit
        onEntered       : {
            thisRectangle._containsMouse = true;
            if(thisRectangle._maskEnabled)
                return;
            //print('MOUSE AREA ENTERED');
            if(thisRectangle._isHighlightBorderOnHover) {
                thisRectangle.border.width = thisRectangle._CHIGHLIGHT_BORDER_WIDTH;
                thisRectangle.border.color = thisRectangle._CHIGHLIGHT_BORDER_COLOR;
            }
            thisRectangle.baseEntered();
        }
        onExited        : {
            thisRectangle._containsMouse = false;
            if(thisRectangle._maskEnabled)
                return;
            //print('MOUSE AREA EXITED');
            if(thisRectangle._isHighlightBorderOnHover) {
                thisRectangle.border.width = thisRectangle._CORIG_BORDER_WIDTH;
                thisRectangle.border.color = thisRectangle._CORIG_BORDER_COLOR;
            }
            thisRectangle.baseExited();
        }
        onPressed       : {
            if(thisRectangle._maskEnabled)
                return;
            print('MOUSE AREA PRESSED');
            thisRectangle.basePressed(mouse);
        }
        onReleased      : {
            if(thisRectangle._maskEnabled)
                return;
            print('MOUSE AREA RELEASED');
            thisRectangle.baseReleased(mouse);
        }

        //onPressed: {
        //    if (!Qt.styleHints.setFocusOnTouchRelease)
        //        button.forceActiveFocus()
        //}
        //onReleased: {
        //    if (Qt.styleHints.setFocusOnTouchRelease)
        //        button.forceActiveFocus()
        //}

        onDoubleClicked : {
            if(thisRectangle._maskEnabled)
                return;
            //print('MOUSE AREA DOUBLE CLICKED');
            thisRectangle.baseDoubleClicked(mouse);
        }
        onPressAndHold  : {
            if(thisRectangle._maskEnabled)
                return;
            //print('MOUSE AREA PRESSED/HOLD');
            thisRectangle.basePressAndHold(mouse);
        }
        onPositionChanged: {
            if(thisRectangle._maskEnabled)
                return;
            //print('MOUSE AREA MOVED');
            thisRectangle.basePositionChanged(mouse);
        }
    }

    // MASKED MOUSE AREA -----------------------------------------------------
    //
    function rebindMaskSize() {
        maskedMouseArea.alphaThreshold = 0.0;
        maskedMouseArea.maskSize = Qt.binding( function maskSize() {
            return Qt.size(backgroundImage.width, backgroundImage.height);
        });
    }
    //
    property bool   _maskEnabled         : false
    property string _maskSource          : _maskEnabled ? thisRectangle._backgroundSource : K3DRC.TEXT.EMPTY
    property alias  _maskRotationAngle   : maskedMouseArea.rotationAngle // [] > 0
    property alias  _baseMaskedMouseArea : maskedMouseArea
    K3DMaskedMouseArea {
        id: maskedMouseArea
        z : mouseArea.z

        // !NOTE: Sometimes we still let _maskEnabled but NO _maskSource to make use of K3DMaskedMouseArea services.
        property bool _enabled  : thisRectangle._mouseAreaEnabled &&
                                  thisRectangle._maskEnabled &&
                                  _maskSource !== K3DRC.TEXT.EMPTY
        enabled      : _enabled

        anchors.fill : backgroundImage
        maskSource   : _enabled ? thisRectangle._maskSource       : K3DRC.TEXT.EMPTY
        maskSize     : Qt.size(backgroundImage.width, backgroundImage.height)
        dragTarget  : thisRectangle._maskEnabled ? thisRectangle._dragTarget : null
        isMovingAgentRole   : true
        isResizingAgentRole : false

        dragMinimumX: thisRectangle._dragMinimumX
        dragMaximumX: thisRectangle._dragMaximumX
        dragMinimumY: thisRectangle._dragMinimumY
        dragMaximumY: thisRectangle._dragMaximumY

        offsetMinX  : thisRectangle._offsetMinX
        offsetMaxX  : thisRectangle._offsetMaxX
        offsetMinY  : thisRectangle._offsetMinY
        offsetMaxY  : thisRectangle._offsetMaxY

        dragRefRectX      : thisRectangle._CREF_RECTX
        dragRefRectY      : thisRectangle._CREF_RECTY
        dragRefRectWidth  : thisRectangle._CREF_RECT_WIDTH
        dragRefRectHeight : thisRectangle._CREF_RECT_HEIGHT

        acceptedButtons         : Qt.AllButtons
        propagateComposedEvents : thisRectangle._isPropagateComposedEvents
        alphaThreshold          : 0.1

        //property variant _previousPosition: Qt.point(0, 0)
        onPositionChanged: {
            // --------------------------------------
            thisRectangle.basePositionChanged(mouse);
        }
        onPressedMoved : {
            // --------------------------------------
            thisRectangle.basePressedMoved(mouse);
        }

        onWheel: {
            //print('MASKED MOUSE AREA WHEELED');
            // DISABLE MOUSE WHEEL BEING PASSED TO BENEATH ITEMS
            wheel.accepted = !thisRectangle._isWheelPropagated;
            thisRectangle.baseWheel(wheel);
        }

        onClicked: {
            //print('MASKED MOUSE AREA CLICKED');
            // !!! REFRESH UI
            if(!thisRectangle._stayPut) {
                ;
                //MAINGB.refreshMainWindowUI();
            }
            // -----------------------------
            if(mouse.button === Qt.LeftButton) {
                thisRectangle.baseClicked(mouse);
            }
            else if(mouse.button === Qt.RightButton) {
                thisRectangle.baseRightClicked(mouse);
            }
            else if(mouse.button === Qt.MiddleButton) {
                thisRectangle.baseMiddleClicked(mouse);
            }
        }

        cursorShape : thisRectangle._cursorShape
        // By default, no Enter or Exit
        hoverEnabled: thisRectangle._hoverEnabled && !mouseArea.hoverEnabled // Without Normal Mouse Area hoverEnabled
        onEntered       : {
            //print('MASKED MOUSE AREA ENTERED');
            thisRectangle._containsMouse = true;
            if(thisRectangle._isHighlightBorderOnHover) {
                thisRectangle.border.width = thisRectangle._CHIGHLIGHT_BORDER_WIDTH;
                thisRectangle.border.color = thisRectangle._CHIGHLIGHT_BORDER_COLOR;
            }
            thisRectangle.baseEntered();
        }
        onExited        : {
            //print('MASKED MOUSE AREA EXITED');
            thisRectangle._containsMouse = false;
            if(thisRectangle._isHighlightBorderOnHover) {
                thisRectangle.border.width = thisRectangle._CORIG_BORDER_WIDTH;
                thisRectangle.border.color = thisRectangle._CORIG_BORDER_COLOR;
            }
            thisRectangle.baseExited();
        }
        onPressed       : {
            //print('MASKED MOUSE AREA PRESSED', mouse);
            thisRectangle.basePressed(mouse);
            //_previousPosition = Qt.point(mouse.x, mouse.y);
        }
        onReleased      : {
            //print('MASKED MOUSE AREA RELEASED');
            thisRectangle.baseReleased(mouse);
        }
        onDoubleClicked : {
            //print('MASKED MOUSE AREA DOUBLE CLICKED');
            thisRectangle.baseDoubleClicked(mouse);
        }
//        onPressAndHold  : {
//            print('MASKED MOUSE AREA PRESSED/HOLD');
//            thisRectangle.basePressAndHold(mouse);
//        }
    }

//    signal mouseDelayOut
//    Timer {
//        id: delayTimer
//        interval: 500
//        repeat: false
//        triggeredOnStart: false
//        running: false

//        onTriggered: {
//            itemListBaseFrame.mouseDelayOut();
//        }
//    }
}
