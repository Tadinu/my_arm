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
/* Use Rectangle instead of ToolButton for freedom in scaling the button icon */

K3DRectangle {
    id: thisDialog
    objectName: K3DRC.TEXT.K3D_DIALOG_OBJECT_NAME    
    _stayPut: true // ALL DIALOGS ARE OF STAY-PUT TYPE!

    // PROPAGATE EVENTS ----------------------------------------------------------------------------
    //
    _maskEnabled : true
    _isDraggable : true
    _origDragMinimumX   : MAINGB._CGBDRAG_ZONE_MIN_X
    _origDragMaximumX   : MAINGB._CGBDRAG_ZONE_MAX_X - thisDialog.width
    _origDragMinimumY   : MAINGB._CGBDRAG_ZONE_MIN_Y
    _origDragMaximumY   : MAINGB._CGBDRAG_ZONE_MAX_Y - thisDialog.height
    _cursorShape        : thisDialog._pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor


    // DIALOG SIZE ---------------------------------------------------------------------------------
    //
    // Note: _CBASE_WIDTH, _CBASE_HEIGHT are provided being based on HD resolution!
    property real _CSTANDARD_SIZE_SCALE : K3DRC.SIZE.SCREEN_FULL_HD_HEIGHT / _CBASE_HEIGHT // Following height (Text Following height)
    property real _CBASE_WIDTH  // FULL HD WIDTH  OF DIALOG
    property real _CBASE_HEIGHT // FULL HD HEIGHT OF DIALOG

    width : height * (_CBASE_WIDTH / _CBASE_HEIGHT)
    height: MAINGB._SCREEN_HEIGHT  <= K3DRC.SIZE.SCREEN_MIN_HEIGHT ?
            K3DRC.SIZE.SCREEN_MIN_HEIGHT/_CSTANDARD_SIZE_SCALE : MAINGB._SCREEN_HEIGHT/_CSTANDARD_SIZE_SCALE

    property real _posX
    property real _posY

//    _backgroundImage._sourceWidth  : width
    // DIALOG TITLE -------------------------------------------------------------------------
    //
    property string _title

    // DIALOG MODELESS LOADER (MODELESS ONLY) ------------------------------------------------------
    //
    property Loader _modelessDialogLoader : null
    _baseMaskedMouseArea.isModal : _isModal

    // DIALOG SHORT KEY ----------------------------------------------------------------------------
    //
    property int _shortKey

    // DIALOG DISPLAY STATES -----------------------------------------------------------------------
    //
    property bool _isBeingShown: false     // Being Shown
    _isActivated: false                    // Being Focused

    color: K3DRC.COLOR.TRANSPARENT
    //property color _color // Define color for the Canvas's context fillStyle

    visible: false // By default: invisible.

    // DIALOG SIGNALS ------------------------------------------------------------------------------
    // [USED TO REMOTELY ORDER SPECIFIC DIALOG OPERATIONS]
    //
    //! Since a signal can be used to trigger a series of operation. We use them instead of invoking the functions directly!
    signal closed   // CLOSE: [ALT_F4] ,[X]
    signal rejected // CANCEL: [ESC]
    signal applied  // OK

    // DIALOG METHODS ------------------------------------------------------------------------------
    //
    // [HIDE]
    function hide() {
        if(thisDialog.state === K3DRC.STATE.HIDDEN)
            return;

        _k3dMainWindow.saveDialogPos(thisDialog._itemId, thisDialog.x, thisDialog.y);

        // --------------------------------------------------------------------
        thisDialog.state   = K3DRC.STATE.HIDDEN;
        thisDialog.visible = _isBeingShown = _isActivated = false;
        thisDialog.focus   = false;
        MAINGB.resetLastActivatedDialogId();
        print('HIDE DIALOG', thisDialog.x, thisDialog.y);
    }

    // [SHOW] - !NOTE: QML::show() does not call start(), K3DQMLItemAgent::showDialog() does that in some proper situation!
    function show() {
        var isHighlight = arguments[0];
        if(isHighlight === undefined) {
            isHighlight = true;
        }
        // --------------------------------------------------------------------
        thisDialog._isClosed = false;
        //
        //print('DIALOG SHOW:', _itemId);
        thisDialog.focus = true;
        thisDialog.opacity = 1;
        if(false === thisDialog._isModal) { // Modal one always has z as top
            thisDialog._modelessDialogLoader.z = 65533; //thisDialog.z = 65535;
        }

        thisDialog._isActivated = true; // Put here in case state is still K3DRC.STATE.SHOWN but in dim
        thisDialog.forceActiveFocus();


        //print('DIALOG SHOW:', thisDialog.state);

        // Already Shown:
        if(thisDialog.state ===  K3DRC.STATE.SHOWN) {
            if(thisDialog._isModal && isHighlight) {
                _highLightAnimation2.start();
            }
        }

        // Start being Shown:
        else {
            thisDialog.visible = thisDialog._isBeingShown
                               = true;
            //print('DIALOG SIZE', thisDialog.width, thisDialog.height);

            _posX = _k3dMainWindow.getDialogPosX(thisDialog._itemId);
            if(_posX === 0)
                _posX = (MAINGB._QAPP_WIDTH - thisDialog.width)/2;
            else if(_posX + thisDialog.width > MAINGB._QAPP_WIDTH)
                _posX = MAINGB._QAPP_WIDTH - thisDialog.width;

            _posY = _k3dMainWindow.getDialogPosY(thisDialog._itemId);
            if(_posY === 0)
                _posY = (MAINGB._QAPP_HEIGHT - thisDialog.height)/2;
            else if(_posY + thisDialog.height > MAINGB._QAPP_HEIGHT)
                _posY = MAINGB._QAPP_HEIGHT - thisDialog.height;

            thisDialog.x = _posX;
            thisDialog.y = _posY;

            thisDialog.state = K3DRC.STATE.SHOWN;

            // RESTORE DIALOG POSITION

            //print('DIALOG ID:',thisDialog._itemId, thisDialog.x, thisDialog.y, thisDialog.activeFocus);

            // SET MAIN WINDOW IN MODAL SHOWN STATE
            if(thisDialog._isModal) {
                //print('ENTER MODAL STATE');
                MAINGB.setInModalShownState();
            }
        }

        if(false === thisDialog._isModal) {
            MAINGB.dimAllShownModelessDialogsBUT(thisDialog._itemId);
        }
        // else
        // MAINGB.dimAllCurrentModelesses(); // => Already done in MAINGB.setInModalShownState();
    }

    // [CLOSE]
    // ! NOTE: For this code design, we know that close() is called, thisDialog itself is revoked from loader,
    // and so are all of its properties!
    function close() {
        // -----------------------------------------------------------------------
        if(_isClosed) {
            //print('Closed 2nd - REJECTED!');
            return;
        }

        // -----------------------------------------------------------------------
        // I - C++ Jobs -- (FIRST)
        if(thisDialog._isModal) {
            print('MODAL DIALOG CLOSED:',thisDialog._itemId, thisDialog.x, thisDialog.y);
            if(_itemAgent !== null && _itemAgent.isBusy()) { // NOT CLOSED YET!
                print("RETURN ON CLICK CLOSE");
                return;
            }
        }
        else {
            //print('MODELESS DIALOG CLOSED:',thisDialog._itemId, thisDialog.x, thisDialog.y);
        }

        // -----------------------------------------------------------------------
        // FROM NOW, THE DIALOG IS SURELY CLOSED THEN!
        print('Closed 1st - PROCEED!');
        _isClosed = true;

        // -----------------------------------------------------------------------
        //
        // I.2 - Save Dialog Position, Revoke Dialog memory:
        //print('CLOSE DIALOG', thisDialog.x, thisDialog.y);
        _k3dMainWindow.revokeDialogAgent(thisDialog._itemId, thisDialog.x, thisDialog.y);
        // => _itemAgent.end() <=== !NOTE: CALLED IN C++, WHICH MAY INVOKE ANOTHER CLOSE()!

        // -----------------------------------------------------------------------
        // II - QML Jobs -- (SECOND)
        thisDialog.hide(); // => MAINGB.resetLastActivatedDialogId();();
        print('Closed Dialog!');

        // II.1 - Reset gbMainModalDialogLoader.sourceComponent to undefined!
        if(thisDialog._isModal) {
            // 1 - [Resume Main Window]
            MAINGB.setDisabledModeAbsolute(false);

            // 2 - [Revoke Main Modal Dialog UI Loader]
            MAINGB.revokeMainModalDialogLoader();
            //
            // 2.1 - Reset K3DQMLAdapter::_modalDialogId (Id of the currently opened modal dialog)
            MAINGB.resetModalDialogId();
        }
        else {
            if(_modelessDialogLoader !== null && _isDeletedOnClosed) {
                // Reset regardless of the sourceComponent is currently set or not!
                _modelessDialogLoader.sourceComponent = undefined; // Invoke K3DDialog.onDestruction()
            }
        }
        // -----------------------------------------------------------------------
        // III - Focus Main Window
        MAINGB.focusMainWindow();
    }

    // [DIM]
    function dim() {
        thisDialog.opacity = 0.8;
        if(false === thisDialog._isModal) // Actually, this func is never called on a modal
            thisDialog._modelessDialogLoader.z = 50;
        thisDialog._isActivated = false;
        thisDialog.focus   = false;
    }

    // [ACTIVATE-FOCUS]
    on_IsActivatedChanged: {
        print('ACTIVATED CHANGED', _itemId, _isActivated);
        if(_isActivated) {
            MAINGB.setLastActivatedDialogId(_itemId);
        }
    }

    function isLastFocused() {
        return MAINGB._lastActivatedDialogId === _itemId;
    }

    function isActivated() {
        return thisDialog._isActivated;
    }

    function setActivated(isActivated) {
        thisDialog._isActivated = isActivated;
    }

    // !NOTE: The children property contains the list of visual children of this item.
    function activate(isNeedFocusToDialog) {
        if(isNeedFocusToDialog === undefined)
            isNeedFocusToDialog = true;
        if(_isActivated) { // !!! Not the same as K3DRC.STATE.SHOWN (Sometimes shown with opacity < 1)
            if(isNeedFocusToDialog)
                thisDialog.forceActiveFocus(); // forward keys event to thisDialog
            return;
        }
        print("ACTIVATE DIALOG ...");
        // [1] - DISPLAY, RASINGING THE DIALOG
        thisDialog.show(false);

        //...Maybe doing sth else!

        // [2] - DIALOG AGENT: onActivated() --> SIGNALED ONLY NECESSARY, SO WE PUT INSIDE THIS IF:
        if(thisDialog._itemAgent !== null)
            thisDialog._itemAgent.onActivated();
    }

    // [APPLY]
    function apply() {
        print('K3Dialog ON APPLY');
        if(!_isModal)
            thisDialog.activate();
        //
        //thisDialog.opacity = 0;
        // !NOTE: ONLY CALL close() on apply() returning True
        if(thisDialog._itemAgent.apply()) {
            //print('APPLY DONE TRUE!');
            thisDialog.close(); // Call K3DialogAgent::end()
        }
//        else {
//            thisDialog.opacity = 1;
//        }
    }

    function paramsChanged() {

    }

    // COMPONENT DESTRUCTION -----------------------------------------------------------------------
    //
    Component.onDestruction: {
        // !NOTE: close() must be called BEFORE! It's not onDestruction duty to invoke close()
        //print('DESTROYED');
    }

    // COMPONENT LOADING ---------------------------------------------------------------------------
    //
    Component.onCompleted: {
        thisDialog.show();
        //
        for (var i = 0; i < thisDialog.children.length; ++i) {
            if(thisDialog.children[i].objectName === K3DRC.TEXT.K3D_RECTANGLE_OBJECT_NAME) { // !NOTE: NO NEED TO DISABLE BACK ITEMS HAVING NO OPACITY
                thisDialog.children[i]._stayPut = true;
            }
        }
        delayTimer.start();
    }

    signal dialogComponentCompleted()
    Timer {
        id: delayTimer
        interval: 100
        triggeredOnStart: false
        running: false
        onTriggered: {
            thisDialog.dialogComponentCompleted();
        }
    }

    // KEYS PRESSED --------------------------------------------------------------------------------
    //
    // Internally Invoked:
    focus: true
    Keys.onPressed: {
        print('DIALOG KEY PRESSED:', _itemId, event.key );
        switch(event.key) {
        case Qt.Key_Enter:                               // [ENTER/RETURN] --- //16777221
            //print('K3DDIALOG-ENTER', event.key);
            //if(thisDialog.focus) {
            thisDialog.applied();
            //}
            break;

        case Qt.Key_Return: //16777220 - Main Enter Button on Keyboard
            //print('K3DDIALOG-RETURN', event.key);
            //if(thisDialog.focus) {
            thisDialog.applied();
            //}
            break;

        case Qt.Key_F4:                                 // [F4] --------------
            //print('K3DDIALOG-QML ALT F4', thisDialog.focus);
            if (event.modifiers & Qt.AltModifier) {     // [ALT_F4]
                thisDialog.closed(); // -> MAINGB.focusMainWindow();
            }
            break;

        case Qt.Key_Escape:                             // [ESC] -------------
            //print('K3DDIALOG-ESC:', thisDialog.focus);
            //if(thisDialog.focus) {
            thisDialog.closed();
            //}
            break;

        default:
            // ! WE DON'T NEED TO HIGHLIGHT THE DIALOG IF MODAL UPON UNSUPPORTED KEYS!
            break;
        } // Switch(event.key)

        // !!!
        // To prevent the event from propagating up the item hierarchy.
        event.accepted = true;
    }

    // Externally Invoked:
    onClosed  : thisDialog.close();
    onRejected: thisDialog.close();
    onApplied : thisDialog.apply();
    signal dialogClicked()
    onBasePressed: {
        thisDialog.dialogClicked();
        if(!thisDialog._isActivated || !thisDialog.focus) {
            thisDialog.activate();
        }
    }
    // DIALOG TITLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    property real _CBASE_TITLE_BG_WIDTH  : 0//dialogTitleText.width // With Full HD screen, calculate by cross the line on top of Title Text over BG
    property real _titleRightMarginRatio : 2
    property real _titleRightMargin      : ((thisDialog._CBASE_TITLE_BG_WIDTH / thisDialog._CBASE_WIDTH * thisDialog.width)
                                           - dialogTitleText.width -(closeDialogRect.width / 2 - _closeDialogBtHoriOffset)) / _titleRightMarginRatio
    K3DText {
        id : dialogTitleText
        anchors {
            top          : thisDialog.top
            topMargin    : MAINGB._CDIALOG_BUTTON_HEIGHT / 4
            right        : thisDialog.right
            rightMargin  : (closeDialogRect.width / 2 - _closeDialogBtHoriOffset) + (_titleRightMargin > MAINGB._CDIALOG_CLOSE_BUTTON_SIZE / 7 ?
                           _titleRightMargin : MAINGB._CDIALOG_CLOSE_BUTTON_SIZE / 7)
            //MAINGB._CDIALOG_CLOSE_BUTTON_SIZE / 7 is a space which suppose that thisDialog looks good.
        }
        text         : thisDialog._title.toUpperCase()
        font.bold: true
    }

    // DIALOG CLOSE BUTTON ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    //property alias _closeDialogBtHoriOffset : closeDialogButton.anchors.horizontalCenterOffset
    property real _closeDialogBtHoriOffset : -closeDialogRect.width / 5
    property alias _closeDialogBtn : closeDialogRect
    K3DRectangle{
        id : closeDialogRect
        z: thisDialog.z + 65535
        width : MAINGB._CDIALOG_CLOSE_BUTTON_SIZE
        height: width
        radius: width * 0.5
        visible: enabled
        anchors {
            verticalCenter : thisDialog.top
            verticalCenterOffset: closeDialogRect.width / 5
            horizontalCenter: thisDialog.right
            horizontalCenterOffset: _closeDialogBtHoriOffset
        }
        _isPropagateComposedEvents : false

        //_maskEnabled      : true
        _hoverEnabled     : true

        _backgroundSource : (_containsMouse && !_pressed) ? K3DRC.ICON.CLOSE_DIALOG_BTN_HOVER:
                                                            K3DRC.ICON.CLOSE_DIALOG_BTN

        onBaseClicked: {
            thisDialog.activate();
            thisDialog.close();
        }
    }

    //    K3DButton {
    //        id:closeDialogButton
    //        _isHoverChangeImage : true
    //        _btnNormalSource : K3DRC.ICON.CLOSE_DIALOG_BTN
    //        _btnHoveredSource: K3DRC.ICON.CLOSE_DIALOG_BTN_HOVER
    //        _btnPressedSource: K3DRC.ICON.CLOSE_DIALOG_BTN

    //        z:65535
    //        _btnType : _CBUTTON_TYPE_ICON_ONLY
    //        _btnSourceSize: 25

    //        anchors {
    //            verticalCenter : thisDialog.top
    //            verticalCenterOffset: 5
    //            horizontalCenter: thisDialog.right
    //            horizontalCenterOffset: thisDialog._closeDialogBtHoriOffset
    //        }
    ////        rotation: closeDialogButton._touchButtonMouseArea.containsMouse ? 180: 0;
    ////        Behavior on rotation { PropertyAnimation { duration :  MAINGB._CANIMATION_DURATION; } }
    ////        _btnSourceSize : closeDialogButton._touchButtonMouseArea.containsMouse ? 20 : 25;
    ////        Behavior on _btnSourceSize { PropertyAnimation { duration: MAINGB._CANIMATION_DURATION; } }
    //        onClicked: {
    //            thisDialog.close();
    //        }
    //    }

    /*
    Canvas {
        id:canvas
        width  : thisDialog.width
        height : thisDialog.height + thisDialog.radius
        property color strokeStyle : "white"
        property color fillStyle   : thisDialog._color
        property real lineWidth    : thisDialog._borderWidth
        property real alpha        : 1.0
        antialiasing: true
        smooth: true

        onLineWidthChanged: requestPaint();

        onPaint: {
            var ctx     = canvas.getContext('2d');
            var originX = thisDialog.x;
            var originY = thisDialog.y;

            var width   = canvas.width;
            var height  = canvas.height;
            ctx.save();
            //print("ORIGIN X, Y:",originX, originY);
            ctx.translate(originX, originY);
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.globalAlpha = canvas.alpha;
            ctx.strokeStyle = canvas.strokeStyle;
            ctx.fillStyle = canvas.fillStyle;
            ctx.lineWidth = canvas.lineWidth;

            //! [0]
            ctx.beginPath();

                var radius = 20;
                var titleHeight = height/5;
                ctx.moveTo(width/5, titleHeight);

                ctx.bezierCurveTo(width/2.7, 2/3*titleHeight , width/2.7, 0, width/2, 0);
                ctx.lineTo(width*3/4, 0);
                ctx.arcTo(width, 0, width, titleHeight, radius);

                ctx.lineTo(width, titleHeight + 2/3*height);                                    // Continue with vertical line
                ctx.arcTo(width, titleHeight  + height, width/2, titleHeight + height ,radius); // Create an arc
                ctx.lineTo(width/3, titleHeight + height);                                      // Continue with vertical line
                ctx.arcTo(0, titleHeight + height, 0, titleHeight + 2/3*height, radius);        // Create an arc
                ctx.lineTo(0, titleHeight  + height/3);
                ctx.arcTo(0, titleHeight , width/3, titleHeight , radius);                      // Create an arc

                ctx.closePath();
                //! [0]
                ctx.fill();
            ctx.stroke();

            // Draw some text
            ctx.beginPath();
                ctx.strokeStyle = "white";
                ctx.font = "15px sans-serif";
                ctx.text(thisDialog._title, width*3/4, titleHeight/2);
            ctx.stroke();

            ctx.restore();
        } // End Canvas: onPaint
    } // End Canvas
    */


    // ---------------------------------------------------------------------------------------------------------------------
    // -- STATES
    //http://qt-project.org/doc/qt-5/qtquick-statesanimations-animations.html
    //http://qt-project.org/doc/qt-5/qtquick-usecase-animations.html
    //http://qt-project.org/doc/qt-5/qml-qtquick-scriptaction.html
    //http://qt-project.org/doc/qt-4.8/qml-propertyanimation.html#details
    //http://qt-project.org/doc/qt-4.8/qdeclarativeanimation.html
    property real _startupXpos : _posX
    property real _startupYpos : _posY

    state : K3DRC.STATE.HIDDEN
    states: [
                 State {
                     name: K3DRC.STATE.SHOWN
//                     PropertyChanges { target: thisDialog; x : _posX; y   : _posY;
//                         //width  : _width; height : _height
//                     }
                 },
                 State {
                     name: K3DRC.STATE.HIDDEN
//                     PropertyChanges {
//                         target: thisDialog; x      : _startupXpos; y      : _startupYpos;
//                         //width  : 0;  height : 0;
//                     }
                 }
            ]
    transitions: Transition {
            //NumberAnimation { target: thisDialog; properties: "x, y"; easing.type: Easing.Linear; duration: 150}
    }
//    states: [
//                 State {
//                     name: K3DRC.STATE.SHOWN

//                     PropertyChanges {
//                         target: thisDialog
//                         //focus: true
//                         //width: _originalWidth
//                         //opacity:1
//                     }

//                     StateChangeScript {
//                         name: "ShownScript"
//                         script: {
//                         }
//                     }
//                 },
//                 State {
//                     name: K3DRC.STATE.HIDDEN
//                     PropertyChanges {
//                         target: thisDialog
//                         //focus: false
//                         //width: 20
//                         //opacity:0
//                     }
//                     StateChangeScript {
//                         name: "'HiddenScript"
//                         script: {
//                         }
//                     }
//                 }
//            ]
    // End STATES

    /*
    // -- TRANSITIONS
    transitions:[    // NumberAnimation inherits PropertyAnimation
                     Transition {
                         from : K3DRC.STATE.HIDDEN
                         to   : K3DRC.STATE.SHOWN
                         NumberAnimation   { properties: "width";   duration:700}
                         NumberAnimation   { properties: "opacity"; duration:700}
                         ScriptAction      { scriptName: "ShownScript" }
                     },
                     Transition {
                         //from: K3DRC.STATE.SHOWN
                         to   : K3DRC.STATE.HIDDEN
                         NumberAnimation   { properties: "width";   duration:700}
                         NumberAnimation   { properties: "opacity"; duration:700}
                     }
                ]
    // End TRANSITIONS
    */
}
