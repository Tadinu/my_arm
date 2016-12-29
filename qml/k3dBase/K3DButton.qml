/*
 Rectangle {
     id: photo                                               // id on the first line makes it easy to find an object

     property bool thumbnail: false                          // property declarations
     property alias image: photoImage.source

     signal clicked                                          // signal declarations

     function doSomething(x)                                 // javascript functions
     {
         return x + photoImage.width
     }

     color: "gray"                                           // object properties
     x: 20; y: 20; height: 150                               // try to group related properties together
     width: {                                                // large bindings
         if(photoImage.width > 200){
             photoImage.width;
         }else{
             200;
         }
     }

     Rectangle {                                             // child objects
         id: border
         anchors.centerIn: parent; color: "white"

         K3DImage { id: photoImage; anchors.centerIn: parent }
     }

     states: State {                                         // states
         name: "selected"
         PropertyChanges { target: border; color: "red" }
     }

     transitions: Transition {                               // transitions
         from: ""; to: "selected"
         ColorAnimation { target: border; duration: 200 }
     }
 }

*/

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
import "qrc:///qml/k3dBase"

/* Use Rectangle instead of ToolButton for freedom in scaling the button icon */

Rectangle {
    id: touchButton
    objectName: K3DRC.TEXT.K3D_BUTTON_OBJECT_NAME
    //button {
    // cursor: pointer;
    //}
    // == Properties ==================
    // Button Operation Id (Id of the callback defined in K3DQMLAdapter class)
    property int  _CK3DOPID                 : -1
    property int  _CK3DITEMID               : 0 // K3DQMLAdapter.K3D_ACTION_VOID
    property bool _isItemOperationNeedIndex : false
    property bool _stayPut                  : false
    property bool _isChecked                : false
    property bool _isCheckable              : false
    property bool _agentEnabled             //: true // Refer to K3DQMLItemInfo::_enabled
    property bool _isHoverChangeImage       : false
    // To synchronize enabled vs _agentEnabled
    on_AgentEnabledChanged: {
        enabled = _agentEnabled;
    }

    /* // Not stable yet
    onEnabledChanged: {
        //!Note: We cannot do this since _agentEnabled is already bound with MAINGB._K3DACTIONS[_CK3DITEMID].enabled!
        //_agentEnabled = enabled;
        if(_CK3DITEMID > K3DQMLAdapter.K3D_ACTION_VOID && _CK3DITEMID < K3DQMLAdapter.K3D_ACTION_TOTAL &&
           MAINGB._K3DACTIONS[_CK3DITEMID] !== undefined) {
            MAINGB._K3DACTIONS[_CK3DITEMID].enabled = enabled;
        }
    }
    */


    property bool _isAvailable              : true // Used instead of visible
    property bool _agentVisible             : true
    /* NOT READY TO IMPLEMENT YET:
    on_AgentVisibleChanged: {
        visible = _agentVisible;
    }
    */

    // !NOTE: NEVER EVER EVER PUT _agentClicked directly bound to MAINGB._K3DACTIONS[] IN CHILD CLASS
    // SINCE THAT CHILD COULD RESIDE INSIDE A COMPONENT, THEREFORE CAN BE RELOADED INTO A LOADER, WHICH COULD
    // CAUSE POINTLESS _agentClickedChanged SIGNAL!!!
    //
    property bool _agentClicked //: false            // Refer to K3DQMLItemInfo::_clicked
    on_AgentClickedChanged: {
        print('K3DBUT CLICKED', _CK3DITEMID);
        if(/*touchButton.enabled && */_k3dMainWindow.isK3DActionValid(_CK3DITEMID)) { // Check again on valid action!
            touchButton.clicked(null);
        }
    }

    // [SIGNAL] : Write code to make use of C++ instance in onInitialized()
    signal initialized()
    //
    onInitialized: {
        touchButton.initializeAgentBindings();
    }

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
            _agentVisible = Qt.binding(function actionVisible() { return MAINGB._K3DACTIONS[_CK3DITEMID].visible; });
        }
    }

    function isCurrentlyLimited() {
        //print('emitiscurrenlimited');
        //MAINGB.isAppExpressEdition() &&
        //print('maingb', MAINGB._K3D_QML_ADAPTER/*,  MAINGB._K3D_QML_ADAPTER.isActionLimited(_CK3DITEMID)*/);
        return (MAINGB._K3D_QML_ADAPTER === null) ? false :
                                                    MAINGB.isAppExpressEdition() && MAINGB._K3D_QML_ADAPTER.isActionLimited(_CK3DITEMID);
    }

    function updateIconForAppEdittion() {
        if(MAINGB.isAppExpressEdition()) {
            if(touchButton.isCurrentlyLimited()) {
                _iconBaseSource = _btnDisabledSource;
            }
        }
        else {
            _iconBaseSource = _btnNormalSource;
        }
    }

    Connections {
        target: MAINGB.getMainWindow()
        ignoreUnknownSignals : true
        onDynamicComponentsInitialized: {
            touchButton.initialized();
        }
        onAppEditionChanged:{
            // ----------------------------------------------------------------------
            updateIconForAppEdittion();
        }
    }

    Component.onCompleted: {
        if(objectName !== K3DRC.TEXT.K3D_ITEM_OBJECT_NAME || _CK3DITEMID === K3DQMLAdapter.K3D_ACT_SHOW_HIDE_MARK_MENU
                                                          || _CK3DITEMID === K3DQMLAdapter.K3D_ACT_SUPPORT_ADD_AREA)
            touchButton.initializeAgentBindings();

        // -------------------------------------------
        visible = true;
        // Do these to serve K3DItemListBase.addItem()
        if(_btnNormalSource  == "")
            _btnNormalSource = _iconSource;
        if(_btnHoveredSource == "")
            _btnHoveredSource = _iconSource;
        if(_btnPressedSource == "")
            _btnPressedSource = _iconSource;
        updateIconForAppEdittion(); // Useful for button which was loader later in a component
    }

    // ###################################################################################################################
    // ===================================================================================================================
    //
    // Button Common Type --
    property int _CBUTTON_TYPE_ICON_ONLY  : 0x0001
    property int _CBUTTON_TYPE_TEXT_ONLY  : 0x0002
    property int _CBUTTON_TYPE_ICON_TEXT  : 0x0004
    property int _CBUTTON_TYPE_ABNORMAL   : 0x0008 // Until now, only K3DStateButton, Multistate item in K3DItemListBase and some else specific are of this type!

    property int _CBUTTON_TYPE_MULTISTATE : 0x0010
    property int _btnType: _CBUTTON_TYPE_ICON_ONLY

    // Button Icon source --
    property alias  _iconSource       : touchButtonImage.source // Use internally only!
    property string _iconBaseSource   : _btnNormalSource
    property string _btnNormalSource
    property string _btnHoveredSource
    property string _btnPressedSource
    property string _btnCheckedSource : _btnNormalSource
    property string _btnDisabledSource: _btnHoveredSource
    // Button size
    property alias  _btnSourceSize     : touchButton.width
    property real   _btnSizeByTextRatio : 1.5
    width                              : //K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ICON_ONLY) ? touchButtonImage.sourceSize.width : // Here we cannot use touchButtonImage.width since it is set to touchButton.width!
                                         K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_TEXT_ONLY) ? touchButtonText.width  * _btnSizeByTextRatio        : 0
    height                             : K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_TEXT_ONLY) ? touchButtonText.height * _btnSizeByTextRatio        : width // == touchButtonImage.sourceSize.width
    // Button-Extended Properties --
    property bool   _isExtendedType    : false
    property bool   _shownExtended     : false // Should be set outside, (either Hovered or Clicked) deciding when button should be shown extended!
    property bool   _isForceShownExtended: false
    property int    _extendingState    : K3DRC.EXTEND_DIRECTION.UPWARD
    property bool   _showExtendedMark  : false
    property real   _disabledOpacity   : 0.8
    visible : true
    opacity : enabled ? 1 : touchButton._disabledOpacity
    clip    : false
    color   : K3DRC.COLOR.TRANSPARENT
    antialiasing: true
    smooth: true

    property int    _CORIG_BORDER_WIDTH : 0
    property string _CORIG_BORDER_COLOR : "white" // "#4D5859" //#464342"//"#F7F7F7" - "#"+ Math.floor(Math.random()*16777215).toString(16)
    border.width: _CORIG_BORDER_WIDTH
    border.color: _CORIG_BORDER_COLOR

    property bool  _isHighlightBorderOnHover : false
    property int    _CHIGHLIGHT_BORDER_WIDTH : 3
    property string _CHIGHLIGHT_BORDER_COLOR : "#939393"//"#B5B5B5"

    // Mouse Area
    property bool  _containsMouse       : false//btnMouseArea.containsMouse
    property alias _pressed             : btnMouseArea.pressed
    property bool  _isWheelPropagated   : true
    property bool  _isDraggable         : false
    property alias _cursorShape         : btnMouseArea.cursorShape
    property bool  _startMouseDelayout  : false
    property int   _animationType       : _startMouseDelayout ? K3DRC.ANIMATION_TYPE.GEOMETRY :
                                                               K3DRC.ANIMATION_TYPE.IMAGE // false

    // == Signals =====================
    //
    signal clicked(var mouse)
    signal rightClicked(var mouse)
    signal middleClicked(var mouse)

    signal pressed
    signal released
    signal mouseIn
    signal mouseOut
    signal mouseDelayOut
    onMouseIn: {
        _isMouseOnBtnWithDelay = true;
    }
    onMouseDelayOut: {
        _isMouseOnBtnWithDelay = false;
    }

    property bool _isMouseOnBtnWithDelay : false
//    signal wheelup
//    signal wheeldown

    // == Functions/Methods ===========
    //
    onPressed: {
        MAINGB.onItemPressed(_CK3DITEMID);
    }

    onClicked: {
        //print('K3DBUTTON CLICKED');
        if(false === K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ABNORMAL)) {
            if(_CK3DOPID > -1) {
                if(false === _isItemOperationNeedIndex) {
                    MAINGB.k3dRunOp(_CK3DOPID);
                }
                // else MAINGB.k3dRunOp(k3DItem._CK3DOPID, _listIndex); invoked in K3DItem
            }
        }
    }

    // == CHILD ELEMENTS ==============
    //
    //
    // Background Theme --
    property alias  _themeRect         : touchButtonBackground
    property real   _themeBorderWidth  : 0
    property alias  _themeRadius       : touchButtonBackground.radius
    property alias  _themeOpacity      : touchButtonBackground.opacity
    property string _themeColor        : K3DRC.COLOR.TRANSPARENT
    property string _themeSource       : ""
    property bool   _themeAnimationOn  : false
    property int    _widthHeightMargin : 0

    NumberAnimation {
        id      : themeAnimation
        target  : touchButtonBackground
        property: "_rotation"
        from    : 0
        to      : 360
        loops   : Animation.Infinite
        duration: 3000
        running : touchButton._themeAnimationOn
    }

    K3DRectangle {
        id: touchButtonBackground
        z: -100
        antialiasing      : true

        _backgroundSource : touchButton._themeSource

        //border.width: control.activeFocus ? 2 : 1
        //border.color: "#888"
        _isWheelPropagated : touchButton._isWheelPropagated
        anchors.centerIn   : touchButton

        width             : (_backgroundSource !== K3DRC.TEXT.EMPTY) ? (touchButton.width  - _widthHeightMargin) : 0
        height            : (_backgroundSource !== K3DRC.TEXT.EMPTY) ? (touchButton.height - _widthHeightMargin) : 0

        color             : (_backgroundSource === K3DRC.TEXT.EMPTY) ? K3DRC.TEXT.EMPTY : touchButton._themeColor
        border.width      : touchButton._themeBorderWidth
    }

    //
    // Image source --
    property bool  _setTouchBtnImageOutSide : false
    property bool  _touchButtonImageVisible : false
    property alias _touchButtonImage : touchButtonImage
    property alias _btnRotation  : touchButtonImage.rotation
    property real  _touchButtonImageVerticalOffset : 0
    property real  _touchButtonImageLeftMargin     : 0
    property real  _rotationVectorX : 0
    property real  _rotationVectorY : 0
    property real  _rotationVectorZ : 0
    property real  _rotationAngle : 0
    //Behavior on _rotationAngle { RotationAnimation { duration: 2000 } }

    on_IsCheckedChanged: {
        if(_isCheckable) {
            _iconBaseSource = _isChecked ? _btnCheckedSource : _btnNormalSource;
        }
    }
    K3DImage {
        id: touchButtonImage
        visible: _setTouchBtnImageOutSide ? _touchButtonImageVisible : (false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY))
        anchors.left: parent.left
        anchors.leftMargin: _touchButtonImageLeftMargin
        anchors.verticalCenter: parent.verticalCenter
        anchors.verticalCenterOffset : _touchButtonImageVerticalOffset
        fillMode: Image.PreserveAspectFit
        smooth: true
        antialiasing: true
        mipmap: true
        source: touchButton._iconBaseSource // So, auto visible only if _btnNormalSource is provided!

        // We just need to set either one of these, the other would be scaled maintaining image's aspect ratio!
        sourceSize.width : touchButton.width
        //sourceSize.height: touchButton.height
        //property alias  _btnImageRotation      : btnImageRotation

        transform: Rotation {
            id : btnImageRotation
            origin.x : touchButtonImage.width / 2
            origin.y : touchButtonImage.height / 2
            axis {
                x : touchButton._rotationVectorX
                y : touchButton._rotationVectorY
                z : touchButton._rotationVectorZ
            }
            angle : touchButton._rotationAngle
        }

        // !NOTE:
        // With fillMode as Image.PreserveAspectFit, we just need to set the Image's width, and the height would
        // be scaled accordingly!
        Component.onCompleted: {
            if (K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ICON_ONLY)) {
                anchors.fill = touchButton;
                //anchors.margins  = 3;
            }
            else {
                switch(touchButton._btnTextPosition)
                {
                case K3DRC.BUTTON_TEXT_POS.TOP:
                case K3DRC.BUTTON_TEXT_POS.BOTTOM:
                    anchors.centerIn         = touchButton;
                    break;
                case K3DRC.BUTTON_TEXT_POS.LEFT:
                    anchors.right            = touchButton.right;
                    //anchors.rightMargin      = 20;
                    anchors.verticalCenter   = touchButton.verticalCenter;
                    break;
                case K3DRC.BUTTON_TEXT_POS.RIGHT:
                    anchors.left             = touchButton.left;
                    //anchors.leftMargin       = 20;
                    anchors.verticalCenter   = touchButton.verticalCenter;
                    break;
                default: // Icon Only
                    break;
                }
            }
        }
    }

    // Mouse Area --
    property alias _touchButtonMouseArea : btnMouseArea
    property bool  _extendedMarkRotate   : true
    property alias _btnHoverEnabled      : btnMouseArea.hoverEnabled
    MouseArea { // ! Don't use K3DMouseArea here-in!!! (For calling refreshUI() unexpectedly
        id:btnMouseArea
        anchors.fill: K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ICON_ONLY)   ? touchButtonImage :
                      //K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY) ? touchButtonBackground:
                      touchButton
        //acceptedButtons: Qt.LeftButton | Qt.RightButton // PLEASE SPECIFY THIS IN CHILD CLASS, DON'T UPDATE IT HERE!

        drag.target     : touchButton._isDraggable? touchButton : undefined
        drag.threshold: 1

        // Clicked --
        onClicked: {
            if(isCurrentlyLimited())
                return;

            if(!touchButton._stayPut) {
                MAINGB.refreshMainWindowUI();
            }

            // ---------------------------------------------------------------------------------------------
            if(mouse.button === Qt.LeftButton) {
                if (touchButton._extendedMarkRotate){
                    touchButton._extendingState =
                    touchButton._extendingState === K3DRC.EXTEND_DIRECTION.UPWARD ? K3DRC.EXTEND_DIRECTION.DOWNWARD :
                                                                                    K3DRC.EXTEND_DIRECTION.UPWARD;
                }
                touchButton.clicked(mouse);
            }
            else if(mouse.button === Qt.RightButton) {
                touchButton.rightClicked(mouse);
            }
            else if(mouse.button === Qt.MiddleButton) {
                touchButton.middleClicked(mouse);
            }
        }

        // Pressed --
        onPressed: {
            touchButton.pressed();
            if(false === touchButton._isCheckable &&
               false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY) &&
               false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ABNORMAL) &&
               false === touchButton.isCurrentlyLimited()) {
                    touchButton._iconBaseSource = touchButton._btnPressedSource;                     // -- Pressed background
            }
        }
        onReleased:{
            touchButton.released();
            if(false === touchButton._isCheckable &&
               false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY) &&
               false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ABNORMAL)  &&
               touchButton._animationType === K3DRC.ANIMATION_TYPE.IMAGE                              &&
               false === touchButton.isCurrentlyLimited()) {
                if(_isHoverChangeImage) {
                    touchButton._iconBaseSource = (hoverEnabled && btnMouseArea.containsMouse) ? touchButton._btnHoveredSource       // -- Released background
                                                               : touchButton._btnNormalSource;
                }
                else
                    touchButton._iconBaseSource = touchButton._btnNormalSource;
            }
        }
        //onPressed: {
        //    if (!Qt.styleHints.setFocusOnTouchRelease)
        //        button.forceActiveFocus()
        //}
        //onReleased: {
        //    if (Qt.styleHints.setFocusOnTouchRelease)
        //        button.forceActiveFocus()
        //}

        onWheel: {
            wheel.accepted = !touchButton._isWheelPropagated;
            //print('BUTTON WHEEL ACCEPTED', wheel.accepted);
//            if (wheel.modifiers & Qt.ControlModifier) {
//                if (wheel.angleDelta.y > 0)
//                    touchButton.wheelup();
//                else
//                    touchButton.wheeldown();
//            }
        }
        // Hovered --
        hoverEnabled: true // FOR USING onEntered & onExited
        //Change cursor key to pointinghand
        cursorShape: hoverEnabled ? Qt.PointingHandCursor : Qt.ArrowCursor
        onEntered: {
            touchButton._containsMouse = true;
            if(K3DUTIL.FLAG.test(touchButton._btnType,touchButton._CBUTTON_TYPE_TEXT_ONLY)) {
                //touchButtonText.font.underline = true;
                //touchButtonText.font.bold = true;
                //touchButtonText.style = Text.Raised;
                //touchButtonText.font.pixelSize = 14;

                if(touchButton._isHighlightBorderOnHover) {
                    touchButton.border.color = touchButton._CHIGHLIGHT_BORDER_COLOR;
                }
            }
            else {
                if(false === _isCheckable && // State Button
                   false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ABNORMAL)&&
                   touchButton._animationType === K3DRC.ANIMATION_TYPE.IMAGE &&
                   false === touchButton.isCurrentlyLimited() && _isHoverChangeImage) {
                    // we do not change image when hover anymore!!!!
                    touchButton._iconBaseSource = touchButton._btnHoveredSource;        // -- Hovered background
                }
                delayTimer.stop();
                //print('ENTERED');
            }
            // ------------------------------------------
            touchButton.mouseIn();
            tooltipFrame.startTime = 0;
        }
        onExited:{
            touchButton._containsMouse = false;
            if(K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY)) {
                //touchButtonText.font.underline  = false;
                //touchButtonText.font.bold = false;
                //touchButtonText.style = Text.Normal;
                //touchButtonText.font.pixelSize = 12;

                if(touchButton._isHighlightBorderOnHover) {
                    // If backgroundImage is BorderImage !!!
                    //backgroundImage.anchors.margins = thisRectangle.border.width = thisRectangle._CORIG_BORDER_WIDTH;
                    touchButton.border.color = touchButton._CORIG_BORDER_COLOR;
                }
            }
            else {
                if(false === _isCheckable &&
                   false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ABNORMAL) &&
                   touchButton._animationType === K3DRC.ANIMATION_TYPE.IMAGE &&
                   false === touchButton.isCurrentlyLimited())
                    touchButton._iconBaseSource = touchButton._btnNormalSource;                     // -- Normal background
                //if(touchButton._shownExtended) // Only start timer for button being in Shown Extended Mode only!
            }
            // ------------------------------------------
            if(touchButton._startMouseDelayout)
                delayTimer.start(); //!NOTE: SOMETIMES, SHOULD START TIMER NOT REGARDING WHETHER touchButton is being Shown Extended or NOT!

            touchButton.mouseOut();
            tooltipFrame.startTime = -100;
            tooltipFrame.secondsElapsed = 0;
        }
    }

    // Button text --
    //http://qt-project.org/doc/qt-5/qml-qtquick-text.html#style-prop
    property alias  _btnText                   : touchButtonText.text
    property alias  _btnTextBold               : touchButtonText.font.bold
    property int    _btnTextPosition           : K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY) ?
                                                 K3DRC.BUTTON_TEXT_POS.TEXTONLY_CENTER : K3DRC.BUTTON_TEXT_POS.BOTTOM
    property alias  _btnTextRotation           : touchButtonText.rotation
    property alias  _btnTextStyle              : touchButtonText.style
    property string _btnTextStyleColor         : "white"
    property alias  _btnTextFontSource         : touchButtonText._fontSource

    property string _btnTextColor              : "white" //: touchButtonText.color // Cannot use alias for color type...
    property alias  _btnTextPixelSize          : touchButtonText.font.pixelSize
    property alias  _btnTextPixelSizeScale     : touchButtonText._fontSizeScale
    property alias  _btnTextLetterSpacing      : touchButtonText.font.letterSpacing
    property alias  _btnTextFontCapitalization : touchButtonText.font.capitalization
    property alias  _btnTextVerticalAlignment  : touchButtonText.verticalAlignment
    property alias  _btnTextHorizontalAlignment: touchButtonText.horizontalAlignment

    property alias  _touchButtonText           : touchButtonText
    property int    _CTEXT_ONLY_LEFT_MARGIN    : MAINGB._SCREEN_HEIGHT / 54
    property int    _textOnlyRightMargin       : MAINGB._SCREEN_HEIGHT / 54
    property int    _CICON_TEXT_LEFT_MARGIN    : MAINGB._SCREEN_HEIGHT / 162
    property alias  _btnTextWidth              : touchButtonText.width
    property real   _CTEXT_WIDTH               : touchButton.width * 2
    property alias  _CTEXT_HEIGHT              : touchButtonText.height
    property alias  _btnTextWrap               : touchButtonText._wrapMode
    property alias  _btnMaxTextWidth           : touchButtonText._maxTextWidth

    K3DText {
        id                 : touchButtonText
        //text
        z: 65534

        visible            : K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY) ||
                             K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_ICON_TEXT)

        font.letterSpacing : 1
        color              : touchButton._btnTextColor
        styleColor         : touchButton._btnTextStyleColor

        Component.onCompleted: {
            if(false === K3DUTIL.FLAG.test(touchButton._btnType, touchButton._CBUTTON_TYPE_TEXT_ONLY)) {
                width = (touchButton._btnTextPosition === K3DRC.BUTTON_TEXT_POS.TOP ||
                         touchButton._btnTextPosition === K3DRC.BUTTON_TEXT_POS.BOTTOM)? _CTEXT_WIDTH : touchButton.width //(touchButton.width * 2) : touchButton.width
            }
            // else width kept as native

            switch(touchButton._btnTextPosition)
            {
            case K3DRC.BUTTON_TEXT_POS.TOP:
                anchors.bottom           = touchButtonImage.top;
                anchors.bottomMargin     = touchButtonText.font.pixelSize;
                anchors.horizontalCenter = touchButton.horizontalCenter;
                //width                    = touchButton.parent.width
                break;
            case K3DRC.BUTTON_TEXT_POS.LEFT:
                anchors.left             = touchButton.left;
                anchors.right            = touchButtonImage.left;
                anchors.rightMargin      = MAINGB._SCREEN_HEIGHT / 162;
                anchors.verticalCenter   = touchButton.verticalCenter;
                touchButtonText.horizontalAlignment = Text.AlignRight
                break;
            case K3DRC.BUTTON_TEXT_POS.BOTTOM:
                anchors.top              = touchButtonImage.bottom;
                anchors.topMargin        = touchButtonText.font.pixelSize;
                anchors.horizontalCenter = touchButton.horizontalCenter;
                //width                    = touchButton.parent.width
                break;
            case K3DRC.BUTTON_TEXT_POS.RIGHT:
                anchors.left                 = touchButtonImage.right;
                anchors.right                = touchButton.right;
                anchors.leftMargin           = _CICON_TEXT_LEFT_MARGIN;
                anchors.verticalCenter       = touchButton.verticalCenter;
                touchButtonText.horizontalAlignment = Text.AlignLeft
                break;
            case K3DRC.BUTTON_TEXT_POS.TEXTONLY_LEFT: // _CBUTTON_TYPE_TEXT_ONLY - Anchors Left
                anchors.left             = touchButton.left;
                if(horizontalAlignment === Text.AlignLeft)
                    anchors.leftMargin   = touchButton._CTEXT_ONLY_LEFT_MARGIN;
                anchors.verticalCenter   = touchButton.verticalCenter;
                break;
            case K3DRC.BUTTON_TEXT_POS.TEXTONLY_RIGHT: // _CBUTTON_TYPE_TEXT_ONLY - Anchors Right
                anchors.right            = touchButton.right;
                if(horizontalAlignment === Text.AlignRight)
                    anchors.rightMargin  = touchButton._textOnlyRightMargin;
                anchors.verticalCenter   = touchButton.verticalCenter;
                break;
            case K3DRC.BUTTON_TEXT_POS.TEXTONLY_CENTER: // _CBUTTON_TYPE_TEXT_ONLY - Anchors Center
                anchors.centerIn         = touchButtonBackground;
                break;
            default:
                break;
            }
        }
    }

    // Timer --
    property int _intervalTime: 200
    property alias _delayTimer: delayTimer
    Timer {
        id: delayTimer
        interval: touchButton._intervalTime
        repeat: false
        triggeredOnStart: false
        running: false
        onTriggered: {
            touchButton.mouseDelayOut();
            //tooltipFrame.timeChanged()
        }
    }

    // Tooltip --
    property alias  _btnTooltip                : tooltipFrame._tooltipText
    property alias  _btnTooltipColor           : tooltipFrame._tooltipColor
    property real   _btnTooltipDelayShownTimer : 500
    property bool   _btnTooltipShownAllowed    : true
    property bool   _btnTooltipForceShown      : false
    property int    _btnTooltipPos             : K3DRC.TOOLTIP_POS.BOTTOM
    property alias  _touchButtonTooltip        : tooltipFrame
    property real   _btnTooltipTopMargin       : 10
    property alias  _btnTooltipRotation           : tooltipFrame.rotation
    property alias  _btnTooltipLeftMargin      : tooltipFrame._tooltipLeftMargin
    property alias  _setbtnToolTipOpacity      : tooltipFrame._setToolTipOpacity
    property alias  _toolTipWidth              : tooltipFrame.width
    property alias  _toolTipHorizontalOffset   : tooltipFrame._toolTipHorizontalOffset
    property real _tooltipRotationVectorX: 0
    property real _tooltipRotationVectorY: 0
    property real _tooltipRotationVectorZ: 0
    property real _tooltipRotationAngle: 0

    //Behavior on _tooltipRotationAngle { RotationAnimation { duration : 2000 } }
    K3DTooltip {
        id: tooltipFrame
        z: 65535
        property double startTime: -10
        property double secondsElapsed: 0
        property bool   result : (touchButton._btnTooltipShownAllowed && touchButton._isDraggable) ||
                                  ((tooltipFrame._tooltipText !== K3DRC.TEXT.EMPTY) &&
                                  (touchButton._btnTooltipForceShown || (touchButton._btnTooltipShownAllowed && touchButton._containsMouse))
                                  &&(tooltipFrame.secondsElapsed >=2));//&&(tooltipFrame.secondsElapsed<=7);

        function timeChanged(){
            if (tooltipFrame.startTime ==0){
                secondsElapsed++ //+= elapsedTimer.interval/500
            }
        }
        Timer  {
            id: elapsedTimer
            interval: _btnTooltipDelayShownTimer;
            running: true;
            repeat: true;
            onTriggered: tooltipFrame.timeChanged()
        }
        _target    : touchButton
        _tooltipTopMargin : touchButton._btnTooltipTopMargin
        _tooltipPos: touchButton._btnTooltipPos
        _isShown   : result
        transform: Rotation {
            id : _btnTooltipRotation
            origin.x : tooltipFrame.width / 2
            origin.y : tooltipFrame.height / 2
            axis {
                x : touchButton._tooltipRotationVectorX
                y : touchButton._tooltipRotationVectorY
                z : touchButton._tooltipRotationVectorZ
            }
            angle : touchButton._tooltipRotationAngle
        }
        //{if(result) {tooltipFrame.secondsElapsed = 0;}

                     ///print('TOOLTIP:', tooltipFrame._tooltipText, _tooltipPos, tooltipFrame.visible, touchButton._containsMouse, touchButton._btnTooltipShownAllowed);
        //}
        //function restartCounter()  {
        //    if(result == true)
        //        tooltipFrame.startTime = -10;
        //}
    } // End Tooltip



    // Menu Extend Mark --
    property alias _extendedMark           : extendedMark
    property int   _extendMarkPos          : 1
    property int   _mainMachineBarDuration : 200
    property real  _CEXT_MARK_RIGHT_POS_LEFT_MARGIN  : -MAINGB._CDIALOG_BUTTON_HEIGHT
    K3DImage {
        id: extendedMark
        z:1000

        Component.onCompleted: {
            switch(_extendMarkPos) {
                case 1: // LEFT
                    anchors.left             = touchButton.right;
                    anchors.leftMargin       = touchButton._CEXT_MARK_RIGHT_POS_LEFT_MARGIN; //extendedMark.width*2.5;
                    anchors.verticalCenter   = touchButton.verticalCenter;
                    //height                   = touchButton.height/6;
                    break;

                case 2: // BELOW
                    anchors.top              = touchButton.bottom;
                    anchors.topMargin        = - MAINGB._CDIALOG_BUTTON_HEIGHT; //extendedMark.height;
                    anchors.horizontalCenter = touchButton.horizontalCenter;
                    //height                   = touchButton.height/8;
                    break;
                case 3: // Right of center
                    anchors.left             = touchButton.horizontalCenter;
                    anchors.leftMargin       = MAINGB._CDIALOG_BUTTON_HEIGHT; //extendedMark.width*2.5;
                    anchors.verticalCenter   = touchButton.verticalCenter;
            }
        }

        visible: touchButton._isExtendedType && touchButton._showExtendedMark
        height : MAINGB._CBUTTON_EXTENDED_IMAGE_HEIGHT//4.5//touchButton.height/10
        width  : height *3/2
        //scale : 0.7
        source : touchButton._shownExtended? K3DRC.ICON.EXTENSION_MARK_RED : K3DRC.ICON.EXTENSION_MARK_WHITE
        rotation: (touchButton._extendingState === K3DRC.EXTEND_DIRECTION.UPWARD) ? 0: 180; // Upward => It depends on the original source image's rotation
        //Behavior on rotation { PropertyAnimation { duration : _isOnMainMachineBar ? _mainMachineBarDuration: MAINGB._CANIMATION_DURATION; } }
    }

    // Here, _containMouse can also be automatically bound by btnMouseArea.containMouse in its declaration!
    /*
    states: [
                 State {
                     name: "MOUSE_IN"
                     when: btnMouseArea.containsMouse
                     PropertyChanges {
                         target :touchButton
                         _containsMouse: true
                     }
                     StateChangeScript { //script: console.log("state = SHOW")
                     }
                 },
                 State {
                     name: "MOUSE_OUT"
                     when: !btnMouseArea.containsMouse
                     PropertyChanges {
                         target :touchButton
                         _containsMouse: false
                     }
                     StateChangeScript { //script: console.log("state = HIDE")
                     }
                 }
            ]
    */
    // End STATES
} // End touchButton
