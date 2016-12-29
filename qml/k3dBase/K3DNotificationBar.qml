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
    id: thisNotificationBar
    z: 1

    _CHIGHLIGHT_BORDER_WIDTH : 3
    _CORIG_BORDER_WIDTH      : 0
    _CORIG_BORDER_COLOR      : "white"

    property real _originalWidth
    property real _hiddenWidth  : 0
    property real _CITEM_SPACING: MAINGB._SCREEN_HEIGHT / 108
    antialiasing: true

    property int    _messageType
    property string _messageText : MAINGB.tr(qsTr("Notification Message!"))
    property int    _messageTextFontSizeScale : 1.0
    property string _messageIconSource
    property string _messageIconText
    property int    _messageDefaultButton

    // == Methods
    signal hidden
    property bool _isBeingShown : false

    property var _msgButtonLoaders : [msgBtn0Loader, msgBtn1Loader, msgBtn2Loader, msgBtn3Loader]

    function getMessageType() {
        return _messageType;
    }

    function show() {
        if(thisNotificationBar.state === K3DRC.STATE.SHOWN) {
            _highLightAnimation2.start();
            //MAINGB.playSound("qrc:///sound/chime_big_ben.wav");
        }
        else {
            // DISABLE MODAL DIALOG IF SHOWN:
            if(MAINGB.isModalDialogShown())
                MAINGB.setModalDialogEnabled(false); // !!! IMPORTANT!
            // -------------------------------------------------------
            // DISABLE MODE:
            //
            MAINGB.setDisabledModeAbsolute(true);
            thisNotificationBar.state = K3DRC.STATE.SHOWN;
            thisNotificationBar._isBeingShown = true;
            //thisNotificationBar.focus = true;
            thisNotificationBar.forceActiveFocus();
            //msgButtonGroup.forceActiveFocus(Qt.MouseFocusReason);
            thisNotificationBar.enabled = true;
        }
    }

    function hide(answer) {
        if(thisNotificationBar._isBeingShown && thisNotificationBar.state !== K3DRC.STATE.HIDDEN) { // Avoid repeated call on this hide()
            // NOT RESUME TO ENABLE MODE IF MODAL DIALOG IF SHOWN:
            if(false === MAINGB.isModalDialogShown()) // Set as a Modal Agent is still being shown
                MAINGB.setDisabledModeAbsolute(false);
            // ---------------------------------------------------------------
            thisNotificationBar.state = K3DRC.STATE.HIDDEN;
            thisNotificationBar._isBeingShown = false;
            // ---------------------------------------------------------------
            MAINGB.onNotificationRevoked(answer);

            // ----------------------------------------------------------------
            // Emit [SIGNAL] hidden
            thisNotificationBar.hidden();
            // ----------------------------------------------------------------
            _messageText       = "";
            _messageIconSource = "";
            msgIconAnimation.stop();
        }
        // --------------------------------------------------------------------
        // RESET FOCUS TO MAIN WINDOW:
        MAINGB.focusMainWindow(); // RESUME FOCUS TO MODAL DIALOG IF SHOWN HERE-IN!
        if(processingTimer.running)
            processingTimer.stop();
    }

    // ------------------------------------------------------------------------
    // QMessageBox::Cancel <->  StandardButton.Cancel
    function showWaitingInfo(text) {
        _buttonType  = K3DRC.NOTIFY_BUTTON_TYPE.STANDARD;
        // -----------------
        _messageType          = -1;
        _messageText          = text;
        _messageDefaultButton = -1;
        _messageIconSource    = K3DRC.ICON.SPINNING;

        // -------------------------------------------------------------------
        if(processingTimer.running)
            processingTimer.stop();
        thisNotificationBar.show();
        msgIconAnimation.start();

        if(_messageText.slice(_messageText.indexOf('.'), _messageText.length/*lastIndexOf('.')*/).trim() === "...") {
            processingTimer.start();
        }
    }

    Timer {
        id: processingTimer
        interval: 500
        repeat  : true
        running : false
        triggeredOnStart: false

        onTriggered: {

            switch(_messageText.slice(_messageText.indexOf('.'), _messageText.length/*lastIndexOf('.')*/).trim().length) {
            case 1:
                _messageText = _messageText.slice(0, _messageText.indexOf('.')) + "..";
                break;
            case 2:
                _messageText = _messageText.slice(0, _messageText.indexOf('.')) + "...";
                break;
            case 3:
                _messageText = _messageText.slice(0, _messageText.indexOf('.')) + ".";
                break;
            default:
                break;
            }
        }
    }


    property int _buttonType
    function showNotification(type, iconId, text, buttons, defaultButton) {
        _buttonType  =  K3DRC.NOTIFY_BUTTON_TYPE.STANDARD;
        // -----------------
        _messageType = type;
        _messageText = text;
        _messageDefaultButton = defaultButton;
        /* TEST ++ */
        iconId = 1;
        /* TEST -- */
//        _messageIconSource = _k3dMainWindow.getK3DOperationIconSource(iconId);
//        _messageIconText   = _k3dMainWindow.getK3DOperationIconText(iconId);

        // --------------------------------------------------------------------
        switch(buttons) {
            // !NOTE: msgBtn Loaders are all deallcocated in Hidden Script!
            //
            case (StandardButton.Yes | StandardButton.No | StandardButton.NoToAll | StandardButton.Cancel):
                //print("Yes-No-NoToAll-Cancel");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_YES;     msgBtn0Loader.item._btnCode = StandardButton.Yes;
                msgBtn1Loader.sourceComponent = messageButtonComp; msgBtn1Loader.item._btnText = MAINGB._BTN_NO;      msgBtn1Loader.item._btnCode = StandardButton.No;
                msgBtn2Loader.sourceComponent = messageButtonComp; msgBtn2Loader.item._btnText = MAINGB._BTN_NO_ALL;  msgBtn2Loader.item._btnCode = StandardButton.NoToAll;
                msgBtn3Loader.sourceComponent = messageButtonComp; msgBtn3Loader.item._btnText = MAINGB._BTN_CANCEL;  msgBtn3Loader.item._btnCode = StandardButton.Cancel;
                break;
            case (StandardButton.Yes | StandardButton.No | StandardButton.Cancel):
                //print("Yes-No-Cancel");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_YES;     msgBtn0Loader.item._btnCode = StandardButton.Yes;
                msgBtn1Loader.sourceComponent = messageButtonComp; msgBtn1Loader.item._btnText = MAINGB._BTN_NO;      msgBtn1Loader.item._btnCode = StandardButton.No;
                msgBtn2Loader.sourceComponent = messageButtonComp; msgBtn2Loader.item._btnText = MAINGB._BTN_CANCEL;  msgBtn2Loader.item._btnCode = StandardButton.Cancel;
                break;
            case (StandardButton.Yes | StandardButton.No):
                //print("Yes-No");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_YES;     msgBtn0Loader.item._btnCode = StandardButton.Yes;
                msgBtn1Loader.sourceComponent = messageButtonComp; msgBtn1Loader.item._btnText = MAINGB._BTN_NO;      msgBtn1Loader.item._btnCode = StandardButton.No;
                break;
            case (StandardButton.Yes | StandardButton.Cancel):
                //print("Yes-Cancel");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_YES;     msgBtn0Loader.item._btnCode = StandardButton.Yes;
                msgBtn1Loader.sourceComponent = messageButtonComp; msgBtn1Loader.item._btnText = MAINGB._BTN_CANCEL;  msgBtn1Loader.item._btnCode = StandardButton.Cancel;
                break;
            case (StandardButton.Ok | StandardButton.Cancel):
                //print("OK-Cancel");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_OK;      msgBtn0Loader.item._btnCode = StandardButton.Ok;
                msgBtn1Loader.sourceComponent = messageButtonComp; msgBtn1Loader.item._btnText = MAINGB._BTN_CANCEL;  msgBtn1Loader.item._btnCode = StandardButton.Cancel;
                break;
            case (StandardButton.Ok):
                //print("OK");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_OK;      msgBtn0Loader.item._btnCode  = StandardButton.Ok;
                break;
            case (StandardButton.Cancel):
                //print("OK");
                msgBtn0Loader.sourceComponent = messageButtonComp; msgBtn0Loader.item._btnText = MAINGB._BTN_CANCEL;  msgBtn0Loader.item._btnCode  = StandardButton.Cancel;
                break;
            default:
                print("UNKNOWN MESSAGE BUTTONS");
                break;
        }

        // -------------------------------------------------------------------
        thisNotificationBar.show();
    }

    // !NOTE : Though we have 4 button loaders, but up to now we support maximum 3 self-text-defined buttons
    function showNotificationII(type, iconId, text, buttonText0, buttonText1, buttonText2, defaultButton) {
        _buttonType  =  K3DRC.NOTIFY_BUTTON_TYPE.CUSTOM;
        // -----------------
        _messageType = type;
        _messageText = text;
        _messageDefaultButton = defaultButton;
//        _messageIconSource = _k3dMainWindow.getK3DOperationIconSource(iconId);
//        _messageIconText   = _k3dMainWindow.getK3DOperationIconText(iconId);

        // -------------------------------------------------------------------
        // !NOTE: msgBtn Loaders are all deallcocated in Hidden Script!
        //
        if(buttonText0 !== "") {
            msgBtn0Loader.sourceComponent = messageButtonComp;
            msgBtn0Loader.item._btnText = buttonText0;
            msgBtn0Loader.item._btnCode = 0;
        }
        if(buttonText1 !== "") {
            msgBtn1Loader.sourceComponent = messageButtonComp;
            msgBtn1Loader.item._btnText = buttonText1;
            msgBtn1Loader.item._btnCode = 1;
        }
        if(buttonText2 !== "") { /* CANCEL meaning button expected! */
            msgBtn2Loader.sourceComponent = messageButtonComp;
            msgBtn2Loader.item._btnText = buttonText2;
            msgBtn2Loader.item._btnCode = 2;
        }

        // -------------------------------------------------------------------
        //console.log("showNotification-II: ", text);
        thisNotificationBar.show();
    }

    // [MESSAGE TYPE]: CRITICAL, WARNING, INFORMATION, QUESTION
    // StandardIcon.Critical, StandardIcon.Warning, StandardIcon.Information, StandardIcon.Question
    function messageColor() {
        return (_messageType === StandardIcon.Critical)   ? "#D41208" : // RED
               (_messageType === StandardIcon.Warning)    ? "#D6C909" : // BRASS YELLOW
               (_messageType === StandardIcon.Question)   ? "#B108D3" : // QUESTION
               (_messageType === StandardIcon.Information)? "#8CD509" : /* GREEN */ "#39C0DE"; // BLUE
    }

    function messageButtonBackground() {
        return (_messageType === StandardIcon.Critical)   ? "qrc:///res/notificationbar/notifyButtonCritical.svg" :
               (_messageType === StandardIcon.Warning)    ? "qrc:///res/notificationbar/notifyButtonWarning.svg"  :
               (_messageType === StandardIcon.Question)   ? "qrc:///res/notificationbar/notifyButtonQuestion.svg" :
               (_messageType === StandardIcon.Information)? "qrc:///res/notificationbar/notifyButtonInfo.svg"     : "";
    }
    //Hover
    function messageButtonBackgroundHover(){
        return(_messageType === StandardIcon.Critical)   ? "qrc:///res/notificationbar/notifyButtonCriticalHover.svg" :
              (_messageType === StandardIcon.Warning)    ? "qrc:///res/notificationbar/notifyButtonWarningHover.svg"  :
              (_messageType === StandardIcon.Question)   ? "qrc:///res/notificationbar/notifyButtonQuestionHover.svg" :
              (_messageType === StandardIcon.Information)? "qrc:///res/notificationbar/notifyButtonInfoHover.svg"     : "";
    }

    // [DEFAULT BUTTON]
    function getCurrentDefaultButtonCode() {
        if(_buttonType === K3DRC.NOTIFY_BUTTON_TYPE.STANDARD) {
            for(var i = 0; i < _msgButtonLoaders.length; i++) {
                if(_msgButtonLoaders[i].item !== null && _msgButtonLoaders[i].item._isDefault) {
                    return _msgButtonLoaders[i].item._btnCode;
                }
            }
            return StandardButton.Cancel;
        }
        else {
            return 2; // 2 is the Cancel button code of Type button K3DRC.NOTIFY_BUTTON_TYPE.CUSTOM
        }
    }

    function setDefaultButton(isNext) {
        var numberOfButtons = 0;
        var i;
        for(i = 0; i < _msgButtonLoaders.length; i++) {
            if(_msgButtonLoaders[i].item !== null) {
                numberOfButtons ++;
            }
        }

        // !NOTE: _msgButtonLoaders ARE ALWAYS LOADED FROM 0!!!
        for(i = 0; i < numberOfButtons; i++) {
            if(false === _msgButtonLoaders[i].item._isDefault) {
                continue;
            }

            // [Next] as Default
            if(isNext) {
                _messageDefaultButton = (i === numberOfButtons-1) ?
                                        _msgButtonLoaders[0].item._btnCode :
                                        _msgButtonLoaders[i+1].item._btnCode;


            }

            // [Prev] as Default
            else {
                _messageDefaultButton = (i === 0) ?
                                        _msgButtonLoaders[numberOfButtons-1].item._btnCode :
                                        _msgButtonLoaders[i-1].item._btnCode;
            }

            return;
        }

        // ELSE
        if(_msgButtonLoaders[0].item !== null) {
            _messageDefaultButton = _msgButtonLoaders[0].item._btnCode;
        }
    }

    // KEY HANDLING =======================================================================================================
    //
    focus: true
    Keys.onPressed: {
        //print('K3DNOTIFICATION BAR-KEY PRESSED:', event.key);

        //MAINGB.refreshMainWindowUI();
        // ----------------------------------------------------------------------

        switch(event.key) {
            case Qt.Key_Enter:                               // [ENTER/RETURN] --
            case Qt.Key_Return:
                thisNotificationBar.hide(thisNotificationBar.getCurrentDefaultButtonCode());
                break;

            case Qt.Key_F4:                                 // [F4] -------------
                if (event.modifiers & Qt.AltModifier) {     // [ALT_F4]
                    var answerCancel1 = (thisNotificationBar._buttonType === K3DRC.NOTIFY_BUTTON_TYPE.STANDARD) ?
                                        StandardButton.Cancel : 2; // 2 is the Cancel button code of Type button K3DRC.NOTIFY_BUTTON_TYPE.CUSTOM
                    thisNotificationBar.hide(answerCancel1);
                }
                break;

            case Qt.Key_Escape:                             // [ESC] ------------
                var answerCancel = (thisNotificationBar._buttonType === K3DRC.NOTIFY_BUTTON_TYPE.STANDARD) ?
                                    StandardButton.Cancel : 2; // 2 is the Cancel button code of Type button K3DRC.NOTIFY_BUTTON_TYPE.CUSTOM
                thisNotificationBar.hide(answerCancel);
                break;

            case Qt.Key_Tab:                                // [TAB] ------------
            case Qt.Key_Right:                              // [RIGHT] ----------
                setDefaultButton(true);
                break;
            case Qt.Key_Left:                               // [LEFT] -----------
                setDefaultButton(false);
                break;

            default:
                // HIGHLIGHT ITSELF
                if(thisNotificationBar.state === K3DRC.STATE.SHOWN)
                    thisNotificationBar.show();
                else
                    return; // WRITE HERE JUST IN CASE SOME HOW THIS STILL HAS FOCUS WHILE IT IS NOT SHOWN AT ALL!

                break;
        } // Switch(event.key)

        // !!!
        // To prevent the event from propagating up the item hierarchy.
        // Generally, if the item acts on the key event then it should be accepted so that
        // ancestor items do not also respond to the same event.
        event.accepted = true;
    }

    // MAIN ROW ======================================================================================================================
    //
    Row {
        id: mainRow
        //anchors.fill: thisNotificationBar
        //anchors.margins: thisNotificationBar._CHIGHLIGHT_BORDER_WIDTH

        //width:thisNotificationBar.width
        height:thisNotificationBar.height

        anchors.left: thisNotificationBar.left
        anchors.verticalCenter: thisNotificationBar.verticalCenter

        spacing: thisNotificationBar._CITEM_SPACING
        // layoutDirection: Qt.LeftToRight
        //clip: true

        property real _commonSubItemHeight: (thisNotificationBar.height - 2*thisNotificationBar._CHIGHLIGHT_BORDER_WIDTH)*9/10

        // MESSAGE TYPE ==============================================================================================================
        // [_messageType]
        //
        Rectangle {
            id:msgType
            z :5
            color : thisNotificationBar.messageColor()
            anchors.verticalCenter: parent.verticalCenter

            height: mainRow.height
            width : thisNotificationBar.width/54

            //border.width: 1
            //border.color: "white"
        }

        // MESSAGE ICON ==============================================================================================================
        // [_messageIconSource]
        //
        Rectangle {
            id: msgIcon
            color : K3DRC.COLOR.TRANSPARENT

            anchors.verticalCenter: parent.verticalCenter
            height: mainRow.height
            width : thisNotificationBar.width/18

            //border.width: 1
            //border.color: "white"

            //clip:true

            NumberAnimation { id: msgIconAnimation; target: msgIconImage; property: "rotation"; from: 360; to: 0; running: true;
                              duration: 500   ; loops: Animation.Infinite}

            K3DImage {
                id: msgIconImage
                anchors.centerIn: parent
                height: parent.height*2/5
                width : height
                source: thisNotificationBar._messageIconSource
            }
            /*
            K3DText {
                id: msgIconText
                anchors.bottom: parent.bottom
                anchors.bottomMargin:  msgIconImage.anchors.topMargin
                anchors.horizontalCenter: parent.horizontalCenter
                height: parent.height/5
                text  : thisNotificationBar._messageIconText
                color : "white"
                _fontSource     : K3DRC.FONT.MAVEN_PRO_BOLD // !!
                font.bold       : true; // !!
            }
            */
        }

        // MESSAGE TEXT ==============================================================================================================
        // [_messageText]
        //
        ScrollView {
            id: msgTextFrame
            horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff
            verticalScrollBarPolicy  : Qt.ScrollBarAlwaysOff
            highlightOnFocus         : false

            width                  : thisNotificationBar.width - 5*thisNotificationBar._CITEM_SPACING - msgType.width - msgIcon.width - msgButtonGroup.width
            height                 : mainRow._commonSubItemHeight
            anchors.verticalCenter : parent.verticalCenter
            //clip                   : true
            K3DText {
                id: msgText
                text               : thisNotificationBar._messageText
                width              : msgTextFrame.width
                height             : { if (msgText.contentHeight < mainRow._commonSubItemHeight) return mainRow._commonSubItemHeight; } // !!!!~!
                // !!! if (msgText.contentHeight >= mainRow._commonSubItemHeight) -> Let the height be self-decided to make scroll works!
                anchors.verticalCenter : parent.verticalCenter

                _fontSource        : K3DRC.FONT.MAVEN_PRO_BOLD // !!
                font.bold          : true; // !!
                font.letterSpacing : 1
                _fontSizeScale     : thisNotificationBar._messageTextFontSizeScale
                wrapMode           : thisNotificationBar.width === thisNotificationBar._originalWidth? Text.WordWrap : Text.NoWrap
            }
        }

        // MESSAGE <=> BUTTONS SEPARATOR =============================================================================================
        //
        /*K3DRectangle {
            id: notificationSeparator
            anchors.verticalCenter: parent.verticalCenter
            width  : 2
            height : 2/3*thisNotificationBar.height

            _backgroundSource  : "qrc:///res/notificationbar/notificationSeparator.svg"
        }*/

        // MESSAGE BUTTONS ===========================================================================================================
        // [_messageButtons]
        //
        Row {
            id: msgButtonGroup
            //width  : thisNotificationBar.width - 3*thisNotificationBar._CITEM_SPACING - msgType.width - msgIcon.width - msgText.width
            height : mainRow._commonSubItemHeight
            anchors.verticalCenter: parent.verticalCenter

            spacing: thisNotificationBar._CITEM_SPACING
            visible: true

            Loader { id: msgBtn0Loader; z: 10 }
            Loader { id: msgBtn1Loader; z: 10 }
            Loader { id: msgBtn2Loader; z: 10 }
            Loader { id: msgBtn3Loader; z: 10 }

            focus: true
            Component {
                id: messageButtonComp
                K3DButton {

                    //id: msgBtn
                    property int  _btnCode   : StandardButton.Ok
                    property bool _isDefault : (_btnCode === thisNotificationBar._messageDefaultButton)

                    _btnTextPixelSizeScale : thisNotificationBar._messageTextFontSizeScale
                    _btnType          : _CBUTTON_TYPE_TEXT_ONLY
                    width             : msgButtonGroup.height
                    height            : width
                    _themeAnimationOn : _isDefault
                    _themeSource      : _isDefault ?
                                        (_touchButtonMouseArea.containsMouse ? thisNotificationBar.messageButtonBackgroundHover():
                                                                               thisNotificationBar.messageButtonBackground())
                                        :
                                        (_touchButtonMouseArea.containsMouse ? "qrc:///res/notificationbar/notifyButtonVoidHover.svg" :
                                                                               "qrc:///res/notificationbar/notifyButtonVoid.svg")
                    focus             : _isDefault
//                    radius : _btnSourceSize/2
//                    border.width: 2
//                    border.color: "#9A6C5D"
                    visible: true
                    _btnTextColor     : "white"
                        //MouseArea{
                        //    anchors.fill: parent
                        //    hoverEnabled: true
                        //}
                    //_btnText is set in thisNotification.showNotification()
//                    _btnTextVerticalAlignment  : Text.AlignVCenter
//                    _btnTextHorizontalAlignment: Text.AlignHCenter

                    onClicked :{
                        //console.log("Message button clicked:", _btnCode);
                        thisNotificationBar.hide(_btnCode);
                    }
                }
            }
        }
    } // THE BIG ROW

    // ---------------------------------------------------------------------------------------------------------------------
    // -- STATES
    //http://qt-project.org/doc/qt-5/qtquick-statesanimations-animations.html
    //http://qt-project.org/doc/qt-5/qtquick-usecase-animations.html
    //http://qt-project.org/doc/qt-5/qml-qtquick-scriptaction.html
    //http://qt-project.org/doc/qt-4.8/qml-propertyanimation.html#details
    //http://qt-project.org/doc/qt-4.8/qdeclarativeanimation.html
    state : K3DRC.STATE.HIDDEN
    states: [
                 State {
                     name: K3DRC.STATE.SHOWN

                     PropertyChanges {
                         target  : thisNotificationBar
                         width   : _originalWidth
                         opacity : 1
                     }

                     StateChangeScript {
                         name: "ShownScript"
                         script: {
                             //print('NOTIFICATION SHOW DONE');
                             MAINGB.qmlUIReturned(0); // Signal to C++ if as a return to a C++ synchronous QML invoke
                         }
                     }
                 },
                 State {
                     name: K3DRC.STATE.HIDDEN
                     PropertyChanges {
                         target  : thisNotificationBar
                         width   : _hiddenWidth
                         opacity : 0
                     }
                     StateChangeScript {
                         name: "HiddenScript"
                         script: {
                             // QML local UI operations done here:
                             for(var i = 0; i < _msgButtonLoaders.length; i++) {
                                 if(_msgButtonLoaders[i] !== undefined && _msgButtonLoaders[i].item !== null)
                                     _msgButtonLoaders[i].item.width = 0; // !!! IMPORTANT: RESET WIDTH TO MAKE msgButtonRow's width RESET!
                                 //
                                 _msgButtonLoaders[i].sourceComponent = undefined;
                             }
                         }
                     }
                 }
            ]
    // End STATES

    // -- TRANSITIONS
    transitions:[    // NumberAnimation inherits PropertyAnimation
                     Transition {
                         from : K3DRC.STATE.HIDDEN
                         to   : K3DRC.STATE.SHOWN
                         NumberAnimation   { properties: "width";   duration:100}
                         NumberAnimation   { properties: "opacity"; duration:100}
                         ScriptAction      { scriptName: "ShownScript" }
                     },
                     Transition {
                         //from: K3DRC.STATE.SHOWN
                         to   : K3DRC.STATE.HIDDEN
                         NumberAnimation   { properties: "width";   duration:100}
                         NumberAnimation   { properties: "opacity"; duration:100}
                     }
                ]
    // End TRANSITIONS
}
