/*
- VAN DE PRINTER LIST MODEL CLEARED FREQUENTLY BASED ON PRINTER TIMER LIST.
- VAN DE DISABLE TIMER, PRINTER LIST MODEL CLEARED, model.serial is not updated, binding does not work
=> NOT ABLE TO UPDATE TO LOADING STATE.
*/

// http://doc.qt.digia.com/qt-5.1/qtdoc/qtquick-performance.html
// http://www.slideshare.net/PassoK/qt-and-qml-performance-tips-tricks-for-qt-47

import QtQuick 2.0
import QtMultimedia 5.0
import QtQuick.Particles 2.0
import QtQuick.Window 2.1
import QtQuick.Controls 1.2        // Provide Qt control widgets
import QtQuick.Controls.Styles 1.2 // Provide Qt control widget styles
import QtQuick.Layouts  1.1        // Provide Qt layouts
import QtQuick.Dialogs  1.2        // StandardButton

import com.rb.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

//import "qrc:///javascript/qmlcommonresource.js" as RBRC
////import "qrc:///javascript/qmlcommonutil.js" as RBUTIL

//import "qrc:///qml/k3dBase"

pragma Singleton

//ApplicationWindow - Used by QQmlApplicationEngine
Window {
    id: gbMainAppWindow

    opacity: 1
    visible: true
    //visibility : QWindow.Minimized / QWindow::Maximized
    modality: Qt.WindowModal
    color: RBRC.COLOR.TRANSPARENT

    property int _CBUTTON_WIDTH: mainFrame.width/10
    property int _CBUTTON_HEIGHT: mainFrame.height/14.4

    // These would be reset in Component.onCompleted
    // width : minimumWidth
    // height: minimumHeight
    maximumWidth  : Screen.desktopAvailableWidth
    maximumHeight : Screen.desktopAvailableHeight
    flags: Qt.FramelessWindowHint

    property real _screenWidth  : Screen.width
    property real _screenHeight : Screen.height

    height : mainFrame.height
    onHeightChanged: {
        MAINGB._QAPP_HEIGHT = height;
    }

    on_ScreenWidthChanged: {
        MAINGB._SCREEN_WIDTH = _screenWidth;
    }

    on_ScreenHeightChanged: {
        MAINGB._SCREEN_HEIGHT = _screenHeight;
    }


    property alias _mainContentItem         : gbMainAppWindow.contentItem
    //
    MouseArea {
        id: mainFrameMouseArea
        anchors.fill: mainFrame
        z:5 // VERY IMPORTANT!
        propagateComposedEvents: true // VERY IMPORTANT TOO!! -- ALLOW PARENT TO BE DRAGGED & CHILDREN TO BE CLICKED!

        // CURSOR
        cursorShape      : mainFrameMouseArea.enabled ?
                           (mainFrameMouseArea.pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor):
                           Qt.ArrowCursor

        // DRAGGING ++
        property point clickPos: Qt.point(1,1)

        drag {
            target:  mainFrame

            minimumX: 0
            maximumX: Screen.desktopAvailableWidth - gbMainAppWindow.width
            minimumY: 0
            maximumY: Screen.desktopAvailableHeight - gbMainAppWindow.height
        }

        onPressed: {
            clickPos = Qt.point(mouse.x,mouse.y);
            //print('PRESSED:', mouse.x, mouse.y);
        }

        onPositionChanged: {
            var delta = Qt.point(mouse.x-clickPos.x, mouse.y-clickPos.y);
            gbMainAppWindow.x = gbMainAppWindow.x + delta.x;
            gbMainAppWindow.y = gbMainAppWindow.y + delta.y;
        }
        // DRAGGING --

        onClicked: {
            //print('CLICK;', gbMainNotificationBar._isBeingShown);
            if(gbMainNotificationBar._isBeingShown) {
                gbMainNotificationBar.show();
            }
            mouse.accepted = false;
        }
    }

    // COMPONENT ONCOMPLETED ===============================================================================
    //
    Component.onCompleted: {
        // ------------------------------------------------------------------------------------------------
        //_mainContentItem.clip = false;
        gbMainAppWindow.resize(Screen.height/1.3, Screen.height/1.3);
        MAINGB.setMainWindow(gbMainAppWindow); // Used to call func in MAINGB, so not use _mainContentItem
        MAINGB.setMainQMLAdapter(_geopadQMLAdapter);
        // ------------------------------------------------------------------------------------------------
        print("Geopad - Startup ##########################################");
        _mainContentItem.focus = true;
        gbMainAppWindow.forceFocus();
    }

    // KEYBOARD EVENTS =====================================================================================
    //
    signal keyCtrlTabPressed
    signal keyEscPressed
    signal keyAltF4Pressed
    signal keyDeletePressed

    signal keyUpPressed
    signal keyDownPressed
    signal keyLeftPressed
    signal keyRightPressed

    signal keyCtrl_ZPressed
    signal keyCtrl_YPressed
    signal keyCtrl_PPressed

    onKeyAltF4Pressed : {
        gbMainAppWindow.showMinimized();
    }

    // MAIN WINDOW SERVICES ==================================================================================
    //
    function log(variant) {
        _k3pmMainDialog.qmlLog(variant);
    }
    //
    function resetScreenSize(screenWidth, screenHeight) {
        print('SCREEN SIZE RESET:', MAINGB._SCREEN_WIDTH, MAINGB._SCREEN_HEIGHT);
        MAINGB._SCREEN_WIDTH   = screenWidth;
        MAINGB._SCREEN_HEIGHT  = screenHeight;
    }

    function resize(w,h) {
        MAINGB._SCREEN_WIDTH  = Screen.width;
        MAINGB._SCREEN_HEIGHT = Screen.height;
        // ----------------------------------------------
        MAINGB._QAPP_WIDTH  = gbMainAppWindow.width  = w;

        mainFrame.height   = h * 0.6; // gbMainAppWindow.height updated following mainFrame.height
        print('GBMAINAPPWINDOW:', w, h);
    }

    property string _title
    function setWindowTitle(title) {
        gbMainAppWindow.title = title;
    }

    function setDisabledModeAbsolute(isDisabled) {
        var i;

        if(isDisabled) {
            //upperFrameMouseArea.drag.target = null;

            // Child Quick Items --
            for (i = 0; i < _mainContentItem.children.length; ++i) {
                if(_mainContentItem.children[i] !== gbMainNotificationBar){
                    if(_mainContentItem.children[i].opacity !== 0) { // !NOTE: NO NEED TO DISABLE BACK ITEMS HAVING NO OPACITY
                        _mainContentItem.children[i].enabled = false;
                    }
                }
            }
        }
        else {
            // Child Quick Items --
            for (i = 0; i < _mainContentItem.children.length; ++i) {
                if(_mainContentItem.children[i] !== gbMainNotificationBar) {
                    if(_mainContentItem.children[i].opacity !== 0) { // !NOTE: DON'T ENABLE BACK ITEMS HAVING NO OPACITY
                        _mainContentItem.children[i].enabled = true;
                        _mainContentItem.children[i].opacity = 1;
                    }
                }
            }
            //upperFrameMouseArea.drag.target = mainFrame;
        }
    }

    function refreshUI() {
        //print('REFRESHUI --- ');
    }

    function forceFocus() {
        if(gbMainNotificationBar._isBeingShown) {
            gbMainNotificationBar.forceActiveFocus();
        }
//        else if(_mainModalDialog !== null)
//                _mainModalDialog.forceActiveFocus();
//        }
        else {
            _mainContentItem.forceActiveFocus();
        }
    }


    // CPP FUNCTION CALL --
    //
    function k3dRunOp(opId, arg1, arg2, arg3) {
        if(arg1 === undefined && arg2 === undefined && arg3 === undefined) {
            return _k3pmQMLAdapter.k3dRunOpVoid(opId);
        }
        else if(arg2 === undefined && arg3 === undefined) {
            return _k3pmQMLAdapter.k3dRunOpParam(opId, arg1);
        }
        else if(arg3 === undefined) {
            return _k3pmQMLAdapter.k3dRunOpDoubleParam(opId, arg1, arg2);
        }
        else {
            return _k3pmQMLAdapter.k3dRunOpTripleParam(opId, arg1, arg2, arg3);
        }
    }

    // Invoked from C++
    function goActivated() {
        //print('GO ACTIVATED!');
        //if(gbMainAppWindow.visibility === Window.Minimized)
        gbMainAppWindow.showNormal();
        gbMainAppWindow.raise();
        gbMainAppWindow.requestActivate();

        gbMainAppWindow.forceFocus();
    }

    // MAIN MODAL DIALOG =====================================================================================
    // [PENDING]...
    function closeModalDialog() { // [C++]
    }

    function isModalDialogShown() { // [QML]
        return false;
    }

    function setModalDialogEnabled(isEnabled) {
    }

    // GLOBAL FILE DIALOG =====================================================================================
    //
    function showFileDialog(title, nameFilters){
        gbMainFileDialog.nameFilters = nameFilters;
        gbMainFileDialog.open();
    }

    FileDialog {
        id: gbMainFileDialog
        visible: false
        property string _fileFullPath : {
            var filePath = fileUrl.toString().replace(/^(file:\/{3})/,"");
            // unescape html codes like '%23' for '#'
            return decodeURIComponent(filePath);
        }
    }

    property real _CMARGIN : _mainContentItem.width/83 // = 10 with Full HD screen Ratio
    // MINIMIZE/CLOSE BUTTONS =================================================================================
    //
    Row {
        anchors.top    : _mainContentItem.top
        anchors.right  : _mainContentItem.right
        anchors.margins: _CMARGIN
        spacing        : _CMARGIN
        z:50

        K3DButton {
            id: aboutButton
            _btnType: _CBUTTON_TYPE_ICON_ONLY
            _btnNormalSource  : RBRC.ICON.MAIN_WINDOW_ABOUT_BUTTON
            _btnHoveredSource : RBRC.ICON.MAIN_WINDOW_ABOUT_BUTTON_HOVER
            _btnPressedSource : RBRC.ICON.MAIN_WINDOW_ABOUT_BUTTON
            _btnSourceSize    : _mainContentItem.width/45

            opacity: gbMainAppWindow._currentMenuPage === 0 ? 1 : 0
            enabled: opacity === 1
            Behavior on opacity {
                PropertyAnimation { duration: RBRC.ANIMATION_DURATION.OPACITY * 2 }
            }

            onClicked: {
            }
        }

        K3DButton {
            id: minimizeWindowButton
            _btnType: _CBUTTON_TYPE_ICON_ONLY
            _btnNormalSource  : RBRC.ICON.MAIN_WINDOW_MINIMIZE_BUTTON
            _btnHoveredSource : RBRC.ICON.MAIN_WINDOW_MINIMIZE_BUTTON_HOVER
            _btnPressedSource : RBRC.ICON.MAIN_WINDOW_MINIMIZE_BUTTON
            _btnSourceSize : _mainContentItem.width/45
            onClicked: {
                gbMainAppWindow.close();
            }
        }

        K3DButton {
            id: closeWindowButton
            _btnType: _CBUTTON_TYPE_ICON_ONLY
            _btnNormalSource  : RBRC.ICON.MAIN_WINDOW_CLOSE_BUTTON
            _btnHoveredSource : RBRC.ICON.MAIN_WINDOW_CLOSE_BUTTON_HOVER
            _btnPressedSource : RBRC.ICON.MAIN_WINDOW_CLOSE_BUTTON
            _btnSourceSize : _mainContentItem.width/45

            onClicked: {
                gbMainAppWindow.close();
            }
        }
    }

    K3DRectangle {
        id: mainFrame
        z : mainFrameMouseArea.z - 1 // Must be lower than upperFrameMouseArea.z
        //anchors.fill: gbMainAppWindow

        anchors {
            top  : _mainContentItem.top
            left : _mainContentItem.left
            right: _mainContentItem.right
        }
        height: Screen.height/2
        color:  "#2B2B2B" // RBRC.COLOR.TRANSPARENT
        //_backgroundSource : RBRC.BACKGROUND.MAIN_WINDOW_BACKGROUND
        clip: true

        focus: true

        Keys.onPressed: {
            console.log('MAIN FRAME KEY PRESSED:', event.key);
            // ------------------------------------------------------------------

            switch(event.key) {
                case Qt.Key_F4:                                 // [F4] -------------
                    if (event.modifiers & Qt.AltModifier)       // [ALT_F4]
                        gbMainAppWindow.keyAltF4Pressed();
                    break;
                case Qt.Key_Escape:                             // [ESC] ------------
                    gbMainAppWindow.keyEscPressed();
                    break;
                case Qt.Key_Tab:                                // [TAB] ------------
                    if (event.modifiers & Qt.ControlModifier)
                        gbMainAppWindow.keyCtrlTabPressed();
                    break;
                case Qt.Key_Delete:                             // [DELETE] ---------
                    gbMainAppWindow.keyDeletePressed();
                    break;
                case Qt.Key_Up:                                 // [UP] -------------
                    gbMainAppWindow.keyUpPressed();
                    break;
                case Qt.Key_Down:                               // [DOWN] -----------
                    gbMainAppWindow.keyDownPressed();
                    break;
                case Qt.Key_Left:                               // [LEFT] -----------
                    gbMainAppWindow.keyLeftPressed();
                    break;
                case Qt.Key_Right:                              // [RIGHT] ----------
                    gbMainAppWindow.keyRightPressed();
                    break;
                case Qt.Key_Z:                                  // [CTRL_Z] ---------
                    if (event.modifiers & Qt.ControlModifier)
                        gbMainAppWindow.keyCtrl_ZPressed();
                    break;
                case Qt.Key_Y:                                  // [CTRL_Y] ---------
                    if (event.modifiers & Qt.ControlModifier)
                        gbMainAppWindow.keyCtrl_YPressed();
                    break;
                case Qt.Key_P:                                  // [CTRL_P] ---------
                    if (event.modifiers & Qt.ControlModifier)
                        gbMainAppWindow.keyCtrl_PPressed();
                    break;
            } // Switch(event.key)

            event.accepted = true;
        } // End Keys.onPressed

        Repeater {
            id: mainRobotList
            anchors.fill: mainFrame
            delegate: robotComp
        }
    } // END MAIN FRAME

    // AUTOBOT LIST =================================================================================
    //
    function initializeRobotList(no) {
        mainRobotList.model = null;
        mainRobotList.model = no;
        print('initialize mainRobotList',no);
        //print('Edit box List', mainRobotList.width, mainRobotList.height, mainRobotList.x, mainRobotList.y);
        for(var i = 0 ; i < no; i++) {
            mainRobotList.itemAt(i).anchors.centerIn = mainRobotList;
            mainRobotList.itemAt(i).setPos(Qt.point(10, 10*(i+1)));
        }
    }

    function getRobot(index) {
        if(index < 0 || index >= mainRobotList.count)
            return null;
        return mainRobotList.itemAt(index);
    }

    function moveRobot(index, pos) {
        if(index < 0 || index >= mainRobotList.count)
            return;
        mainRobotList.itemAt(index).x = pos.x;
        mainRobotList.itemAt(index).y = pox.y;
    }

    function setRobotAllVisible(isAllVisible) {
        mainRobotList.visible = isAllVisible;
    }

    function setRobotName(index, name) {
        if(index < 0 || index >= mainRobotList.count)
            return null;
        mainRobotList.itemAt(index)._name = name;
    }

    function getRobotName(index) {
        if(index < 0 || index >= mainRobotList.count)
            return "";
        return mainRobotList.itemAt(index)._name;
    }

    function getRobotListCount() {
        return mainRobotList.count;
    }

    Component {
        id: robotComp

        K3DButton {
            id: autobot
            color: _state === _CSTATE_IDLE      ? "green" :
                   _state === _CSTATE_OPERATING ? "blue"  :
                   _state === _CSTATE_ERROR     ? "red"   : "white"

            _btnSourceSize      : _mainContentItem.height/10
            radius: _btnSourceSize/2
            property string _name
            property int _state            : _CSTATE_IDLE
            property int _CSTATE_IDLE      : 1
            property int _CSTATE_OPERATING : 2
            property int _CSTATE_ERROR     : 3

            function getState() {
                return _state;
            }

            function setState(state) {
                _state = state;
            }

            function setPos(pos) {
                //('Pos edit box: ', pos.x, pos.y);
                anchors.horizontalCenterOffset = -mainRobotList.width/2  + pos.x;
                anchors.verticalCenterOffset   = -mainRobotList.height/2 + pos.y;
            }

            Behavior on anchors.horizontalCenterOffset {
                PropertyAnimation { duration: RBRC.ANIMATION_DURATION.OPACITY * 2 }
            }
            Behavior on anchors.verticalCenterOffset {
                PropertyAnimation { duration: RBRC.ANIMATION_DURATION.OPACITY * 2 }
            }
        }
    }

    // ===================================================================================================
    // GLOBAL COMPONENT LOADER
    //
    Loader { id: gbLoader }

    // UI Synchronous (Blocking invoked from C++) --
    property int _qmlUIRet : 0
    signal qmlUIReturned(int ret)
    function getQMLUIReturn() {
        return _qmlUIRet;
    }

    onKeyEscPressed: {
//        // Revoke [Notification Bar]
//        if(gbMainNotificationBar._isBeingShown) {
//            var answerCancel = (gbMainNotificationBar._buttonType === RBRC.NOTIFY_BUTTON_TYPE.STANDARD) ?
//                                StandardButton.Cancel : 2; // 2 is the Cancel button code of Type button 2
//            gbMainNotificationBar.hide(answerCancel);
//        }

//        // ----------------------------------------------
//        // Back to Main Menu Panel if currently in Menu Dialog
//        print('ESC:', mainMenuPanelView.depth);
//        if(mainMenuPanelView.depth > 0)
//            mainMenuPanelView.goMainMenuPanelPage();
    }

    // ===================================================================================================
    // MAIN PROGRESS BAR
    //
    function setProgressBarValue(value) { // [C++]
    }

    function getProgressBarValue() { // [C++]
        return 0;
    }

    function setProgressBarText(text) { // [C++]
    }

    function isProgressBarVisible() { // [C++]
        //console.log("gbMainProgressBar.visible", gbMainProgressBar.visible);
        return true;
    }

    function setProgressBarVisible(isVisible) { // [C++]
    }

    function showProgressBar(isShown) { // [C++]
    }

    /*
    MainProgressBar {
        id: gbMainProgressBar
        z: gbMainNotificationBar.z

        anchors.top: gbMainNotificationBar.bottom
        anchors.topMargin: gbMainNotificationBar.height/2
        anchors.horizontalCenter: gbMainAppWindow.horizontalCenter

        _backgroundSource: "qrc:///res/progressbar/mainProgressBar.svg"
        //color: "#2B2B2B"

        _originalWidth: MAINGB._SCREEN_HEIGHT / 1.6875
        width : _originalWidth
        height: _originalWidth/10
    }
    */

    // ===================================================================================================
    // MAIN NOTIFICATION BAR
    //
    property int _notificationRet : StandardButton.Cancel
    signal notificationRevoked(int ret)
    function getNotificationReturn() {
        return _notificationRet;
    }

    onNotificationRevoked : {
        // refreshUI();
    }

    function showNotification(type, iconId, text, buttons, defaultButton) {
        //console.log("Main::showNotification: ", text);
        gbMainNotificationBar.showNotification(type, iconId, text, buttons, defaultButton);
        //print('ACTIVE FOCUS', gbMainAppWindow.activeFocusItem);
    }

    function showNotificationII(type, iconId, text, buttonText0, buttonText1, buttonText2, defaultButton) {
        //console.log("Main::showNotification-II: ", text);
        gbMainNotificationBar.showNotificationII(type, iconId, text, buttonText0, buttonText1, buttonText2, defaultButton);
        //print('ACTIVE FOCUS', gbMainAppWindow.activeFocusItem);
    }

    function showWaitingInfo(text) {
        gbMainNotificationBar.showWaitingInfo(text);
    }

    function hideWaitingInfo() { // [C++]
        if(gbMainNotificationBar.getMessageType() === -1) {
            //print('HIDE: ', gbMainNotificationBar._messageText);
            gbMainNotificationBar.hide(StandardButton.Cancel);
        }
    }

    function hideNotification(answer) {
        if(answer === undefined)
            answer = StandardButton.Cancel;
        //print('HIDE: ', gbMainNotificationBar._messageText);
        gbMainNotificationBar.hide(answer);
    }

    function isNotificationShown() { // [C++]
        return gbMainNotificationBar._isBeingShown || gbMainNotificationBar.opacity === 1;
    }

    K3DNotificationBar {
        id: gbMainNotificationBar
        z: 100
        focus: true
        visible: true
        anchors.centerIn: mainFrame

        _backgroundSource: "qrc:///res/notificationbar/mainNotificationBar.svg"
        //color: "#2B2B2B"

        _originalWidth: gbMainAppWindow.width *7/10
        width : _originalWidth
        height: gbMainAppWindow.height/10

        // STILL NEED THIS SINCE gbMainAppWindow cannot handle Key Event
        Keys.onPressed: {
            switch(event.key) {
                case Qt.Key_Escape:                             // [ESC] ------------
                    // Revoke [Notification Bar]
                    if(gbMainNotificationBar._isBeingShown) {
                        var answerCancel = (gbMainNotificationBar._buttonType === RBRC.NOTIFY_BUTTON_TYPE.STANDARD) ?
                                            StandardButton.Cancel : 2; // 2 is the Cancel button code of Type button 2
                        gbMainNotificationBar.hide(answerCancel);
                    }
                    break;
            }
        }
    }

    // #####################################################################################################################
    // #####################################################################################################################

    // =====================================================================================================================
    // MAIN MENU DIALOG COMPONENTS =========================================================================================
    //
    // !NOTE: 'K3DQMLAdapter' is kept the same name for K3DBuilder or K3PM
    function openQMLItem(qmlItemId) {
        //print('OPEN ITEM', qmlItemId);
    }

    // #####################################################################################################################
    // #####################################################################################################################

    // ===================================================================================================
    // GLOBAL PARTICLE SYSTEM
    //
    //ParticleSystem { id: gbMainParticleSystem; z: 100; running: true; //anchors.fill: gbMainAppWindow;
    //    ImageParticle {
    //        id: imageParticle
    //        //anchors.fill: gbMainParticleSystem
    //
    //        system: gbMainParticleSystem
    //        source: RBRC.ICON.QUICK_PARTICLE
    //        color: "white"
    //        alpha: 1.0
    //        colorVariation: 0
    //        groups: ["A"]
    //
    //        property string _red   : "#CB5525"
    //        property string _green : "#6A9F06"
    //        property string _blue  : "#098ADB"
    //        /*
    //        SequentialAnimation on color {
    //            loops: Animation.Infinite
    //
    //            ColorAnimation {
    //                from: imageParticle._blue
    //                to: imageParticle._red
    //                duration: 2000
    //            }
    //            ColorAnimation {
    //                from: imageParticle._red
    //                to: imageParticle._green
    //                duration: 2000
    //            }
    //            ColorAnimation {
    //                from: imageParticle._green
    //                to: imageParticle._blue
    //                duration: 2000
    //            }
    //        }
    //        */
    //    }
    //
    //    Emitter {
    //        id: emitter
    //        //anchors.fill: gbMainParticleSystem
    //
    //        system: gbMainParticleSystem
    //        group: "A"
    //
    //        emitRate: 1000
    //        lifeSpan: 2000
    //
    //        property real percent: 0
    //        x: (mainFrame.width/2 - 10) * Math.sin(percent * 6.283185307179) + mainFrame.x + mainFrame._centerX
    //        y: (mainFrame.width/2 - 10)* Math.cos(percent * 6.283185307179) + mainFrame.y + mainFrame._centerY
    //
    //        //velocity: AngleDirection{magnitude: 64; angleVariation: 360 }
    //        velocity: PointDirection {xVariation: 4; yVariation: 4;}
    //        acceleration: PointDirection {xVariation: 10; yVariation: 10;}
    //        velocityFromMovement: 8
    //
    //        size: 4
    //        sizeVariation: 4
    //
    ////            anchors{
    ////                left: parent.left
    ////                right: parent.right
    ////                top: parent.top
    ////            }
    //
    //        SequentialAnimation on percent {
    //            loops: Animation.Infinite
    //            running: true
    //            NumberAnimation {
    //            duration: 1000
    //            from: 1
    //            to: 0
    //            loops: 8
    //            }
    //            NumberAnimation {
    //            duration: 1000
    //            from: 0
    //            to: 1
    //            loops: 8
    //            }
    //
    //        }
    //    }
    //}
} // End ApplicationWindow


    //Timer {
    //    id: timer
    //}
    //
    //function delay(delayTime, cb) {
    //    timer.interval = delayTime;
    //    timer.repeat = false;
    //    timer.triggered.connect(cb);
    //    timer.start();
    //}
    //
    //Rectangle {
    //    id: rectangle
    //    color: "yellow"
    //    anchors.fill: parent
    //    anchors.margins: 100
    //    opacity: 0
    //
    //    Behavior on opacity {
    //        NumberAnimation {
    //            duration: 500
    //        }
    //    }
    //}
    //
    //Component.onCompleted: {
    //    print("I'm printed right away..")
    //    delay(1000, function() {
    //        print("And I'm printed after 1 second!")
    //        rectangle.opacity = 1
    //    })
    //}
