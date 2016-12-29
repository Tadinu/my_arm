// GEOPAD - MAINGB
//
import QtQuick 2.0
import QtQuick.Window 2.2


import "qrc:///javascript/qmlcommonresource.js" as GEORC
//import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL

import com.k3d.qmladapter 1.0
import GEOPAD 1.0

pragma Singleton
// !NOTE:
// THE PROPERTY NAME OF THIS CLASS MUST BE STARTED WITH _ OR LOW CHARACTER.
// NAME LIKE "CABC" IS NOT ACCEPTABLE!
//

QtObject {
    id : thisMainGB
    property var  _K3D_QML_ADAPTER : null
    function setMainQMLAdapter(qmladapter) {
        _K3D_QML_ADAPTER = qmladapter;
    }

    property real _SCREEN_WIDTH  : Screen.width > 0  ? Screen.width  : K3DRC.SIZE.SCREEN_FULL_HD_WIDTH  // FULL HD // Actually, QtObject does not know about Screen -_- property real _SCREEN_HEIGHT : Screen.height > 0 ? Screen.height : K3DRC.SIZE.SCREEN_FULL_HD_HEIGHT // FULL HD
    property real _SCREEN_HEIGHT : Screen.height > 0 ? Screen.height : K3DRC.SIZE.SCREEN_FULL_HD_HEIGHT // FULL HD

    property real _QAPP_WIDTH : 0
    property real _QAPP_HEIGHT: 0

    // Global Dragging Zone Info
    property real _CGBDRAG_ZONE_MIN_X
    property real _CGBDRAG_ZONE_MAX_X
    property real _CGBDRAG_ZONE_MIN_Y
    property real _CGBDRAG_ZONE_MAX_Y

    // Support Widget Info
    property real _CSUPPORT_WIDGET_X      : 0
    property real _CSUPPORT_WIDGET_Y      : 0
    property real _CSUPPORT_WIDGET_WIDTH  : 0
    property real _CSUPPORT_WIDGET_HEIGHT : 0

    function getSupportWidgetInfo() {
        // DUMMY FUNCTION
    }

    property string _CCOLOR_MAIN_BACKGROUND         : "#333333"
    property int _BASE_FONT_SIZE: _SCREEN_HEIGHT <= GEORC.SIZE.SCREEN_MIN_HEIGHT ?
                                  GEORC.SIZE.TEXT_BASE_SIZE : (_SCREEN_HEIGHT * GEORC.SIZE.STANDARD_SCALE);
    property real _CITEM_BORDER_WIDTH               : _SCREEN_HEIGHT / 360
    property real _CDIALOG_BUTTON_HEIGHT            : _SCREEN_HEIGHT / 51.477  // [20] With FullHD Screen 1080
    property real _CDIALOG_SWITCH_HEIGHT            : _SCREEN_HEIGHT / 72.068  // [15] With FullHD Screen 1080
    property real _CDIALOG_SPINBOX_HEIGHT           : _CDIALOG_BUTTON_HEIGHT   // [20] With FullHD Screen 1080
    property real _CDIALOG_CHECKBOX_HEIGHT          : _SCREEN_HEIGHT / 66.920  // [16] With FullHD Screen 1080
    property real _CDIALOG_SLIDER_BG_HEIGHT         : _SCREEN_HEIGHT / 175.023 // [6] With FullHD Screen 1080
    property real _CDIALOG_CLOSE_BUTTON_SIZE        : _SCREEN_HEIGHT / 42.211  // [25] With FullHD Screen 1080
    property real _CDIALOG_TOUCH_VOLUME_UNIT_HEIGHT : _SCREEN_HEIGHT / 107.073 // [10] With FullHD Screen 1080
    property real _CBUTTON_EXTENDED_IMAGE_HEIGHT    : _SCREEN_HEIGHT / 236.796 // [4.5] With FullHD Screen 1080v
    property real _CQUICK_VIEW_BAR_ITEM_SIZE        : _SCREEN_HEIGHT / 33.975  // [33] With FullHD Screen 1080
    property real _CDIALOG_APPLY_BUTTON_RIGHT_MARGIN: _CDIALOG_BUTTON_HEIGHT   // [20] With FullHD Screen 1080
    // COMMON ANIMATION DURATION ++
    property real _CANIMATION_DURATION : 200

    property var _mainWindow            : null
    property var _mainModalDialogLoader : null
    property var _appItem               : null

    // [K3D: APP LICENSE] -------------------------------------
    property int _appEdition
    function setAppEdition(edition) {
        _appEdition = edition;
    }
    function getAppEdition() {
        return _appEdition;
    }
    function isAppExpressEdition() {
        return K3DQMLAdapter.UNYK_EXPRESS === _appEdition;
    }

    // Main Window

    // [K3D ACTIONS] ------------------------------------------
    // Refer to MainWindow::_k3dActionList
    property var _K3DACTIONS: [] // [actionId, actionState]
    // [K3D OPERATION RUN] ------------------------------------
    function k3dRunOp(opId, arg1, arg2, arg3) {
        if(_mainWindow !== null)
            return _mainWindow.k3dRunOp(opId, arg1, arg2, arg3);
        else
            return -1;
    }

    // [K3D: LOG] -------------------------------------
    function log(variant) {
        if(_mainWindow !== null)
            _mainWindow.log(variant);
    }

    // [ITEM QML PRESSED]
    function onItemPressed(k3dItemId) {
        //if(_mainWindow !== null)
        //    _mainWindow.onItemPressed(k3dItemId);
    }
    // [MAIN WINDOW] ------------------------------------------
    function getMainWindow() { return _mainWindow; }
    function setMainWindow(mainWindow) {
        _mainWindow = mainWindow;
    }

    function getRefSize(size) {
        return size * (_SCREEN_HEIGHT / GEORC.SIZE.SCREEN_FULL_HD_HEIGHT);
    }


    // [MAIN WINDOW: ACTIVE & REFRESH] ------------------------
    function refreshMainWindowUI() {
        //K3DAppMainWindow.refreshUI(); // => SLOW
        if(_mainWindow !== null)
            _mainWindow.refreshUI();
    }

    function focusMainWindow() {
        if(_mainWindow !== null)
            _mainWindow.forceFocus();
    }

    // [MAIN WINDOW: DISABLED] --------------------------------
    function setMainWindowDisabled(isDisabled, isGoNight) {
        if(_mainWindow !== null)
            _mainWindow.setDisabledMode(isDisabled, isGoNight);
    }

    function setDisabledModeAbsolute(isDisabled) {
        if(_mainWindow !== null)
            _mainWindow.setDisabledModeAbsolute(isDisabled);
    }

    // [MAINWINDOW: MODAL STATE] ------------------------------
    function setInModalShownState() {
        // I. [Set Main Window to Absolute-Disabled]
        setDisabledModeAbsolute(true);

        // II. [Dim all currently shown modeless]
        dimAllCurrentModelesses();
    }

    function closeModalDialog() {
        if(_mainWindow !== null) {
            _mainWindow.closeModalDialog();
        }
    }

    function isModalDialogShown() {
        if(_mainWindow !== null) {
            var dialogId = arguments[0];
            if(dialogId === undefined) {
                return _mainWindow.isModalDialogShown();
            }
            else {
                return _mainWindow.isModalDialogShown(dialogId);
            }
        }

        return false;
    }

    function setModalDialogEnabled(isEnabled) {
        if(_mainWindow !== null) {
            _mainWindow.setModalDialogEnabled(isEnabled);
        }
    }

    function resetModalDialogId() {
        if(_mainWindow !== null) {
            _mainWindow.resetModalDialogId();
        }
    }
    function getMainTabBarDropDownIcon(groupType, isHover) {
       if(_mainWindow !== null) {
           return _mainWindow.getMainTabBarDropDownIcon(groupType, isHover);
        }
    }

    // [MAINWINDOW: UI SYNC FOO INVOKE FROM C++] --------------
    function qmlUIReturned(ret) {
        if(_mainWindow !== null)
            _mainWindow.qmlUIReturned(ret);
    }

    // [MAINWINDOW: NOTIFICATION REVOKED] ---------------------
    function onNotificationRevoked(answer) {
        if(_mainWindow !== null) {
            // ---------------------------------------------------------------
            // Emit [SIGNAL] MainWindow::messageBoxRevoked()
            //
            // 1- Store anser here first for C++ to query right after receiving the signal:
            _mainWindow._notificationRet = answer;
            // 2- In MainWindow, C++ still EventLooping watiting for this signal!!!
            _mainWindow.notificationRevoked(answer);
        }
    }

    // [SPINBOX] : INPUT VALIDATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // Refer to [C++]: Convert::convertValidation(...)
    //
    function convertSpinboxValidation() {
        var argNo = arguments.length;
        //print("CONVERT SPINBOX: ", arguments);
        if (argNo < 8)
            return;

        var spinBox           = arguments[argNo - 8];
        var isSpinBox         = spinBox.objectName === K3DRC.TEXT.K3D_SPINBOX_OBJECT_NAME;
        if(!isSpinBox && spinBox.objectName !== K3DRC.TEXT.K3D_SPINBOX_ITEM_OBJECT_NAME)
            return;
        /*
        if(isSpinBox)
            print('SPINBOX 1 :', spinBox.value, spinBox.minimumValue, spinBox.maximumValue,
                  spinBox.decimals, spinBox.stepSize);
        else
            print('SPINBOX ITEM 1:', spinBox._value, spinBox._minValue, spinBox._maxValue,
                  spinBox._decimals, spinBox._stepSize);
        */

        var max               = arguments[argNo - 7];
        var min               = arguments[argNo - 6];
        var decimals          = arguments[argNo - 5];
        var stepSize          = arguments[argNo - 4];
        var isConvertValue    = arguments[argNo - 3];
        var dimension         = arguments[argNo - 2];
        var needValidateRange = arguments[argNo - 1];

        var isCurrentlyMM = false;
        if(argNo === 8) {
            isCurrentlyMM = isMM();
        }
        else {
            isCurrentlyMM = arguments[argNo - 9];
        }

        // -----------------------------------------------------------------------------------
        // Calculate
        var aux = 0.0;
        // [Value]
        if (isConvertValue) {
            aux = isSpinBox ? spinBox.value : spinBox._value;
            if (isCurrentlyMM) {
                aux = K3DUTIL.UTIL.inchToMM(aux, dimension);
            }
            else {
                aux = K3DUTIL.UTIL.mmToInch(aux, dimension);
            }
        }
        // decimal, max, min maybe make value change before set value.
        // [Min, Max] --
        if (needValidateRange) {
            min = isCurrentlyMM ? min : min * Math.pow(K3DRC.VAL.MMTOINCH, dimension);
            max = isCurrentlyMM ? max : max * Math.pow(K3DRC.VAL.MMTOINCH, dimension);
        }

        // [Decimals] --
        decimals = isCurrentlyMM ? decimals :
                   ((decimals + 3) > 4) ? 4 : (decimals + 3);

        // [Single Step] --
        stepSize = decimals === 0 ? stepSize : stepSize / Math.pow(10, decimals);

        // -----------------------------------------------------------------------------------
        // Set to SpinBox

        // [MM -> INCH]
        if(!isCurrentlyMM) {
            if(isSpinBox) {
                spinBox.decimals                   = decimals ; /* [Decimals] --*/
                spinBox.stepSize                   = stepSize ; /* [StepSize] --*/
                spinBox.minimumValue               = min      ; /* [Min]      --*/
                if (isConvertValue) spinBox.value  = aux      ; /* [Value]    --*/
                spinBox.maximumValue               = max      ; /* [Max]      --*/
            }
            else {
                spinBox._decimals                  = decimals; /* [Decimals] --*/  //print("DECIMAL: ", decimals);
                spinBox._stepSize                  = stepSize; /* [StepSize] --*/  //print("STEP SIZE: ", stepSize);
                spinBox._minValue                  = min     ; /* [Min]      --*/  //print("MIN: ", min);
                if (isConvertValue) spinBox._value = aux     ; /* [Value]    --*/  //print("VALUE: ", aux);
                spinBox._maxValue                  = max     ; /* [Max]      --*/  //print("MAX: ", max);
            }
        }

        // [INCH -> MM]
        if(isCurrentlyMM) {
            // [Max] --
            if(isSpinBox) {
                spinBox.maximumValue               = max     ; // [Max]      --
                if (isConvertValue) spinBox.value  = aux     ; // [Value]    --
                spinBox.decimals                   = decimals; // [Decimals] --
                spinBox.stepSize                   = stepSize; // [StepSize] --
                spinBox.minimumValue               = min     ; // [Min]      --
            }
            else {
                spinBox._maxValue                  = max     ; // [Max]      --
                if (isConvertValue) spinBox._value = aux     ; // [Value]    --
                spinBox._decimals                  = decimals; // [Decimals] --
                spinBox._stepSize                  = stepSize; // [StepSize] --
                spinBox._minValue                  = min     ; // [Min]      --
            }
        }
        /*
        if(isSpinBox)
            print('SPINBOX 2 :', spinBox.value, spinBox.minimumValue, spinBox.maximumValue,
                  spinBox.decimals, spinBox.stepSize);
        else
            print('SPINBOX ITEM 2:', spinBox._value, spinBox._minValue, spinBox._maxValue,
                  spinBox._decimals, spinBox._stepSize);
        */
    }

    function convertValueValidation(value) {
        return isMM() ? K3DUTIL.UTIL.inchToMM(value) : K3DUTIL.UTIL.mmToInch(value);
    }

    function setter(value, dimension) {
        if(dimension === undefined)
            dimension = 1;
        return !isMM() ? K3DUTIL.UTIL.inchToMM(value, dimension) : value;
    }

    function getter(value, dimension) {
        if(dimension === undefined)
            dimension = 1;
        return !isMM() ? K3DUTIL.UTIL.mmToInch(value, dimension) : value;
    }

    function validateSpinBoxStepSize(value, decimal) {
        var absVal = Math.abs(value);
        var stepsize;
        var minStepSize = Math.pow(10, -decimal);
        if(absVal === 0 ) stepsize = minStepSize;
        else if(absVal <= 0.001 ) stepsize = Math.max(0.0001,minStepSize) ;
        else if(absVal <= 0.01  ) stepsize = Math.max(0.001 ,minStepSize) ;
        else if(absVal <= 0.1   ) stepsize = Math.max(0.01  ,minStepSize) ;
        else if(absVal <= 1     ) stepsize = Math.max(0.1   ,minStepSize) ;
        else stepsize = 1;

        return stepsize;
    }

    function validateSpinBoxParamsSingleStep(spinbox) { // K3DSpinBox
        spinbox.stepSize = validateSpinBoxStepSize(spinbox.value, spinbox.decimals);
        //print("SPINBOX STEPSIZE : ", spinbox.toString(), spinbox.value, spinbox.stepSize, spinbox.decimals );
    }

    function validateSpinBoxItemParamsSingleStep(spinboxItem) { // K3DSpinBoxItem
        spinboxItem._stepSize = validateSpinBoxStepSize(spinboxItem._value, spinboxItem._decimals);
        //print("SPINBOX STEPSIZE : ", spinboxItem.toString(), spinboxItem._value, spinboxItem._stepSize, spinboxItem._decimals );
    }
    // [DIALOGS] ----------------------------------------------
    function dimAllShownModelessDialogsBUT(dialogId) {
        if(_mainWindow === undefined)
            return;

        for(var i = 0; i < _mainWindow.children.length; i++) {
            if ((GEOUTIL.UTIL.qmltypeof(_mainWindow.children[i], "QQuickLoader")) &&
                (_mainWindow.children[i].item !== null) &&
                (_mainWindow.children[i].item._isBeingShown === true) &&
                (_mainWindow.children[i].item._isModal === false) &&
                (_mainWindow.children[i].item._itemId !== dialogId)
                ) {
                _mainWindow.children[i].item.dim();
            }
        }
    }

    function dimAllCurrentModelesses() {
        if(_mainWindow === undefined)
            return;

        //print('DIM ALL MODELESS');

        for(var i = 0; i < _mainWindow.children.length; i++) {
            if ((GEOUTIL.UTIL.qmltypeof(_mainWindow.children[i], "QQuickLoader")) && // Dialog Loader [Modal or Modeless]
                (_mainWindow.children[i].item !== null) &&
                (_mainWindow.children[i].item._isBeingShown === true) &&
                (_mainWindow.children[i].item._isModal === false)
                ) {
                _mainWindow.children[i].item.dim();
            }
        }
    }

    // Dialog Loader --
    function getMainModalDialogLoader() { return _mainModalDialogLoader; }
    function setMainModalDialogLoader(dialogLoader) {
        _mainModalDialogLoader = dialogLoader;
    }
    // K3DDialog.close() calls this:
    function revokeMainModalDialogLoader() {
        if(_mainModalDialogLoader !== null) {
            // Reset regardless of the sourceComponent is currently set or not!
            _mainModalDialogLoader.sourceComponent = undefined; // Invoke K3DDialog.onDestruction()
        }
    }


    // [OTHER APP ITEMS] ------------------------------------------
    function setAppItem(appItem) {
        _appItem = appItem;
    }
    function getAppItem() { return _appItem; }


    /*
    property int opId: opVal;
    property var qmlComponentId;
    function runAppItemOp(qmlComponentId, opId) {
        if(_appItem === null)
            return;

        switch(opId) {

        }
    }
    */

    // =============================================================================================
    // [COMMON TEXT]
    //
    function tr(text) {
        return (_K3D_QML_ADAPTER !== null) ? text + _K3D_QML_ADAPTER.dummyTextAgent :
                                             text;
    }

    property string _K3D_DIALOG_BUTTON_TEXT_APPLY  : tr(qsTr("Apply"))
    property string _K3D_DIALOG_BUTTON_TEXT_CLOSE  : tr(qsTr("Close"))
    property string _K3D_DIALOG_BUTTON_TEXT_TOUCH  : tr(qsTr("Touch"))
    property string _K3D_DIALOG_BUTTON_TEXT_DELETE : tr(qsTr("Delete"))
    property string _K3D_DIALOG_BUTTON_TEXT_ADD    : tr(qsTr("Add"))

    property string _ON                            : tr(qsTr("On"))
    property string _OFF                           : tr(qsTr("Off"))

    property string _BTN_OK                        : tr(qsTr("Ok"))
    property string _BTN_YES                       : tr(qsTr("Yes"))
    property string _BTN_NO                        : tr(qsTr("No"))
    property string _BTN_NO_ALL                    : tr(qsTr("No All"))
    property string _BTN_CANCEL                    : tr(qsTr("Cancel"))
}
