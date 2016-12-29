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

K3DButton {
    id: thisToolButton
    objectName: K3DRC.TEXT.K3D_STATEBUTTON_OBJECT_NAME
    // [State Button]: NO HOVER ALLOWED!

    // _agentChecked
    property bool _agentChecked
    on_AgentCheckedChanged: {
        //print('CHECKED CHANGE:', _CK3DITEMID, _agentChecked);
        _isChecked = _agentChecked;
    }

    Connections {
        target: MAINGB.getMainWindow()
        ignoreUnknownSignals : true
        onDynamicComponentsInitialized: {
            if(_CK3DITEMID > K3DQMLAdapter.K3D_ACTION_VOID) {
                _agentChecked = Qt.binding(function actionCheckd()  { return MAINGB._K3DACTIONS[_CK3DITEMID].checked; });
            }
        }
    }

    property bool  _isK3DButton        : false
    property bool  _accepted           : true

    _btnType : _isK3DButton ? _CBUTTON_TYPE_ICON_ONLY :
                             (_CBUTTON_TYPE_ICON_ONLY | _CBUTTON_TYPE_ABNORMAL)

    _iconSource: {
        if (_isK3DButton) {
            //print("_iconSource:", _btnType);
            return _iconBaseSource;
        }
        else if(K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ICON_ONLY)) {
            if(isCurrentlyLimited())
            {
                return _btnDisabledSource;
            }
            else
            //print('_ISCHECKED', _isChecked);
            return _isChecked ? _btnNormalSource : _btnHoveredSource;
        }
        else
            return "";
    }

    property string _downColor: "white"
    property string _upColor  : "#888888"
    _btnTextColor : {
         //print('COLOR', _btnType, _isChecked ? _downColor : _upColor);
        if(!_isK3DButton && K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_TEXT_ONLY)) {
            return _isChecked ? _downColor : _upColor;
        }
        else {
            return _downColor;
        }
    }

    signal toggleClicked(bool isChecked)
    //
    onClicked: {
        //print('44Clicked', _btnType);
        if (_isK3DButton)
            return;

        if(false === K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ABNORMAL)) // already run in K3DButton's onClicked()
            return;

        // ---------------------------------------------------
        // [1] - Toggle UI:
        _isChecked = !_isChecked;

        // Emit [toggleClicked()]
        // Invoke UI operation:
        _accepted = true;
        thisToolButton.toggleClicked(_isChecked); // !!! _accepted can be updated here-in!

        if(_accepted) {
            // [2] - Invoke C++ operation with new state (_isChecked updated above)!!!:
            if(_CK3DOPID > -1 && (K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ABNORMAL))) {
                _accepted = (0 === MAINGB.k3dRunOp(thisToolButton._CK3DOPID, _isChecked));
            }
        }

        // [3] - Reset UI if toggling fails:
        if(!_accepted) {
            _isChecked = !_isChecked;
        }
    }
} // End 'View Objects' button
