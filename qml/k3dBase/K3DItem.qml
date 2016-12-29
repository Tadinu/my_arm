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
//import "qrc:///qml/k3dBase"

K3DButton {
    id: k3DItem
    objectName: K3DRC.TEXT.K3D_ITEM_OBJECT_NAME
    // K3DItem peculiar properties --
    //
    property int    _listIndex              : -1 // By default, -1 means the item does not belong to list.
    property int    _CK3DSUBOPID            : -1
    property string _itemFunctionBtnSourceNormal
    property string _itemFunctionBtnSourceHover

    property var    _extListModel
    property int    _extListModelType       : K3DRC.LIST_MODEL_TYPE.QML

    // Item - Function Properties --
    property int    _itemState              : K3DRC.ITEM_STATE.NORMAL
    property string _CFUNCTIONAL_MARK_SOURCE

    property alias  _itemFunctionMark : functionMark
    K3DImage {
        id    : functionMark
        source: k3DItem._CFUNCTIONAL_MARK_SOURCE
        _sourceWidth : k3DItem.height/2.5
        _sourceHeight: k3DItem.height/2.5
        anchors.top: parent.top
        anchors.right: parent.right
        _imgOriginY         : parent.height / 2
        _imgRotationVectorX : parent._rotationVectorX
        _imgRotationAngle   : parent._rotationAngle
        visible: !isCurrentlyLimited() && k3DItem.enabled &&
                 K3DUTIL.FLAG.test(k3DItem._btnType, k3DItem._CBUTTON_TYPE_MULTISTATE) &&
                 k3DItem._itemState === K3DRC.ITEM_STATE.FUNCTIONAL
    }
    /*
    Rectangle {
       id: itemSeparator
       anchors { left: k3DItem.left; right: k3DItem.right; top: k3DItem.bottom; topMargin: 5}
       height: 2
       color: "black"
    }
    */

    // K3DItem peculiar signals --
    //
    onClicked: {
        //print('K3DITEM CLICKED', _CK3DITEMID);

        // [1] - RUN K3DOP WITH ITEM NEED INDEX AS PARAM ONLY:
        if(false === K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ABNORMAL)) {
            // Run the operation if provided:
            // [Index Arg]
            if(k3DItem._isItemOperationNeedIndex) {
                if((k3DItem._CK3DOPID > -1) && (k3DItem._listIndex > -1)) {
                    // Refresh Main Window UI:
                    if(!k3DItem._stayPut)
                        MAINGB.refreshMainWindowUI();

                    MAINGB.k3dRunOp(k3DItem._CK3DOPID, _listIndex);
                }
            }
            // else: MAINGB.k3dRunOp(k3DItem._CK3DOPID) already invoked in K3DButton!
        }
    }

} // End K3DItem
