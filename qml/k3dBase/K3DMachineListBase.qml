import QtQuick 2.3
import QtQuick.Controls 1.2 // Provide Qt control widgets
import QtQuick.Controls.Styles 1.2
import QtQuick.Window 2.1
import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL
//import "qrc:///qml/k3dBase"
import com.k3d.qmladapter 1.0
import MainGBSingletonObject 1.0

import QtQuick.Layouts 1.1


K3DItemListBase  {
    id: machineItemListFrame
    property alias _machineItemWidth: machineItemListFrame.width
    property alias _machineItemHeight: machineItemListFrame.height
    Loader{
        id: machineItemListLoader
    }
    _itemListModel: ListModel{

    }
    delegate: machineItemListDelegate
    _cITEM_HEIGHT           : MAINGB._QAPP_HEIGHT/20 < 25 ? 25 : MAINGB._QAPP_HEIGHT/20
    property string _commonItemTextColor    : "white"
    height: _isHorizontal ? parent.height: (_itemListModel.count * (machineItemListFrame._cITEM_HEIGHT))

    // =========================================================================================
    // == Signals
    signal itemExtAdded(int parentItemIndex, string extItemText, int extItemOpId, bool isAppend)
    signal itemExtRemoved(int parentItemIndex, int extItemIndex)

    // EXT ITEM LIST ---------------------------------------------------------------------------
    property int _extListPosition : 0 // [Right]: 0, [Left]: 1
    // K3DItemList -> K3DItemListBase:
    signal extItemIndexUpdated(int extItemNewIndex)

    // K3DItemListBase -> K3DItemList:
    signal extItemSelectionChanged(int itemIndex, int extItemIndex, string extItemText)
    signal extItemClicked(int itemIndex, int extItemIndex)
    signal extItemHovered(int itemIndex, int extItemIndex)

    onItemExtAdded : {
        //print('ADD TAB: ', parentItemIndex, extItemText, extItemOpId, isAppend);
        machineItemListFrame.addExtendedItem(parentItemIndex, extItemText, extItemOpId, isAppend);
    }
    onItemExtRemoved : {
        machineItemListFrame.remExtendedItem(parentItemIndex, extItemIndex);
    }

    // List Loader
    Loader {
        id: itemListLoader
    }

    // ListView properties --
    /*
      A ListView only renders the elements that are currently visible
        plus a few that are pre rendered to be shown when scrolling.
        The default implementation does not render all of them for
        performance reasons. Any references to the size of the list
        should refer to the model object instead of the ListView itself.
    */

    // ListModel properties --

    // Delegate properties --

    // Backdrop rectangle --

    // -- MODEL (Data Source)
    //!!HERE: We can see there is minor difference between Property vs Child Element
    //_itemListModel : ListModel {
        /* The attributes of ListElement is self-defined and used with Delegate!!!
        ListElement {
            _itemTooltip:       MAINGB.tr(qsTr("Tooltip"))
            _itemDesc   :       MAINGB.tr(qsTr("Description"))
            _itemHoverChangeImage : true
            _itemNormalSource:  "qrc:///res/maincontrol/home_active.png"
            _itemHoveredSource: "qrc:///res/maincontrol/home_hover.png"
            _itemPressedSource: "qrc:///res/maincontrol/home_inactive.png"
        }
        */
    //}

    // -- DELEGATE (ITEM UI)

    Component {
        id: machineitemListDelegate

        Rectangle {
            clip  : false
            width : _machineItemWidth
            height: itemFrame.height
            color : machineItemListFrame._isHighlightCurrent? (ListView.isCurrentItem ? machineItemListFrame._highlightSelectedColor :
                                                       machineItemListFrame._themeColor): machineItemListFrame._themeColor

            K3DItem {
                id: itemFrame
                width : machineItemListFrame._isHorizontal? machineItemListFrame._cITEM_WIDTH : machineItemListFrame.width
                height: machineItemListFrame._isHorizontal? machineItemListFrame.height       : machineItemListFrame._cITEM_HEIGHT
                clip  : false

                color : K3DRC.COLOR.TRANSPARENT

                border.width: machineItemListFrame._itemBorderWidth
                border.color: "white"
                radius      : machineItemListFrame._itemBorderRadius
                antialiasing: true

//                anchors.left: parent.left
//                anchors.leftMargin: 20

                _touchButtonImage.width            : itemFrame.height/2
                _touchButtonMouseArea.anchors.fill: (_btnType & _CBUTTON_TYPE_ICON_ONLY) === _CBUTTON_TYPE_ICON_ONLY ? _themeRect: itemFrame

                visible                : model._itemVisible // !!!
                enabled                : model._itemEnabled
                opacity                : enabled ? 1: 0.5
                _themeColor            : (_btnType & _CBUTTON_TYPE_ICON_ONLY) === _CBUTTON_TYPE_ICON_ONLY ? machineItemListFrame._themeColor : K3DRC.COLOR.TRANSPARENT
                _themeOpacity          : machineItemListFrame._themeOpacity

                _listIndex             : model._itemListIndex // !NOTE: Here, we can use var 'index' <-> _listIndex
                _isItemOperationNeedIndex: model._itemOperationNeedIndex
                _CK3DOPID              : model._itemK3DOpId
                _CK3DSUBOPID           : model._itemK3DSubOpId
                _btnType               : model._itemBtnType
                _btnTooltip            : model._itemTooltip
                _btnTooltipPos         : model._itemTooltipPos
                _btnText               : model._itemText
                _btnTextColor          : machineItemListFrame._isHighlightCurrent? (ListView.isCurrentItem ?
                                                                            machineItemListFrame._highlightSelectedTextColor : model._itemTextColor):
                                         (itemFrame._containsMouse? model._itemTextColor : machineItemListFrame._commonItemTextColor)
                _btnTextPixelSize      : model._itemTextPixelSize
                _btnTextPosition       : model._itemTextPosition
                _btnTextRotation       : model._itemTextRotation
                _btnTextVerticalAlignment   : model._itemTextVerticalAlignment
                _btnTextHorizontalAlignment : model._itemTextHorizontalAlignment
                _isHoverChangeImage    : model._itemHoverChangeImage
                _btnNormalSource       : model._itemNormalSource
                _btnHoveredSource      : model._itemHoveredSource
                _btnPressedSource      : model._itemPressedSource
                _itemFunctionBtnSourceNormal : model._itemFunctionBtnSourceNormal
                _itemFunctionBtnSourceHover  : model._itemFunctionBtnSourceHover
                // ----------------------------------------------------------------------------
                // Extended
                _isExtendedType        : model._itemIsExtended
                _extendingState        : model._itemExtendedDirection
                _showExtendedMark      : model._itemIsExtended && (model._itemExtendedList.count > 0)
                _extListModel          : model._itemExtListModel
                _extListModelType      : model._itemExtListModelType

                onClicked: {
                    // !NOTE: DON'T USE model._item... here, use the property directly itself!
                    if(itemFrame.enabled) {
                        // Update currentIndex
                        //console.log('MAIN LIST-CLICKED:', machineItemListFrame.currentIndex, itemFrame._listIndex);
                        if(machineItemListFrame.currentIndex !== itemFrame._listIndex) {
                            machineItemListFrame.currentIndex = itemFrame._listIndex;
                            //console.log('NEW INDEX:', itemFrame._listIndex, index);

                            machineItemListFrame.itemSelectionChanged(-1, machineItemListFrame.currentIndex);
                        }

                        if(!itemFrame._isExtendedType) {
                            // Reset machineItemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                            machineItemListFrame._oldMouseInItemIndex = machineItemListFrame._newMouseInItemIndex
                                                               = -1;
                        }

                        // ------------------------------------------------------------------------
                        // EMIT SIGNAL itemClicked(itemIndex)
                        machineItemListFrame.itemClicked(itemFrame._listIndex);
                    }
                }

                onMouseIn: {
                    if((itemFrame._btnType & itemFrame._CBUTTON_TYPE_ABNORMAL) === itemFrame._CBUTTON_TYPE_ABNORMAL)
                        return;

                    itemFrame.border.width = 2;
                    itemFrame.border.color = machineItemListFrame._hoverBorderColor;
                    if(!machineItemListFrame._isHighlightCurrent) {
                        itemFrame.color              = machineItemListFrame._hoverColor;
                        itemFrame._btnTextColor      = machineItemListFrame._hoverTextColor;
                        //itemFrame._btnTextPixelSize  = 15;
                    }

                    // Show Function Button
                    //itemFunctionBtn.opacity = 1;

                    // Flag Show Extended List
                    //print('Main List Item IN:', itemFrame._listIndex, itemFrame._isExtendedType);
                    itemFrame._shownExtended = itemFrame._isExtendedType;

                    // ------------------------------------------------------------------------
                    // Entering each row item => Whole frame: _containsMouse true
                    //console.log('Menu List Frame Mouse-Enter Row Item:', menuListMouseArea.containsMouse);
                    machineItemListFrame._containsMouse = true;
                    machineItemListFrame._newMouseInItemIndex = itemFrame._listIndex;
                    if(machineItemListFrame._oldMouseInItemIndex === -1)
                        machineItemListFrame._oldMouseInItemIndex = machineItemListFrame._newMouseInItemIndex;

                    // ------------------------------------------------------------------------
                    // EMIT SIGNAL itemClicked(itemIndex)
                    machineItemListFrame.itemHovered(itemFrame._listIndex);
                }

                onMouseOut: {
                    itemFrame.border.width = 0;
                    if(!machineItemListFrame._isHighlightCurrent) {
                        itemFrame.color              = machineItemListFrame._themeColor;
                        itemFrame._btnTextColor      = machineItemListFrame._commonItemTextColor;
                        //itemFrame._btnTextPixelSize  = 12;
                    }
                    machineItemListFrame._containsMouse = false;
                    machineItemListFrame._oldMouseInItemIndex = machineItemListFrame._newMouseInItemIndex;
                }

                onMouseDelayOut: {
                    if(itemFrame._isExtendedType) {
                        //print('Main List Item OUT:', extmachineItemListFrame._containsMouse);
                        itemFrame._shownExtended = extmachineItemListFrame._containsMouse;

                        // Reset extmachineItemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                        if(itemFrame._shownExtended === false) {
                            extmachineItemListFrame._oldMouseInItemIndex = extmachineItemListFrame._newMouseInItemIndex
                                                                  = -1;
                        }
                    }
                    else {
                        itemFrame._shownExtended = false;
                    }
                }


                // Item Sub Function Button ---------------------------------
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                K3DButton { // control.pressed
                    id: itemFunctionBtn
                    //z:65535
                    // _CK3DOPID: -1
                    anchors.verticalCenter: itemFrame.verticalCenter
                    anchors.right       : itemFrame.right
                    anchors.rightMargin : 8

                    _btnSourceSize      : itemFrame.height/3
                    _isHoverChangeImage : true
                    _btnNormalSource  : model._itemFunctionBtnSourceNormal
                    _btnHoveredSource : model._itemFunctionBtnSourceHover
                    _btnPressedSource : model._itemFunctionBtnSourceNormal

                    visible: itemFrame._containsMouse

    //                    style: ButtonStyle {
    //                        background: Rectangle {
    //                            color: itemFunctionBtn._isDown ? "#ccc" : "transparent"
    //                            border.width: itemFunctionBtn._isDown ? 1 : 0
    //                            border.color: "#888"
    //                            radius: 4
    //                        }
    //                    }

                    onClicked: {
                        //console.log('SUB FUNCTION INVOKED:', model._itemListIndex);

                        if(model._itemK3DSubOpId > -1) {
                            // Static Invocation
                            _k3dQMLAdapter.k3dRunOpParam(model._itemK3DSubOpId, model._itemListIndex);
                            // Dynamic Signal Emission
                            machineItemListFrame.itemFunctionButtonClicked(model._itemListIndex);

                            //  Here we can also use:
                            //  - Using K3D_OP_ID: static connection, best used for statically connecting to C++ gbK3DQMLAdapter's methods.
                            //  - Using signal: dynamic connection, used both for connecting to C++ gbK3DQMLAdapter's methods and QML methods.
                        }
                    }
                } // End Item Sub Function Button

                // Extended Sub Item List -----------------------------------------------------------------
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // ########################################################################################

                K3DItemListBase {
                    id: extmachineItemListFrame
                    visible             : itemFrame._shownExtended && (extmachineItemListFrame._count > 0)

                    // Anchors depending on machineItemListFrame._isHorizontal...: - TBD
                    anchors.top         : machineItemListFrame._isHorizontal? itemFrame.bottom   : itemFrame.top
                    anchors.left        : machineItemListFrame._isHorizontal? itemFrame.left     : (machineItemListFrame._extListPosition === 0) ? itemFrame.right : undefined
                    anchors.right       : machineItemListFrame._isHorizontal? itemFrame.right    : (machineItemListFrame._extListPosition === 0) ? undefined       : itemFrame.left

                    //width               : itemFrame.width
                    height              : extmachineItemListFrame._count * (itemFrame.height)

                    _isHorizontal       : false
                    _isHighlightCurrent : false
                    _itemBorderRadius   : machineItemListFrame._itemBorderRadius
                    _itemBorderWidth    : machineItemListFrame._itemBorderWidth

                    _cITEM_WIDTH        : machineItemListFrame._cITEM_WIDTH
                    _cITEM_HEIGHT       : machineItemListFrame._cITEM_HEIGHT
                    // -----------------------------------------------------------------------------------

                    _isSubList          : true
                    _parentItemIndex    : itemFrame._listIndex

                    // ListModel (DATA SOURCE)
                    _itemListModel      : machineItemListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList

                    Component.onCompleted: {
    //                    if(machineItemListFrame._isHorizontal) {
    //                        anchors.top         = itemFrame.top;
    //                        anchors.left        = itemFrame.right;
    //                    }
    //                    else {
    //                        anchors.top         = itemFrame.bottom;
    //                        anchors.right       = itemFrame.right;
    //                    }

                        machineItemListFrame.extItemIndexUpdated.connect(updateCurrentIndex);
                    }

                    function updateCurrentIndex(newItemIndex) {
                        //print('EXT LIST ITEM INDEX UPDATED:', newItemIndex);
                        extmachineItemListFrame.currentIndex = newItemIndex;
                    }

                    onItemSelectionChanged: {
                        machineItemListFrame.extItemSelectionChanged(parentItemIndex, itemIndex, _itemListModel.get(itemIndex)._itemText);
                    }

                    onItemClicked: {
                        machineItemListFrame.extItemClicked(_parentItemIndex, itemIndex);
                    }

                    onItemHovered: {
                        machineItemListFrame.extItemHovered(_parentItemIndex, itemIndex);
                    }

                    onMouseDelayOut: {
                        if(itemFrame._isExtendedType) {
                            //console.log('Extended List Mouse Delay Out:', itemFrame._containsMouse);
                            itemFrame._shownExtended = itemFrame._containsMouse;

                            // Reset extmachineItemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                            if(itemFrame._shownExtended == false) {
                                itemFrame.border.width = 0;
                                extmachineItemListFrame._oldMouseInItemIndex = extmachineItemListFrame._newMouseInItemIndex
                                                                      = -1;
                            }
                        }
                    }
                } // End K3DRectangle - Extended List Frame
            } // End itemFrame
        } // End K3DRectangle - Main Delegate Item
    } // End List Delegate

    // == Functions/Methods
    //

    Component {
        id: extItemComponent
        K3DItem {
            id: extItemFrame

            _btnType : _CBUTTON_TYPE_TEXT_ONLY
            _btnTextColor               : machineItemListFrame._commonItemTextColor
            _btnTextPixelSize           : 12
            _btnTextVerticalAlignment   : Text.AlignVCenter
            _btnTextHorizontalAlignment : Text.AlignLeft

            _btnTooltip                 : ""
            _btnTooltipPos              : 4
        }
    }

    function addExtendedItem(parentItemIndex, extItemText, extItemOpId, isAppend) {
        // LOAD extendedItemComponent ----------------------------------------
        //
        itemListLoader.sourceComponent = extItemComponent;

        // PREPARE extendedItemComponent -------------------------------------
        //
        itemListLoader.item._CK3DOPID        = extItemOpId;
        if (extItemText.length > gbMainAppWindow._CITEM_TEXT_MAX_LEN) {
            itemListLoader.item._btnTooltip  = extItemText; // Only show Tooltip for over-length text

            extItemText   = extItemText.substr(0, gbMainAppWindow._CITEM_TEXT_MAX_LEN) + "..." +
                            extItemText.slice(extItemText.lastIndexOf('.'), extItemText.length); // ".STL"
        }
        itemListLoader.item._btnText         = extItemText;

        // Other properties kept unchanged!

        // ADD INTO _itemListModel -------------------------------------------
        machineItemListFrame.addItem(parentItemIndex, itemListLoader.item, isAppend);

        // Reset itemListLoader's sourceComponent to NULL:
        itemListLoader.sourceComponent = undefined;
    } // End addExtendedItem

    function remAllExtendedItems(parentItemIndex) {
        machineItemListFrame.remAllItems(parentItemIndex);
    }

    // Remove List Item : remExtendedItem(parentItemIndex, extItemIndex);
    function remExtendedItem(parentItemIndex, extItemIndex) {
        machineItemListFrame.remItem(parentItemIndex, extItemIndex);
    } // End remExtendedItem

    function getExtendedItemListCount(parentItemIndex) {
        return machineItemListFrame._itemListModel.get(parentItemIndex)._itemExtendedList.count;
    }

    Component.onCompleted: {
    }
} // End Menu Extended List


