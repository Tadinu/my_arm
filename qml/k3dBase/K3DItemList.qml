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
// Item List
// -- VIEW (LIST)

K3DItemListBase  {
    id: itemListFrame

    _cITEM_HEIGHT           : MAINGB._SCREEN_HEIGHT /20.591 < 25 ? 25 : MAINGB._SCREEN_HEIGHT /20.591
    property string _commonItemTextColor    : "#7F7F7F"
    property string _extListThemeColor      : K3DRC.COLOR.TRANSPARENT
    height: _isHorizontal ? parent.height: (_itemListModel.count * (itemListFrame._cITEM_HEIGHT))

    // =========================================================================================
    // == Signals
    signal itemExtAdded(int parentItemIndex, string extItemText, int extItemOpId, bool isAppend, string tooltip)
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
        itemListFrame.addExtendedItem(parentItemIndex, extItemText, extItemOpId, isAppend, tooltip);
    }
    onItemExtRemoved : {
        itemListFrame.remExtendedItem(parentItemIndex, extItemIndex);
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

    // -- DELEGATE (ITEM UI)
    delegate: itemListDelegate
    Component {
        id: itemListDelegate

        Rectangle {
            clip  : false
            width : itemListFrame._cITEM_WIDTH
            height: itemFrame.height
            color : itemListFrame._isHighlightCurrent? (ListView.isCurrentItem ? itemListFrame._highlightSelectedColor :
                                                       itemListFrame._themeColor): itemListFrame._themeColor

            K3DItem {
                id: itemFrame
                width : itemListFrame._isHorizontal? itemListFrame._cITEM_WIDTH : itemListFrame.width
                height: itemListFrame._isHorizontal? itemListFrame.height       : itemListFrame._cITEM_HEIGHT
                clip  : false

                color : K3DRC.COLOR.TRANSPARENT

                border.width: itemListFrame._itemBorderWidth
                border.color: "white"
                radius      : itemListFrame._itemBorderRadius
                antialiasing: true

//                anchors.left: parent.left
//                anchors.leftMargin: 20

                _touchButtonImage.sourceSize.width            : itemFrame.height/2
                _touchButtonMouseArea.anchors.fill: (_btnType & _CBUTTON_TYPE_ICON_ONLY) === _CBUTTON_TYPE_ICON_ONLY ? _themeRect: itemFrame

                visible                : model._itemVisible // !!!
                enabled                : model._itemEnabled
                _isChecked             : model._itemChecked
                opacity                : enabled ? 1: 0.5
                _themeColor            : (_btnType & _CBUTTON_TYPE_ICON_ONLY) === _CBUTTON_TYPE_ICON_ONLY ? itemListFrame._themeColor : K3DRC.COLOR.TRANSPARENT
                _themeOpacity          : itemListFrame._themeOpacity

                _listIndex             : model._itemListIndex // !NOTE: Here, we can use var 'index' <-> _listIndex
                _isItemOperationNeedIndex: model._itemOperationNeedIndex
                _CK3DOPID              : model._itemK3DOpId
                _CK3DSUBOPID           : model._itemK3DSubOpId
                _CK3DITEMID            : model._itemK3DItemId
                _btnType               : model._itemBtnType
                _btnTooltip            : model._itemTooltip
                _btnTooltipPos         : model._itemTooltipPos
                _btnText               : model._itemText
                _btnTextBold           : model._itemTextBold
                _btnTextColor          : itemListFrame._isHighlightCurrent? (ListView.isCurrentItem ?
                                                                            itemListFrame._highlightSelectedTextColor : model._itemTextColor):
                                         (itemFrame._containsMouse? model._itemTextColor : itemListFrame._commonItemTextColor)
                _btnTextPixelSizeScale : model._itemTextPixelSizeScale
                _btnTextPosition       : model._itemTextPosition
                _btnTextRotation       : model._itemTextRotation
                _btnTextVerticalAlignment   : model._itemTextVerticalAlignment
                _btnTextHorizontalAlignment : model._itemTextHorizontalAlignment
                _isHoverChangeImage    : _isChecked ? false             : model._itemHoverChangeImage
                _btnNormalSource       : _isChecked ? _btnCheckedSource : model._itemNormalSource
                _btnHoveredSource      : _isChecked ? _btnCheckedSource : model._itemHoveredSource
                _btnPressedSource      : _isChecked ? _btnCheckedSource : model._itemPressedSource
                _btnCheckedSource      : model._itemCheckedSource

                on_IsCheckedChanged: {
                    _iconBaseSource = _isChecked ? model._itemCheckedSource:
                                                   model._itemNormalSource;
                }

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
                        //console.log('MAIN LIST-CLICKED:', itemListFrame.currentIndex, itemFrame._listIndex);
                        if(itemListFrame.currentIndex !== itemFrame._listIndex) {
                            itemListFrame.currentIndex = itemFrame._listIndex;
                            //console.log('NEW INDEX:', itemFrame._listIndex, index);

                            itemListFrame.itemSelectionChanged(-1, itemListFrame.currentIndex);
                        }

                        if(!itemFrame._isExtendedType) {
                            // Reset itemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                            itemListFrame._oldMouseInItemIndex = itemListFrame._newMouseInItemIndex
                                                               = -1;
                        }

                        // ------------------------------------------------------------------------
                        // EMIT SIGNAL itemClicked(itemIndex)
                        itemListFrame.itemClicked(itemFrame._listIndex);
                    }
                }

                onMouseIn: {
                    if((itemFrame._btnType & itemFrame._CBUTTON_TYPE_ABNORMAL) === itemFrame._CBUTTON_TYPE_ABNORMAL)
                        return;

                    itemFrame.border.width = 2;
                    itemFrame.border.color = itemListFrame._hoverBorderColor;
                    if(!itemListFrame._isHighlightCurrent) {
                        itemFrame.color              = itemListFrame._hoverColor;
                        itemFrame._btnTextColor      = itemListFrame._hoverTextColor;
                        //itemFrame._btnTextPixelSizeScale  = 1.3;
                    }

                    // Show Function Button
                    //itemFunctionBtn.opacity = 1;

                    // Flag Show Extended List
                    //print('Main List Item IN:', itemFrame._listIndex, itemFrame._isExtendedType);
                    itemFrame._shownExtended = itemFrame._isExtendedType;

                    // ------------------------------------------------------------------------
                    // Entering each row item => Whole frame: _containsMouse true
                    //console.log('Menu List Frame Mouse-Enter Row Item:', menuListMouseArea.containsMouse);
                    itemListFrame._containsMouse = true;
                    itemListFrame._newMouseInItemIndex = itemFrame._listIndex;
                    if(itemListFrame._oldMouseInItemIndex === -1)
                        itemListFrame._oldMouseInItemIndex = itemListFrame._newMouseInItemIndex;

                    // ------------------------------------------------------------------------
                    // EMIT SIGNAL itemClicked(itemIndex)
                    itemListFrame.itemHovered(itemFrame._listIndex);
                }

                onMouseOut: {
                    itemFrame.border.width = 0;
                    if(!itemListFrame._isHighlightCurrent) {
                        itemFrame.color              = itemListFrame._themeColor;
                        itemFrame._btnTextColor      = itemListFrame._commonItemTextColor;
                        //itemFrame._btnTextPixelSizeScale  = 1;
                    }
                    itemListFrame._containsMouse = false;
                    itemListFrame._oldMouseInItemIndex = itemListFrame._newMouseInItemIndex;


                    // Show Function Button
                    //itemFunctionBtn.opacity = itemFunctionBtn._containsMouse?1:0;

                    /*
                    // To differentiate in this MouseOut Event, the current mouse position
                    // is in an item or not:
                    // IF [MOUSE IN NEW ITEM -> MOUSE OUT OLD ITEM]
                    if(itemListFrame._oldMouseInItemIndex !== itemListFrame._newMouseInItemIndex) { // Having just mouse in new item.
                        // Just update: Old->New
                        itemListFrame._oldMouseInItemIndex = itemListFrame._newMouseInItemIndex;
                    }

                    // IF [MOUSE OUT OLD ITEM -> MOUSE IN NEW ITEM]
                    else { // 1- Move to another item 2- Not moving in any item but still around an item 3- Moving out of the list totally.
                        itemListFrame._containsMouse = false;
                        //itemListFrame._containsMouse will then be updated again if Mouse In later.
                    }
                    */
                }

                onMouseDelayOut: {
                    if(itemFrame._isExtendedType) {
                        //print('Main List Item OUT:', extItemListFrame._containsMouse);
                        itemFrame._shownExtended = extItemListFrame._containsMouse;

                        // Reset extItemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                        if(itemFrame._shownExtended === false) {
                            extItemListFrame._oldMouseInItemIndex = extItemListFrame._newMouseInItemIndex
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
    //                            color: itemFunctionBtn._isChecked ? "#ccc" : "transparent"
    //                            border.width: itemFunctionBtn._isChecked ? 1 : 0
    //                            border.color: "#888"
    //                            radius: 4
    //                        }
    //                    }

                    onClicked: {
                        //console.log('SUB FUNCTION INVOKED:', model._itemListIndex);

                        if(model._itemK3DSubOpId > -1) {
                            // Static Invocation
                            MAINGB.k3dRunOp(model._itemK3DSubOpId, model._itemListIndex);
                            // Dynamic Signal Emission
                            itemListFrame.itemFunctionButtonClicked(model._itemListIndex);

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
                    id: extItemListFrame
                    visible             : itemFrame._shownExtended && (extItemListFrame._count > 0)

                    // Anchors depending on itemListFrame._isHorizontal...: - TBD
                    anchors.top         : itemListFrame._isHorizontal? itemFrame.bottom   : itemFrame.top
                    anchors.left        : itemListFrame._isHorizontal? itemFrame.left     : (itemListFrame._extListPosition === 0) ? itemFrame.right : undefined
                    anchors.right       : itemListFrame._isHorizontal? itemFrame.right    : (itemListFrame._extListPosition === 0) ? undefined       : itemFrame.left

                    //width               : itemFrame.width
                    height              : extItemListFrame._count * (itemFrame.height)

                    _themeColor         : itemListFrame._extListThemeColor
                    _isHorizontal       : false
                    _isHighlightCurrent : false
                    _itemBorderRadius   : itemListFrame._itemBorderRadius
                    _itemBorderWidth    : itemListFrame._itemBorderWidth

                    _cITEM_WIDTH        : itemListFrame._cITEM_WIDTH
                    _cITEM_HEIGHT       : itemListFrame._cITEM_HEIGHT
                    // -----------------------------------------------------------------------------------

                    _isSubList          : true
                    _parentItemIndex    : itemFrame._listIndex

                    // ListModel (DATA SOURCE)
                    _itemListModel      : itemListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList

                    Component.onCompleted: {
    //                    if(itemListFrame._isHorizontal) {
    //                        anchors.top         = itemFrame.top;
    //                        anchors.left        = itemFrame.right;
    //                    }
    //                    else {
    //                        anchors.top         = itemFrame.bottom;
    //                        anchors.right       = itemFrame.right;
    //                    }
                        // !!!
                        itemListFrame.extItemIndexUpdated.connect(updateCurrentIndex);
                    }

                    function updateCurrentIndex(newItemIndex) {
                        print('EXT LIST ITEM INDEX UPDATED:', newItemIndex);
                        extItemListFrame.currentIndex = newItemIndex;
                    }

                    onItemSelectionChanged: {
                        itemListFrame.extItemSelectionChanged(parentItemIndex, itemIndex, _itemListModel.get(itemIndex)._itemText);
                    }

                    onItemClicked: {
                        itemListFrame.extItemClicked(_parentItemIndex, itemIndex);
                    }

                    onItemHovered: {
                        itemListFrame.extItemHovered(_parentItemIndex, itemIndex);
                    }

                    onMouseDelayOut: {
                        if(itemFrame._isExtendedType) {
                            //console.log('Extended List Mouse Delay Out:', itemFrame._containsMouse);
                            itemFrame._shownExtended = itemFrame._containsMouse;

                            // Reset extItemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                            if(itemFrame._shownExtended == false) {
                                itemFrame.border.width = 0;
                                extItemListFrame._oldMouseInItemIndex = extItemListFrame._newMouseInItemIndex
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
            _btnTextColor               : itemListFrame._commonItemTextColor
            _btnTextVerticalAlignment   : Text.AlignVCenter
            _btnTextHorizontalAlignment : Text.AlignLeft

            _btnTooltip                 : ""
            _btnTooltipPos              : K3DRC.TOOLTIP_POS.RIGHT
        }
    }

    function addExtendedItem(parentItemIndex, extItemText, extItemOpId, isAppend, tooltip) {
        // LOAD extendedItemComponent ----------------------------------------
        //
        itemListLoader.sourceComponent = extItemComponent;

        // PREPARE extendedItemComponent -------------------------------------
        //
        itemListLoader.item._CK3DOPID        = extItemOpId;
//        if (extItemText.length > K3DRC.TEXT.ITEM_MAX_LEN) {
//            itemListLoader.item._btnTooltip  = extItemText; // Only show Tooltip for over-length text

//            extItemText   = extItemText.substr(0, K3DRC.TEXT.ITEM_MAX_LEN) + "..." +
//                            extItemText.slice(extItemText.lastIndexOf('.'), extItemText.length); // ".STL"
//        }
        if(tooltip !== extItemText) itemListLoader.item._btnTooltip  = tooltip;
        itemListLoader.item._btnText         = extItemText;

        // Other properties kept unchanged!

        // ADD INTO _itemListModel -------------------------------------------
        itemListFrame.addItem(parentItemIndex, itemListLoader.item, isAppend);

        // Reset itemListLoader's sourceComponent to NULL:
        itemListLoader.sourceComponent = undefined;
    } // End addExtendedItem

    function getExtendedItemToolTip(parentItemIndex,extItemIndex){
        print("TOOLTIP : ", getExtItemTooltip(parentItemIndex,extItemIndex))
        //return itemListFrame._itemListModel.get(parentItemIndex)._itemExtendedList.get(extItemIndex)._btnTooltip
        return getExtItemTooltip(parentItemIndex,extItemIndex);
    } // End get Ext Item Tooltip


    function remAllExtendedItems(parentItemIndex) {
        itemListFrame.remAllItems(parentItemIndex);
    }

    // Remove List Item : remExtendedItem(parentItemIndex, extItemIndex);
    function remExtendedItem(parentItemIndex, extItemIndex) {
        itemListFrame.remItem(parentItemIndex, extItemIndex);
    } // End remExtendedItem

    function getExtendedItemListCount(parentItemIndex) {
        return itemListFrame._itemListModel.get(parentItemIndex)._itemExtendedList.count;
    }

    function getExtendedItemCurrentIndex(parentItemIndex) {
        return 0; //extItemListFrame.currentIndex;
    }

    function setExtendedItemCurrentIndex(parentItemIndex, extItemIndex) {
        //extItemListFrame.currentIndex;

        itemListFrame.extItemIndexUpdated(extItemIndex);
    }

    Component.onCompleted: {
    }
} // End Menu Extended List
