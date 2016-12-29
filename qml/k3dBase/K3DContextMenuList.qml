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

// Item List
// -- VIEW (LIST)

K3DItemListBase  {
    id: itemContextMenuListFrame
    //_modelType: K3DRC.LIST_MODEL_TYPE.QML (Default)

    property int _pointedObjectIndex : -1

    _cITEM_WIDTH     : MAINGB._SCREEN_HEIGHT/4.5; // By Default K3DRC.SIZE.ITEM_CONTEXT_MENU_MIN_HEIGHT
    _cITEM_HEIGHT    : (MAINGB._SCREEN_HEIGHT/30.886 < K3DRC.SIZE.ITEM_CONTEXT_MENU_MIN_HEIGHT)?
                        K3DRC.SIZE.ITEM_CONTEXT_MENU_MIN_HEIGHT : MAINGB._SCREEN_HEIGHT/30.886; // // Screen.desktopAvailableHeight / 50
    _themeColor      : K3DRC.COLOR.TRANSPARENT //"#262626"
    _themeSource     : K3DRC.BACKGROUND.CONTEXT_MENU
    _hoverColor      : "#303030"//"#F0F0F0"
    _hoverTextColor  : "white"//"black"
    _hoverBorderColor: "#3A3635"//"#E4E4E2"
    on_ItemsEnabledCountChanged: { // It need to set two times because of this context menu show before operation in K3DItemListBase completed!!!!
        itemContextMenuListFrame.height =  _isHorizontal ? parent.height: (itemContextMenuListFrame._itemsEnabledCount * (_cITEM_HEIGHT)) + _extRectHeight;
        //print("CONTEXT MENU LIST HEIGHT: ", itemContextMenuListFrame.height,itemContextMenuListFrame._itemsEnabledCount, _extRectHeight );
    }
    on_CITEM_HEIGHTChanged: {
        //print("CITEM HEIGHT 2 :", _cITEM_HEIGHT);
        itemContextMenuListFrame.height =  _isHorizontal ? parent.height: (itemContextMenuListFrame._itemsEnabledCount * (_cITEM_HEIGHT)) + _extRectHeight;
    }
    on_ExtRectHeightChanged: {
        itemContextMenuListFrame.height =  _isHorizontal ? parent.height: (itemContextMenuListFrame._itemsEnabledCount * (_cITEM_HEIGHT)) + _extRectHeight;
    }

    spacing: 0
    clip   : false
    property real _maxWidth : 0
    property real _maxExtWidth : 0
    property bool _showTrace : false
    property int  _extListSize : 0
    property real _extRectHeight : 0
    function updateMaxWidth(){
        itemContextMenuListFrame._maxWidth = Math.max(_cITEM_WIDTH, itemContextMenuListFrame._listWidth, itemContextMenuListFrame._maxExtWidth);
    }
    on_ListWidthChanged:    {
        updateMaxWidth();
        //print("LIST WIDTH",itemContextMenuListFrame._listWidth, _maxWidth,_maxExtWidth,_cITEM_WIDTH);
    }
    on_MaxExtWidthChanged: {
        updateMaxWidth();
        //if ( _showTrace) print("LIST 1 WIDTH",itemContextMenuListFrame._listWidth, _maxWidth,_maxExtWidth,_cITEM_WIDTH);
    }
    on_CITEM_WIDTHChanged: {
        itemContextMenuListFrame._listWidth = 0;
        updateMaxWidth();
        //print("CITEM WIDTH CHANGE : ", _cITEM_WIDTH, itemContextMenuListFrame.width, itemContextMenuListFrame._maxWidth,
        //                               itemContextMenuListFrame._listWidth, itemContextMenuListFrame._maxExtWidth);
    }
    on_MaxWidthChanged: { // Actually change listwidth > _cITEM_WIDTH also change thisItemListBase width
        itemContextMenuListFrame.width = itemContextMenuListFrame._maxWidth;
    }

//    Behavior on height  {
//        PropertyAnimation { duration: MAINGB._CANIMATION_DURATION; }
//    }

    property string _commonItemTextColor    : "#7F7F7F"
    property string _groupHeaderColor

    // [Express disabled items]
    function setupLimitedItems() {
        for (var i = 0; i < _count; i++) {
            setMainItemEnabled(i, isMainItemEnabled(i));
        }
    }

    // Each List has its own self-defined _contextMenuList[]
    //
    function addItemFromList(itemIndex) {
        gbLoader.sourceComponent = _contextMenuList[itemIndex];
        gbLoader.item._btnTextPosition  = K3DRC.BUTTON_TEXT_POS.RIGHT;
        gbLoader.item._btnTooltipPos    = K3DRC.TOOLTIP_POS.TOP; //!!
        //gbLoader.item._btnSourceSize    = 20;
        gbLoader.item._btnTextPixelSizeScale = (itemIndex === 0) ? 1 : 0.9;
        gbLoader.item._btnTextVerticalAlignment   = Text.AlignVCenter;
        gbLoader.item._btnTextHorizontalAlignment = Text.AlignLeft;
        itemContextMenuListFrame.addItem(gbLoader.item, true);
    }

    function setExtItemListEnabled(itemIndex, enabled) {
        var itemNode = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP[itemIndex] :
                                                                    _itemListModel.get(itemIndex); // _itemListModel.get(parentIndex);

        var count, i;
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            if (itemNode.extendedListType === K3DRC.LIST_MODEL_TYPE.CPP) {      // C++ Data List Model
                count = itemNode.extendedList.length;

                for(i = 0; i < count; i++)
                    itemNode.extendedList.at(i).enabled = enabled;
            }
        }
        else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML){
            if(itemNode._extListModelType === K3DRC.LIST_MODEL_TYPE.QML) {  // QML List Model
                count = itemNode._itemExtendedList.count;

                for(i = 0; i < count; i++)
                    itemNode._itemExtendedList.setProperty(i, "_itemEnabled", enabled);
            }
        }
    }

    // == Signals
    signal itemExtAdded(int parentItemIndex, string extItemText, int extItemOpId, bool isAppend)
    signal itemExtAddedII(int parentItemIndex, var item, bool isAppend)
    signal itemExtRemoved(int parentItemIndex, int extItemIndex)

    // EXT ITEM LIST ---------------------------------------------------------------------------
    // K3DContextMenuList -> K3DItemListBase:
    signal extItemIndexUpdated(int extItemNewIndex)
    signal extItemListUpdated(int parentItemIndex, var listModel); // listModel is a CPP Data List Model.
    signal extItemListUpdated2(int parentItemIndex, var extItemList, int extItemOpId); // []

    // K3DItemListBase -> K3DContextMenuList:
    signal extItemSelectionChanged(int itemIndex, int extItemIndex, string extItemText)
    signal extItemClicked(int itemIndex, int extItemIndex)
    signal extItemHovered(int itemIndex, int extItemIndex)

    onItemExtAdded : {
        //itemListFrame.addExtendedItem(parentItemIndex, extItemText, extItemOpId, isAppend);
    }
    onItemExtRemoved : {
        //itemListFrame.remExtendedItem(parentItemIndex, extItemIndex, extItemOpId);
    }

    onExtItemListUpdated2 : {
        //itemListFrame.updateExtendedItemList(parentItemIndex, extItemList, extItemOpId);
    }
    property int _latestExtendedParentIndex  : -1 // Used to mark the current item index, of which the extended list is updated.
    // The binding would signal the item of the index to update its '_extListModel' member.
    property int _currentExtendedParentIndex : -1
    // [C++} LIST MODEL
    property var _extItemCPPListModel // Used to store the cpp list model.
    // [QML] LIST MODEL
    function getExtItemQMLListModel(parentItemIndex) {
        return itemContextMenuListFrame._itemListModel.get(parentItemIndex)._itemExtendedList;
    }
    property real _extListWidth
    onExtItemListUpdated: {
        itemContextMenuListFrame._extListWidth = -1;
        //print("UPDATE EXT ITEM LIST");
        _latestExtendedParentIndex             = parentItemIndex;
        _extItemCPPListModel                   = listModel;

        //print('EXT LIST ITEM INDEX UPDATED:', parentItemIndex, _extItemListModel, listModel[0].text, listModel[0].opId);
        itemContextMenuListFrame._itemListModel.setProperty(parentItemIndex,"_itemExtListModel",     _extItemCPPListModel);
        itemContextMenuListFrame._itemListModel.setProperty(parentItemIndex,"_itemExtListModelType", K3DRC.LIST_MODEL_TYPE.CPP);
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
            ...
        }
        */
    //}

    function hide() {
        itemContextMenuListFrame.visible = false;
        if(_currentExtendedParentIndex !== -1) {
            if(itemContextMenuListFrame._itemListModel.get(_currentExtendedParentIndex)._itemForceShownExtended === false)
                itemContextMenuListFrame._itemListModel.setProperty(_currentExtendedParentIndex,"_itemShownExtended", false);
            itemContextMenuListFrame._itemListModel.setProperty(_currentExtendedParentIndex,"_itemExtendedDirection", K3DRC.EXTEND_DIRECTION.DOWNWARD);
        }
    }
    property bool _showTickMark : false
    // -- DELEGATE (ITEM UI)
    delegate: itemListDelegate
    Component {
        id: itemListDelegate

        Rectangle {
            id: itemFrameRect
            z : itemContextMenuListFrame.z + 1
            clip  : false //!!! - Tooltip
            width : itemFrame.enabled ? itemContextMenuListFrame._maxWidth : 0//width//itemContextMenuListFrame._isHorizontal? itemContextMenuListFrame._cITEM_WIDTH :
                                                         //  itemContextMenuListFrame.width
            height: itemFrame.enabled ? (itemFrame.height + extItemListFrame.height) : 0
            color : itemContextMenuListFrame._isHighlightCurrent? (ListView.isCurrentItem ? itemContextMenuListFrame._highlightSelectedColor : itemContextMenuListFrame._themeColor):
                                                                  itemContextMenuListFrame._themeColor
            // Main Item Frame (itemContextMenuListFrame._isHorizontal: false)
            K3DItem {
                id: itemFrame
                z : itemFrameRect.z + 1
                anchors {
                    top   : itemFrameRect.top
                    left  : itemFrameRect.left
                    right : itemFrameRect.right
                }
                property real _width
                function calculateItemWidth() {
                    itemFrame._width = 0;
                    itemFrame._width = K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_TEXT_ONLY) ? Math.max(itemContextMenuListFrame._cITEM_WIDTH, _labelTextWidth *1.4) :
                                                                                             //K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ICON_TEXT) ? Math.max(itemContextMenuListFrame._cITEM_WIDTH, (itemContextMenuListFrame._cITEM_HEIGHT / 2 + itemFrame._btnTextWidth *1.2)) :
                                                                                             K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ICON_TEXT | _CBUTTON_TYPE_ABNORMAL) ? Math.max(itemContextMenuListFrame._cITEM_WIDTH, _labelTextWidth *1.4) :
                                                                                             itemContextMenuListFrame._cITEM_WIDTH;
                    //print("itemFrame._width", itemFrame._width,itemContextMenuListFrame._listWidth );
                }

                // _btnTextWidth delay 1 click to _btnText
                property real _itemListFrameCItemWidth : itemContextMenuListFrame._cITEM_WIDTH
                on_ItemListFrameCItemWidthChanged: {
                    calculateItemWidth();
                }

                on_LabelTextWidthChanged: {
                    calculateItemWidth();
                }
                on_WidthChanged: {
                    if (itemFrame._listIndex === 0) itemContextMenuListFrame._listWidth = 0;
                    itemContextMenuListFrame._listWidth = Math.max(itemContextMenuListFrame._listWidth, itemFrame._width);
                    //if ( _showTrace) print("LIST 2 WIDTH",itemFrame._width, itemContextMenuListFrame._listWidth, _maxExtWidth, _maxWidth,itemFrame.width,itemFrameRect.width,_labelTextWidth *1.4);
                }
                height: !enabled ? 0 : itemContextMenuListFrame._isHorizontal? itemContextMenuListFrame.height : itemContextMenuListFrame._cITEM_HEIGHT

                clip  : !enabled //false//!!! - Tooltip
                color : K3DRC.COLOR.TRANSPARENT

                border.width: itemContextMenuListFrame._itemBorderWidth
                border.color: "white"
                radius      : itemContextMenuListFrame._itemBorderRadius
                antialiasing: true

                //_touchButtonImage.cache: false
                _touchButtonImage.sourceSize.width             : itemFrame.height/2
                _touchButtonImage.sourceSize.height            : itemFrame.height/2
                _touchButtonImage.anchors.leftMargin: K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ICON_TEXT) ? itemFrame.height/ 1.8 : 0 // TEMPORAL SOLUTION!
                _touchButtonMouseArea.anchors.fill  : K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ICON_ONLY) ? _themeRect: itemFrame

                visible                : enabled ? model._itemVisible : 0// !!!
                enabled                : model._itemEnabled
                _isChecked             : model._itemChecked
                opacity                : enabled ? 1: 0.5
                _themeColor            : K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL) ? "#1C1C1C" : ""
                _themeOpacity          : itemContextMenuListFrame._themeOpacity
                _themeSource           : K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL) ? "" :
                                         itemContextMenuListFrame._themeSource
                _listIndex             : model._itemListIndex // !NOTE: Here, we can use var 'index' <-> _listIndex
                _isItemOperationNeedIndex: model._itemOperationNeedIndex
                _CK3DOPID              : model._itemK3DOpId
                _CK3DSUBOPID           : model._itemK3DSubOpId
                _CK3DITEMID            : model._itemK3DItemId
                _btnType               : model._itemBtnType
                _btnTooltip            : model._itemTooltip
                _btnTooltipPos         : model._itemTooltipPos
                _btnText               : K3DUTIL.TEXT.getContextMenuItemShrinkedText(getItemText(itemFrame._listIndex))
                property alias _labelTextWidth : text.width
                K3DText{
                    id: text
                    height: 0
                    visible : false
                    text : K3DUTIL.TEXT.getContextMenuItemShrinkedText(getItemText(itemFrame._listIndex))
                }
                _btnTextColor          : itemContextMenuListFrame._isHighlightCurrent? (ListView.isCurrentItem ?
                                                                                        itemContextMenuListFrame._highlightSelectedTextColor : model._itemTextColor):
                                         model._itemTextColor
                _btnTextFontSource     : K3DRC.FONT.MAVEN_PRO_BOLD // !!
                _btnTextBold           : true; // !!
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
                _extendedMark.source   : K3DRC.ICON.EXTENSION_MARK_GRAY
                _shownExtended         : model._itemShownExtended
//                {
//                        print('SHOWN EXTENDED:', itemFrame._listIndex, model._itemShownExtended);
//                }
                _isForceShownExtended  : model._itemForceShownExtended
                _isExtendedType        : model._itemIsExtended
                _extendingState        : model._itemExtendedDirection
                _showExtendedMark      : ((itemFrame._extListModelType === K3DRC.LIST_MODEL_TYPE.CPP) && (_extListModel === undefined))? false:
                                         model._itemIsExtended && (extItemListFrame._listModelSize > 0)  ///!!! IMPORTANT: Use extItemListFrame._listModelSize
                _extListModel          : {
                    //print('EXT LIST MODEL:', model._itemExtListModel, itemContextMenuListFrame._extItemListModel);
                    return (itemFrame._listIndex === itemContextMenuListFrame._latestExtendedParentIndex) ?
                           itemContextMenuListFrame._extItemCPPListModel : undefined;
                }
                _extListModelType      : model._itemExtListModelType

//                _extendedMark.anchors.left          : itemFrame.right
//                _extendedMark.anchors.leftMargin    : - itemFrame.width*2.5
//                _extendedMark.anchors.verticalCenter: itemFrame.verticalCenter

                _stayPut : _isExtendedType || K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL)
                onClicked: {
                    // !NOTE: DON'T USE model._item... here, use the property directly itself!
                    if(itemFrame.enabled) {
                        // Update currentIndex
                        //console.log('MAIN LIST-CLICKED:', itemListFrame.currentIndex, itemFrame._listIndex);
                        if(itemContextMenuListFrame.currentIndex !== itemFrame._listIndex) {
                            itemContextMenuListFrame.currentIndex = itemFrame._listIndex;
                            //console.log('NEW INDEX:', itemFrame._listIndex, index);

                            itemContextMenuListFrame.itemSelectionChanged(-1, itemContextMenuListFrame.currentIndex);
                        }

                        if(false === itemFrame._isExtendedType) {
                            // Reset itemListFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                            itemContextMenuListFrame._oldMouseInItemIndex = itemContextMenuListFrame._newMouseInItemIndex
                                                               = -1;
                        }
                        else {
                            // Mark current item as being extended:
                            itemContextMenuListFrame._currentExtendedParentIndex = itemFrame._listIndex;

                            // Flag Show Extended List
                            //if(itemContextMenuListFrame._itemListModel.get(_currentExtendedParentIndex)._itemForceShownExtended === false) // STILL LET IT FLIPPED
                            //{
                                //console.log('Main List Item IN', itemFrame._listIndex);
                                var tmp = !model._itemShownExtended;
                                itemContextMenuListFrame._itemListModel.setProperty(itemFrame._listIndex, "_itemShownExtended", tmp);

                                // Already implemented in K3DButton.onClicked()
    //                            if(itemFrame._extendingState === K3DRC.EXTEND_DIRECTION.UPWARD)
    //                                tmp = K3DRC.EXTEND_DIRECTION.DOWNWARD;
    //                            else
    //                                tmp = K3DRC.EXTEND_DIRECTION.UPWARD;
    //                            itemContextMenuListFrame._itemListModel.setProperty(itemFrame._listIndex, "_itemExtendedDirection", tmp);
                            //}

                            //_shownExtended = model._itemShownExtended;
                            // Reset extItemList's _oldMouseInItemIndex. _newMouseInItemIndex:
                            if(model._itemShownExtended  === false) {
                                extItemList._oldMouseInItemIndex = extItemList._newMouseInItemIndex
                                                                 = -1;
                            }
                        }

                        // ------------------------------------------------------------------------
                        // EMIT SIGNAL itemClicked(itemIndex)
                        itemContextMenuListFrame.itemClicked(itemFrame._listIndex);
                    }
                }

                onMouseIn: {
                    if (K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL)) {
                        return;
                    }

                    itemFrame.border.width = 0;
                    itemFrame.border.color = itemContextMenuListFrame._hoverBorderColor;
                    if(!itemContextMenuListFrame._isHighlightCurrent) {
                        itemFrame.color                = itemContextMenuListFrame._hoverColor;
                        itemFrame._btnTextColor        = itemContextMenuListFrame._hoverTextColor;
                        //itemFrame._btnTextPixelSizeScale  = 1.3;
                    }

                    // Show Function Button
                    //itemFunctionBtn.opacity = 1;

                    // Invert Extending Mark:
                    if(itemFrame._showExtendedMark) {
                        itemFrame._extendedMark.source = K3DRC.ICON.EXTENSION_MARK_BLACK;
                    }

                    // ------------------------------------------------------------------------
                    // Entering each row item => Whole frame: _containsMouse true
                    //console.log('Menu List Frame Mouse-Enter Row Item:', menuListMouseArea.containsMouse);
                    itemContextMenuListFrame._containsMouse = true;
                    itemContextMenuListFrame._newMouseInItemIndex = itemFrame._listIndex;
                    if(itemContextMenuListFrame._oldMouseInItemIndex === -1)
                        itemContextMenuListFrame._oldMouseInItemIndex = itemContextMenuListFrame._newMouseInItemIndex;

                    // ------------------------------------------------------------------------
                    // EMIT SIGNAL itemClicked(itemIndex)
                    itemContextMenuListFrame.itemHovered(itemFrame._listIndex);
                }

                onMouseOut: {
                    if (K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL)) {
                        return;
                    }

                    itemFrame.border.width = 0;
                    if(!itemContextMenuListFrame._isHighlightCurrent) {
                        itemFrame.color          = itemContextMenuListFrame._themeColor;
                        itemFrame._themeSource   = itemContextMenuListFrame._themeSource;
                        itemFrame._btnTextColor  = itemContextMenuListFrame._commonItemTextColor;
                        //itemFrame._btnTextPixelSizeScale  = 1;
                    }

                    // -------------------------------------------------------------------------
                    // Inver Extending Mark:
                    if(itemFrame._showExtendedMark)
                        itemFrame._extendedMark.source = K3DRC.ICON.EXTENSION_MARK_GRAY;

                    // -------------------------------------------------------------------------
                    itemContextMenuListFrame._containsMouse = false;
                    itemContextMenuListFrame._oldMouseInItemIndex = itemContextMenuListFrame._newMouseInItemIndex;

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
                            itemContextMenuListFrame.itemFunctionButtonClicked(model._itemListIndex);

                            //  Here we can also use:
                            //  - Using K3D_OP_ID: static connection, best used for statically connecting to C++ gbK3DQMLAdapter's methods.
                            //  - Using signal: dynamic connection, used both for connecting to C++ gbK3DQMLAdapter's methods and QML methods.
                        }
                    }
                } // End Item Sub Function Button

                // Extended Sub Item List -----------------------------------------------------------------
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // ########################################################################################
            } // End itemFrame

            // Group Title Mark
            K3DRectangle {
                z : itemFrameRect.z + 1
                anchors {
                    top        : itemFrame.top
                    bottom     : itemFrame.bottom
                    left       : itemFrame.left
                    leftMargin : width
                }
                color : (itemFrame._btnText === MAINGB.tr(qsTr("GENERAL")))? "white" : itemContextMenuListFrame._groupHeaderColor
                width : visible ?
                        itemFrame.height/5 : 0
                visible: K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL)
            }

            // Extended List Frame [K3DRectangle]
            K3DRectangle {
                id: extItemListFrame
                z : itemFrameRect.z + 1
                border.width        : 1
                border.color        : "#282828"

                // Anchors depending on itemListFrame._isHorizontal...: - TBD
                anchors.top         : itemFrame.bottom
                anchors.left        : itemFrameRect.left
                width               : itemContextMenuListFrame._maxWidth//

                enabled             : //!itemListFrame.visible ? false:
                                      (extItemList.model === undefined) ? false :
                                      (itemFrame._shownExtended && (extItemListFrame._listModelSize > 0))? true: false
                height              : {
                    itemContextMenuListFrame._extRectHeight = enabled ? _originalHeight : 0
                    return enabled ? _originalHeight : 0;
                }
                opacity             : enabled ? 1: 0
                color               : "#262626" // itemListFrame._themeColor

                property int _originalHeight : itemContextMenuListFrame._cITEM_HEIGHT * extItemListFrame._listModelSize ///!!! IMPORTANT: Use extItemListFrame._listModelSize

                Behavior on height  {
                    PropertyAnimation { duration: MAINGB._CANIMATION_DURATION;}
//                    SequentialAnimation {
//                        ScriptAction {
//                            script: {
////                                if(itemFrame._shownExtended)
////                                    itemContextMenuListFrame.height += extItemListFrame._originalHeight;
////                                else
////                                    itemContextMenuListFrame.height -= extItemListFrame._originalHeight;
//                            }
//                        }
//                        PropertyAnimation { duration: MAINGB._CANIMATION_DURATION;}
//                    }
//                    //SpringAnimation { duration: MAINGB._CANIMATION_DURATION;  spring: 1.2; damping: 0.2; mass: 1.2 }
                }
                Behavior on opacity {
                    PropertyAnimation { duration: MAINGB._CANIMATION_DURATION; }
                }

                property int _listModelSize : {
//                    if (itemFrame._extListModelType !== K3DRC.LIST_MODEL_TYPE.QML)
//                        print('CONTEXT MENU LIST TYPE', itemFrame._extListModelType);

                    if (itemFrame._extListModelType === K3DRC.LIST_MODEL_TYPE.CPP) {      // C++ Data List Model
                        //print('CONTEXT MENU LIST - TYPE 0', itemFrame._listIndex, itemFrame._extListModel.length);
                        if(itemFrame._extListModel !== null) {
                            itemContextMenuListFrame._extListSize = itemFrame._extListModel.length;
                            return itemFrame._extListModel.length;
                        }
                        else return 0;
                    }
                    else if(itemFrame._extListModelType === K3DRC.LIST_MODEL_TYPE.QML) {  // QML List Model
                        //var  count = itemContextMenuListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList.count;
                        //for(var i =0 ; i < count; i++)
                            //print('CONTEXT MENU LIST - TYPE 1: ', itemFrame._listIndex, itemContextMenuListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList.get(i)._itemText);
                        itemContextMenuListFrame._extListSize = itemContextMenuListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList.count;
                        return itemContextMenuListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList.count;
                    }
                    else
                        return 0;
                }
                // Ext Item List [ListView]
                K3DItemListBase {
                    id: extItemList
                    z: extItemListFrame.z + 1
                    anchors.fill        : parent
                    _isHorizontal       : false
                    _isHighlightCurrent : false
                    _itemBorderRadius   : itemContextMenuListFrame._itemBorderRadius
                    _itemBorderWidth    : itemContextMenuListFrame._itemBorderWidth

                    _cITEM_WIDTH        : MAINGB._SCREEN_HEIGHT/4.5// width by default as _cITEM_WIDTH
                    on_CITEM_WIDTHChanged: {
                        extItemList._listWidth = 0;
                    }
                    _cITEM_HEIGHT       : itemContextMenuListFrame._cITEM_HEIGHT
                    on_ListWidthChanged:   {
                        itemContextMenuListFrame._maxExtWidth = Math.max(extItemList._listWidth, MAINGB._SCREEN_HEIGHT/4.5);// It must be so
                        //if ( _showTrace) print("EXT LIST WIDTH",itemContextMenuListFrame._maxExtWidth,extItemList._listWidth, extItemList.width);
                    }
                    height              : parent.height //(model === undefined)? 0 : itemContextMenuListFrame._cITEM_HEIGHT * extItemListFrame._listModelSize
                    // -----------------------------------------------------------------------------------

                    _isSubList          : true
                    _parentItemIndex    : itemFrame._listIndex

                    // ListModel (DATA SOURCE)
                    model                  : (extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)? extItemList._itemListModelCPP:
                                             (extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML)?
                                             itemContextMenuListFrame._itemListModel.get(itemFrame._listIndex)._itemExtendedList  : _itemListModel
                    _itemListModelCPP      : (extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? itemFrame._extListModel : []
                    _modelType             : itemFrame._extListModelType
                    delegate               : extItemComponent

                    Component.onCompleted: {
                                    //getModelText();
                        itemContextMenuListFrame.extItemIndexUpdated.connect(updateCurrentIndex);
                    }
                    function updateCurrentIndex(newItemIndex) {
                        //print('EXT LIST ITEM INDEX UPDATED:', newItemIndex);
                        extItemList.currentIndex = newItemIndex;
                    }

                    // SELF-DEFINED EXT ITEM DELEGATE (TO ADAPT TO DIFFERENT TYPES OF MODEL DATA)
                    Component {
                        id: extItemComponent

                        K3DItem {
                            id: extItemFrame
                            width: extItemList.width
                            property real _width
                            _width : K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_TEXT_ONLY)
                                    ? Math.max(extItemList._cITEM_WIDTH, _btnTextWidth *1.2) : extItemList._cITEM_WIDTH
                            on_WidthChanged: {
                                if (itemContextMenuListFrame._extListWidth === -1) extItemList._listWidth = 0;
                                extItemList._listWidth = Math.max(extItemList._listWidth, extItemFrame._width);
                                itemContextMenuListFrame._extListWidth = extItemList._listWidth;
                                //print("EXT LIST 3 WIDTH",itemContextMenuListFrame._extListWidth, extItemList._listWidth,extItemFrame.width);
                            }

                            height                      : itemContextMenuListFrame._cITEM_HEIGHT

                            //color                       : itemContextMenuListFrame._themeColor
                            antialiasing: true
                            _touchButtonImage.sourceSize.width     : itemFrame.height/2 ///!!! IMPORTANT
                            _touchButtonImage.sourceSize.height    : itemFrame.height/2
                            _touchButtonImage.anchors.leftMargin: K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ICON_TEXT) ? 20: 0 // TEMPORAL SOLUTION!

                            _listIndex                  : index
                            _CK3DOPID                   : getOpId()
                            _isItemOperationNeedIndex   : (extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                            _btnType                    : getBtnType()
                            _btnText                    : K3DUTIL.TEXT.getContextMenuItemShrinkedText(getBtnText())
                            _btnTextColor               : extItemFrame._containsMouse? "white" : itemContextMenuListFrame._commonItemTextColor
                            _btnTextPosition            : getBtnTextPosition()

                            _btnTextBold                : true
                            _btnTextStyle               : Text.Outline
                            _btnTextStyleColor          : "#333333"
                            _btnTextVerticalAlignment   : Text.AlignVCenter
                            _btnTextHorizontalAlignment : Text.AlignLeft
                            _touchButtonText._fontSource : K3DRC.FONT.MAVEN_PRO_BOLD
                            //_btnTextPixelSizeScale     : 1.1

                            _btnTooltip                 : _btnText.length < K3DRC.TEXT.ITEM_MAX_LEN ? "" :
                                                          getBtnText()
                            _btnTooltipPos              : K3DRC.TOOLTIP_POS.RIGHT
                            //_iconBaseSource             : _btnNormalSource
                            _isHoverChangeImage         : true
                            _btnNormalSource            : getBtnNormalSource()
                            _btnHoveredSource           : getBtnHoveredSource()
                            _btnPressedSource           : _btnNormalSource
                            property int _currentItem   : 0
                            onClicked:{
                                //print(" Sub Item Index", extItemFrame._listIndex);
                                extItemList.itemClicked(extItemFrame._listIndex);
                            }
                            _stayPut                    : K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ABNORMAL)

                            function getOpId() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                                    return model.modelData.opId;
                                else if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML)
                                    return model._itemK3DOpId;
                                else
                                    return -1;
                            }

                            function getItemId() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                                    return model.modelData.itemId;
                                else if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML)
                                    return model._itemK3DItemId;
                                else
                                    return -1;
                            }

                            function getBtnType() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                                    return _CBUTTON_TYPE_TEXT_ONLY;
                                else if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML)
                                    return _CBUTTON_TYPE_ICON_TEXT;
                                else
                                    return -1;
                            }

                            function getBtnTextPosition() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML)
                                    return model._itemTextPosition;
                                else
                                    return K3DRC.BUTTON_TEXT_POS.TEXTONLY_LEFT;
                            }

                            function getBtnText() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                                    return K3DUTIL.TEXT.getContextMenuItemText(model.modelData.text);
                                else if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML)
                                    return K3DUTIL.TEXT.getContextMenuItemText(model._itemText);
                                else
                                    return "";
                            }

                            function getBtnNormalSource() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                                    return "";
                                else if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML){
                                    //print("VIEW MODE NORMAL SOURCE", model._itemNormalSource );
                                    return model._itemNormalSource;
                                }
                                else
                                    return "";
                            }

                            function getBtnHoveredSource() {
                                if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.CPP)
                                    return "";
                                else if(extItemList._modelType === K3DRC.LIST_MODEL_TYPE.QML){
                                    //print("VIEW MODE HOVER SOURCE", model._itemHoveredSource);
                                    return model._itemHoveredSource;
                                }
                                else
                                    return "";
                            }
                        } // End extItemFrame
                    } // END EXT ITEM COMPONENT

                    onItemSelectionChanged: {
                        itemContextMenuListFrame.extItemSelectionChanged(parentItemIndex, itemIndex, itemContextMenuListFrame._itemListModel.get(itemIndex)._itemText);
                    }

                    onItemClicked: {

                        itemContextMenuListFrame.extItemClicked(_parentItemIndex, itemIndex);
                    }

                    onItemHovered: {
                        itemContextMenuListFrame.extItemHovered(_parentItemIndex, itemIndex);
                    }

                    onMouseDelayOut: {
                        if(itemFrame._isExtendedType) {
                            //console.log('Extended List Mouse Delay Out:', itemFrame._containsMouse);
                            itemFrame._shownExtended = itemFrame._containsMouse;

                            // Reset extItemList's _oldMouseInItemIndex. _newMouseInItemIndex:
                            if(itemFrame._shownExtended=== false) {
                                itemFrame.border.width = 0;
                                extItemList._oldMouseInItemIndex = extItemList._newMouseInItemIndex
                                                                      = -1;
                            }
                        }
                    }

                } // End K3DItemListBase - Extended List
                // Tick Mark on View Mode
                K3DImage{
                    id         : tickMarkViewMode
                    source     :  K3DRC.ICON.TICK_MARK
                    height     : itemContextMenuListFrame._cITEM_HEIGHT/3
                    width      :  height
                    x          : extItemListFrame.width - 2 * tickMarkViewMode.width
                    y          : itemContextMenuListFrame._tickMarkYPosition
                    visible    : itemContextMenuListFrame._showTickMark
                }
                // Tick Mark on Transparent Button
                K3DImage{
                    id         : tickMarkTransparent
                    source     :  K3DRC.ICON.TICK_MARK
                    height     : itemContextMenuListFrame._cITEM_HEIGHT/3
                    width      :  height
                    x          : extItemListFrame.width - 2 * tickMarkViewMode.width
                    y          : extItemListFrame.height - 2 * tickMarkTransparent.height
                    visible    : itemContextMenuListFrame._showTickMark && gbMainContextMenuIndividual._isPointedObjectTransparent
                }
            } // End K3DRectangle - Extended List Frame
        } // End K3DRectangle - Main Delegate Item
    } // End List Delegate

    // == Functions/Methods
    //
    property real _tickMarkYPosition : -1

    function addExtendedItem(parentItemIndex, extItem, isAppend) {
        if(extItemList._parentItemIndex === parentItemIndex)
            extItemList.addItem(extItem,isAppend);
   } // End addExtendedItem

//    function addExtendedItem(parentItemIndex, extItemText, extItemOpId, isAppend) {
    //        // LOAD extendedItemComponent ----------------------------------------
    //        //
    //        itemListLoader.sourceComponent = extItemComponent;

    //        // PREPARE extendedItemComponent -------------------------------------
    //        //
    //        itemListLoader.item._CK3DOPID        = extItemOpId;
    //        if (extItemText.length > K3DRC.TEXT.ITEM_MAX_LEN) {
    //            itemListLoader.item._btnTooltip  = extItemText; // Only show Tooltip for over-length text

    //            extItemText   = extItemText.substr(0, K3DRC.TEXT.ITEM_MAX_LEN) + "..." +
    //                            extItemText.slice(extItemText.lastIndexOf('.'), extItemText.length); // ".STL"
    //        }
    //        itemListLoader.item._btnText         = extItemText;

    //        // Other properties kept unchanged!

    //        // ADD INTO _itemListModel -------------------------------------------
    //        itemListFrame.addItem(parentItemIndex, itemListLoader.item, isAppend);

    //        // Reset itemListLoader's sourceComponent to NULL:
    //        itemListLoader.sourceComponent = undefined;
    //    } // End addExtendedItem

    //    // Remove List Item : remExtendedItem(parentItemIndex, extItemIndex);
    //    function remExtendedItem(parentItemIndex, extItemIndex) {
    //        itemListFrame.remItem(parentItemIndex, extItemIndex);
    //    } // End remExtendedItem

    Component.onCompleted: {
    }
} // End Menu Extended List
