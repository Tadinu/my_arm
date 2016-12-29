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
//ScrollView {
ListView  {
    id: itemListBaseFrame

    orientation: itemListBaseFrame._isHorizontal? ListView.Horizontal:ListView.Vertical
    //layoutDirection: Qt.LeftToRight
    interactive: false
    spacing: 0//10
    //cacheBuffer:

    // -- MODEL (Data Source)
    //!!HERE: We can see there is minor difference between Property vs Child Element
    model : (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP : _itemListModel
    property ListModel _itemListModel : ListModel {
                                            /* The attributes of ListElement is self-defined and used with Delegate!!!
                                            ListElement {
                                                _itemTooltip:       MAINGB.tr(qsTr("Tooltip"))
                                                _itemDesc   :       MAINGB.tr(qsTr("Description"))
                                                _itemHoverChangeImage : true
                                                _itemNormalSource:  "qrc:///res/maincontrol/home_active.png"
                                                _itemHoveredSource: "qrc:///res/maincontrol/home_hover.png"
                                                _itemPressedSource: "qrc:///res/maincontrol/home_inactive.png"
                                                _itemCheckedSource: ""
                                            }
                                            */
                                        }

    property var    _itemListModelCPP  : []

    property int    _count         : (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP.length :  // var []
                                                                                  _itemListModel.count        // LIST MODEL
    // This Property _itemsEnabledCount have value after list added ( one by one || finish)
    // So with CPP model, we add after program start up, so it update late one time.
    function getItemEnabledCount(){
        var count = 0;
        for ( var i = 0; i < _count ; i++){
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
                if(_itemListModelCPP[i].enabled === true)
                    count+=1;
            }
            else {
                if(_itemListModel !== null && _itemListModel.get(i)._itemEnabled === true)
                    count+=1;
            }
        }
        //print("ITEMSLIST ENABEL : ", _count, count);
        return count;
    }

    property int _itemsEnabledCount : getItemEnabledCount()
    // Width MainFileMenuPanel, CPP model, need set this flag is true
    property bool _isUpdateWidthHeightOutside : false
    on_ItemsEnabledCountChanged: {
        if(!_isUpdateWidthHeightOutside) {
            itemListBaseFrame.updateWidthHeight();
        }
    }


    property int    _modelType     :  K3DRC.LIST_MODEL_TYPE.QML

    delegate: itemListBaseDelegate

    signal modelChanged()
    onModelChanged: {
        //model  = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP : _itemListModel;
        _count = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP.length :  // var []
                                                              _itemListModel.count        // LIST MODEL
    }

    // ---------------------------
    //snapMode: ListView.SnapToItem

    // Draggable/Flickable
    // interactive: true

    // == Properties
    property bool   _isHorizontal             : true
    property bool   _isHighlightCurrent       : false

    property real   _itemBorderWidth          : 0
    property real   _itemBorderRadius         : 0

    property real _cITEM_WIDTH
    property real _cITEM_HEIGHT
    property real _listWidth

    property string _themeColor             : K3DRC.COLOR.TRANSPARENT
    property real   _themeOpacity           : 1
    property string _themeSource

    property string _highlightSelectedColor     : "#E4E4E2"
    property string _highlightSelectedTextColor : "white"
    property string _hoverColor             : "white"
    property string _hoverTextColor         : "#1E1E1E" //"black"
    property string _hoverBorderColor       : "orange"

    // SubList --
    property bool _isSubList                : false
    property int  _parentItemIndex          : -1 // (_isSubList==false): Always -1, else set from outside

    // -- MODEL (Data Source)
    /* The attributes of ListElement is self-defined and used with Delegate!!!
    ListElement {
        _itemTooltip:       "Tooltip"
        _itemDesc   :       "Description"
        _itemNormalSource:  "qrc:///res/maincontrol/home_active.png"
        _itemHoveredSource: "qrc:///res/maincontrol/home_hover.png"
        _itemPressedSource: "qrc:///res/maincontrol/home_inactive.png"
    }
    */
    property real _width : _isHorizontal ? 0 : Math.max(_cITEM_WIDTH, _listWidth)
    width : _width
    height: _isHorizontal ? parent.height : 0 // (itemListBaseFrame._itemsEnabledCount * (itemListBaseFrame._cITEM_HEIGHT))
    contentHeight: height

    //radius: 10
    antialiasing: true
    smooth      : true

    // Used to detect the whole list (either the list itself or at least one item containing mouse)
    property bool _containsMouse
    property int  _newMouseInItemIndex : -1
    property int  _oldMouseInItemIndex : -1
    property real _CITEM_IMAGE_SIZE

    // == Signals
    signal mouseDelayOut
    signal itemFunctionButtonClicked(int itemIndex)
    signal itemSelectionChanged(int parentItemIndex, int itemIndex)
    signal itemClicked(int itemIndex)
    signal itemHovered(int itemIndex)

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

    // Timer --
    property int _intervalTime: 500
    Timer {
        id: delayTimer
        interval: itemListBaseFrame._intervalTime
        repeat: false
        triggeredOnStart: false
        running: false

        onTriggered: {
            itemListBaseFrame.mouseDelayOut();
        }
    }
    // -- DELEGATE (ITEM UI)
    Component {
        id: itemListBaseDelegate

        K3DItem {
            id: itemFrame
            width : !enabled ? 0 : (K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_TEXT_ONLY) || K3DUTIL.FLAG.test(_btnType, _CBUTTON_TYPE_ICON_TEXT))
                    ? Math.max(itemListBaseFrame._cITEM_WIDTH, _btnTextWidth *1.3)         :
                      itemListBaseFrame._cITEM_WIDTH
                    //itemListBaseFrame._isHorizontal? itemListBaseFrame._cITEM_WIDTH : itemListBaseFrame.width

            onWidthChanged: {
                itemListBaseFrame._listWidth = Math.max(itemListBaseFrame._listWidth, itemFrame.width);
            }
            height: !enabled ? 0  :  itemListBaseFrame._isHorizontal ? itemListBaseFrame.height/2     : itemListBaseFrame._cITEM_HEIGHT
            clip  : false
            color : itemListBaseFrame._isHighlightCurrent?
                        (ListView.isCurrentItem ? itemListBaseFrame._highlightSelectedColor : itemListBaseFrame._themeColor):
                    itemListBaseFrame._themeColor

            border.width: itemListBaseFrame._itemBorderWidth
            border.color: "white"
            radius      : itemListBaseFrame._itemBorderRadius
            antialiasing: true

            //_touchButtonImage.sourceSize.width            : itemFrame.height/2
            _touchButtonMouseArea.anchors.fill: ((_btnType & _CBUTTON_TYPE_ICON_ONLY) === _CBUTTON_TYPE_ICON_ONLY ||
                                                 (_btnType & _CBUTTON_TYPE_MULTISTATE) === _CBUTTON_TYPE_MULTISTATE)
                                                ? _themeRect: itemFrame

            visible                : model._itemVisible // !!!
            enabled                : model._itemEnabled
            _isChecked             : model._itemChecked
            opacity                : enabled ? 1 : 0.5
            _themeColor            : ((_btnType & _CBUTTON_TYPE_ICON_ONLY)  === _CBUTTON_TYPE_ICON_ONLY ||
                                      (_btnType & _CBUTTON_TYPE_TEXT_ONLY)  === _CBUTTON_TYPE_TEXT_ONLY ||
                                      (_btnType & _CBUTTON_TYPE_MULTISTATE) === _CBUTTON_TYPE_MULTISTATE)
                                     ? itemListBaseFrame._themeColor : K3DRC.COLOR.TRANSPARENT
            _themeOpacity          : itemListBaseFrame._themeOpacity

            _listIndex             : model._itemListIndex
            _isItemOperationNeedIndex: model._itemOperationNeedIndex
            _CK3DOPID              : model._itemK3DOpId
            _CK3DSUBOPID           : model._itemK3DSubOpId
            _CK3DITEMID            : model._itemK3DItemId
            _btnType               : model._itemBtnType
            _btnTooltip            : model._itemTooltip
            _btnTooltipPos         : model._itemTooltipPos
            _btnText               : model._itemText
            _btnTextBold           : model._itemTextBold
            _btnTextStyle          : model._itemTextStyle
            _btnTextStyleColor     : model._itemTextStyleColor
            _btnTextFontSource     : model._itemTextFontSource

            _rotationVectorX    : model._itemRotationVectorX
            _rotationVectorY    : model._itemRotationVectorY
            _rotationVectorZ    : model._itemRotationVectorZ
            _rotationAngle    : model._itemRotationAngle

            _tooltipRotationVectorX  : model._itemTooltipRotationVectorX
            _tooltipRotationVectorY  : model._itemTooltipRotationVectorY
            _tooltipRotationVectorZ  : model._itemTooltipRotationVectorZ
            _tooltipRotationAngle  : model._itemTooltipRotationAngle

            _btnTextColor          : model._itemTextColor
            _btnTextPixelSizeScale : model._itemTextPixelSizeScale
            _btnTextPosition       : model._itemTextPosition
            _btnTextRotation       : model._itemTextRotation
            _btnTextVerticalAlignment   : model._itemTextVerticalAlignment
            _btnTextHorizontalAlignment : model._itemTextHorizontalAlignment
            //_iconBaseSource        : itemFrame._btnNormalSource
            _isHoverChangeImage    : _isChecked ? false             : model._itemHoverChangeImage
            _btnNormalSource       : _isChecked ? _btnCheckedSource : model._itemNormalSource
            _btnHoveredSource      : _isChecked ? _btnCheckedSource : model._itemHoveredSource
            _btnPressedSource      : _isChecked ? _btnCheckedSource : model._itemPressedSource
            _btnCheckedSource      : model._itemCheckedSource

            on_IsCheckedChanged: {
                _iconBaseSource = _isChecked ? model._itemCheckedSource:
                                               model._itemNormalSource;
            }

            _itemFunctionBtnSourceNormal: model._itemFunctionBtnSourceNormal
            _itemFunctionBtnSourceHover : model._itemFunctionBtnSourceHover
            // ----------------------------------------------------
            // Extended
            _isExtendedType        : false
            _extendingState        : K3DRC.EXTEND_DIRECTION.UPWARD
            _shownExtended         : false
            _extListModel          : model._itemExtListModel
            _extListModelType      : model._itemExtListModelType

            //_showExtendedMark     : true
            // ----------------------------------------------------
            // Functionality
            //_showFunctionalMark    : false
            _CFUNCTIONAL_MARK_SOURCE : model._itemFunctionalMarkSrc
            _itemState               : model._itemState

            onClicked: {
                if(itemFrame.enabled) {
                    // [1] - UPDATE CURRENT INDEX
                    //
                    //console.log('EXTLIST-CLICKED:', itemListBaseFrame.currentIndex, itemFrame._listIndex);
                    if(itemListBaseFrame.currentIndex !== itemFrame._listIndex) {
                        itemListBaseFrame.currentIndex = itemFrame._listIndex;

                        itemListBaseFrame.itemSelectionChanged(itemListBaseFrame._parentItemIndex, itemListBaseFrame.currentIndex);
                    }

                    // Reset itemListBaseFrame's _oldMouseInItemIndex. _newMouseInItemIndex:
                    itemListBaseFrame._oldMouseInItemIndex = itemListBaseFrame._newMouseInItemIndex
                                                       = -1;

                    // [2] - SWITCH ITEM STATE FOR MULTISTATE ITEM
                    //
                    if(K3DUTIL.FLAG.test(itemFrame._btnType, _CBUTTON_TYPE_MULTISTATE)) {
                        //print("Item State", K3DUTIL.FLAG.test(k3DItem._btnType, k3DItem._CBUTTON_TYPE_MULTISTATE));
                        // Switch _itemState
                        var itemState = itemFrame._itemState === K3DRC.ITEM_STATE.NORMAL ? K3DRC.ITEM_STATE.FUNCTIONAL :
                                                                 K3DRC.ITEM_STATE.NORMAL;

                        itemListBaseFrame._itemListModel.setProperty(itemFrame._listIndex, "_itemState", itemState);
                    }

                    // [3] - RUN K3DOP IF ABNORMAL (ELSE, RUN IN K3DBUTTON or K3DITEM EARLIER)
                    print('LISTBASE CLICKED', itemFrame._CK3DITEMID);
                    if(K3DUTIL.FLAG.test(itemFrame._btnType, _CBUTTON_TYPE_ABNORMAL)) {
                        if(itemFrame._CK3DOPID > -1) {
                            MAINGB.k3dRunOp(itemFrame._CK3DOPID);
                        }
                    }

                    // ------------------------------------------------------------------------
                    // EMIT SIGNAL [itemIndex]
                    itemListBaseFrame.itemClicked(itemFrame._listIndex);
                }
            }

            onMouseIn: {
                if( false === K3DUTIL.FLAG.test(itemFrame._btnType, itemFrame._CBUTTON_TYPE_ICON_ONLY)) {
                    itemFrame.border.width = 2;
                    itemFrame.border.color = itemListBaseFrame._hoverBorderColor;

                    if(!itemListBaseFrame._isHighlightCurrent) {
                        itemFrame.color              = itemListBaseFrame._hoverColor;
                        itemFrame._btnTextColor      = itemListBaseFrame._hoverTextColor;
                        //itemFrame._btnTextPixelSizeScale  = 1.3;
                    }
                }

                // Entering each row item => Whole frame: _containsMouse true
                //console.log('Ext List Item IN', itemFrame._listIndex);
                itemListBaseFrame._containsMouse = true;
                itemListBaseFrame._newMouseInItemIndex = itemFrame._listIndex;
                if(itemListBaseFrame._oldMouseInItemIndex == -1)
                    itemListBaseFrame._oldMouseInItemIndex = itemListBaseFrame._newMouseInItemIndex;

                delayTimer.stop();

                // ------------------------------------------------------------------------
                // EMIT SIGNAL itemHovered(itemIndex)
                itemListBaseFrame.itemHovered(itemFrame._listIndex);
            }

            onMouseOut: {
                itemFrame.border.width = 0;
                if(!itemListBaseFrame._isHighlightCurrent) {
                    itemFrame.color         = K3DRC.COLOR.TRANSPARENT;
                    itemFrame._btnTextColor = "white";
                }
                itemListBaseFrame._containsMouse       = false;
                itemListBaseFrame._oldMouseInItemIndex = itemListBaseFrame._newMouseInItemIndex;
                delayTimer.start();

                /*
                // To differentiate in this MouseOut Event, the current mouse position
                // is in an item or not:
                // IF [MOUSE IN NEW ITEM -> MOUSE OUT OLD ITEM]
                if(itemListBaseFrame._oldMouseInItemIndex !== itemListBaseFrame._newMouseInItemIndex) { // Having just mouse in new item.
                    // Just update: Old->New
                    console.log('Ext List Item IN->OUT:Old', itemListBaseFrame._oldMouseInItemIndex);
                    itemListBaseFrame._oldMouseInItemIndex = itemListBaseFrame._newMouseInItemIndex;
                    if(itemListBaseFrame._newMouseInItemIndex === itemListBaseFrame._count-1)
                        delayTimer.start();
                }

                // IF [MOUSE OUT OLD ITEM -> MOUSE IN NEW ITEM]
                else { // 1- Move to another item 2- Not moving in any item but still around an item 3- Moving out of the list totally.
                    console.log('Ext List Item OUT->IN:Old', itemListBaseFrame._oldMouseInItemIndex);
                    itemListBaseFrame._containsMouse = false;
                    //itemListBaseFrame._containsMouse will then be updated again if Mouse In later.

                    delayTimer.start();
                }
                */

                /*
                var m = false;
                for(var i = 0; i < itemListView.count; i++) {
                   if(itemListView.contentItem.children[i]._containsMouse) {
                       m = true;
                       console.log('Item has mouse:',i);
                       break;
                   }
                }
                itemListBaseFrame._itemContainsMouse = m;
                console.log('Item out:',itemListBaseFrame._itemContainsMouse);
                */
            }

            // Item Sub Function Button ---------------------------------
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            K3DButton { // control.pressed
                id: itemFunctionBtn
                //z:65535
                // _CK3DOPID: -1
                anchors.top         : itemFrame.top
                anchors.topMargin   : 7
                anchors.right       : itemFrame.right
                anchors.rightMargin : 7
                //anchors.leftMargin: 5

                width               : (_btnNormalSource !== K3DRC.TEXT.EMPTY) ? (itemFrame.height / 3.5) : 0
                height              : width

                _isHoverChangeImage : true
                _btnNormalSource    : model._itemFunctionBtnSourceNormal
                _btnHoveredSource   : model._itemFunctionBtnSourceHover
                _btnPressedSource   : model._itemFunctionBtnSourceNormal

                visible             : itemFunctionBtn._touchButtonMouseArea.containsMouse

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
                        itemListBaseFrame.itemFunctionButtonClicked(model._itemListIndex);

                        //  Here we can also use:
                        //  - Using K3D_OP_ID: static connection, best used for statically connecting to C++ gbK3DQMLAdapter's methods.
                        //  - Using signal: dynamic connection, used both for connecting to C++ gbK3DQMLAdapter's methods and QML methods.
                    }
                }
            } // End Item Sub Function Button

            /*
            ListView.onAdd: SequentialAnimation {
                            PropertyAction { target: delegateItem; property: "height"; value: 0 }
                            NumberAnimation { target: delegateItem; property: "height"; to: 55; duration: 250; easing.type: Easing.InOutQuad }
                        }

            ListView.onRemove: SequentialAnimation {
                PropertyAction { target: delegateItem; property: "ListView.delayRemove"; value: true }
                NumberAnimation { target: delegateItem; property: "height"; to: 0; duration: 250; easing.type: Easing.InOutQuad }

                // Make sure delayRemove is set back to false so that the item can be destroyed
                PropertyAction { target: delegateItem; property: "ListView.delayRemove"; value: false }
            }
            */


        } // End itemFrame
    } // End List Delegate

    // == Functions/Methods ===================================================================================================================
    //

    // ADD LIST ITEM (K3DItem[QML] / K3DQMLItemInfo[C++])
    function addItem() { // arg[argNo-1] : isAppend, arg[argNo-2]: item - By default: _CK3DOPID:-1
//        itemListBaseFrame._listWidth = 0;
//        print("ITEM LIST WIDTH ZERO", itemListBaseFrame._listWidth);
        var argNo = arguments.length;
        if(argNo <= 1) {
            //console.log('Add Item: No Arg');
            return;
        }

        var isAppend = arguments[argNo-1]; // Append[true] | Prepend[false]
        var item  = arguments[argNo - 2]; // !!! ALWAYS THE SECOND LAST ARG - [QML]: K3DItem / [CPP]: K3DQMLItemInfo
        var i;
        var modelItem;

        // [argNo == 2] : MAIN LIST {_itemListModel}
        if(argNo === 2) {
            // [CPP]
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
                if(isAppend) {
                    _itemListModelCPP.push(item);
                }
                else {
                    _itemListModelCPP.splice(0,0,item);
                    //print("_itemListModelCPP 2 : ", _itemListModelCPP[0].text);
                }
            } // END add [CPP] Item

            // [QML]
            else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML) {
                modelItem = {
                            "_itemListIndex"         : item._listIndex,     // By default as -1
                            "_itemOperationNeedIndex": item._isItemOperationNeedIndex, // By default as false
                            "_itemK3DOpId"           : item._CK3DOPID,      // By default as -1
                            "_itemK3DSubOpId"        : item._CK3DSUBOPID,   // By default as -1
                            "_itemK3DItemId"         : item._CK3DITEMID,    // By default as -1
                            "_itemVisible"           : item.visible,        // By default as true
                            "_itemEnabled"           : item.enabled,        // By default as true
                            "_itemChecked"           : item._isChecked,     // By default as false
                            "_itemBtnType"           : item._btnType,
                            "_itemTooltip"           : item._btnTooltip,
                            "_itemTooltipPos"        : item._btnTooltipPos,
                            "_itemText"              : item._btnText,
                            "_itemTextColor"         : item._btnTextColor,
                            "_itemTextBold"          : item._btnTextBold,
                            "_itemTextStyle"         : item._btnTextStyle,
                            "_itemTextStyleColor"    : item._btnTextStyleColor,
                            "_itemTextFontSource"    : item._btnTextFontSource,
                            "_itemTextPixelSizeScale": item._btnTextPixelSizeScale,
                            "_itemTextPosition"      : item._btnTextPosition,
                            "_itemTextRotation"      : item._btnTextRotation,
                            "_itemTextVerticalAlignment"   : item._btnTextVerticalAlignment,
                            "_itemTextHorizontalAlignment" : item._btnTextHorizontalAlignment,

                            "_itemRotationVectorX"     : item._rotationVectorX,
                            "_itemRotationVectorY"     : item._rotationVectorY,
                            "_itemRotationVectorZ"     : item._rotationVectorZ,
                            "_itemRotationAngle"       : item._rotationAngle,

                            "_itemTooltipRotationVectorX": item._tooltipRotationVectorX,
                            "_itemTooltipRotationVectorY": item._tooltipRotationVectorY,
                            "_itemTooltipRotationVectorZ": item._tooltipRotationVectorZ,
                            "_itemTooltipRotationAngle"  : item._tooltipRotationAngle,

                            "_itemHoverChangeImage"  : item._isHoverChangeImage,
                            //"_itemBaseSource"        : item._iconBaseSource,
                            "_itemNormalSource"      : item._btnNormalSource,
                            "_itemHoveredSource"     : item._btnHoveredSource,
                            "_itemPressedSource"     : item._btnPressedSource,
                            "_itemCheckedSource"     : item._btnCheckedSource,
                            "_itemFunctionBtnSourceNormal" : item._itemFunctionBtnSourceNormal,
                            "_itemFunctionBtnSourceHover"  : item._itemFunctionBtnSourceHover,
                            // ------------------------------------------------------------------------------
                            // Extended
                            "_itemIsExtended"        : item._isExtendedType,
                            "_itemExtendedDirection" : item._extendingState,
                            "_itemShownExtended"     : item._shownExtended,
                            "_itemForceShownExtended": item._isForceShownExtended,

                            "_itemExtListModel"      : item._extListModel,
                            "_itemExtListModelType"  : item._extListModelType,
                            "_itemExtendedList"      : [],

                            // ------------------------------------------------------------------------------
                            // Functional
                            "_itemState"             : item._itemState,
                            "_itemFunctionalMarkSrc" : item._CFUNCTIONAL_MARK_SOURCE
                };

                //console.log('APPEND:',isAppend, 'Model Item:', modelItem._itemText);
                if(isAppend) {
                    modelItem._itemListIndex = itemListBaseFrame._count;
                    _itemListModel.append(modelItem);
                }
                else {
                    //print(" _itemListModelCPP 1 : ", modelItem);
                    for(i = 0; i < itemListBaseFrame._count; i++) {
                        _itemListModel.setProperty(i, "_itemListIndex", _itemListModel.get(i)._itemListIndex + 1);
                    }

                    modelItem._itemListIndex = 0;
                    _itemListModel.insert(0, modelItem);
                }
            } // END add [QML] Item

            // ---------------------------------------------------------------------------------------------------
            //print ('LIST BASE FRAME ITEM HEIGHT:', itemListBaseFrame._cITEM_HEIGHT, itemListBaseFrame._itemsEnabledCount);
            // [SIGNAL] modelChanged()
            itemListBaseFrame.modelChanged();
            // ---------------------------------------------------------------------------------------------------
            itemListBaseFrame.updateWidthHeight();
//            if(itemListBaseFrame._isHorizontal)
//                itemListBaseFrame.width  = itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_WIDTH  + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
//            else {
//                //print('NEW ADDED:', itemListBaseFrame._itemsEnabledCount);
//                itemListBaseFrame.height = itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_HEIGHT + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
//            }

            return;
        }

        // [argNo > 2] : EXT LIST {itemNode._itemExtendedList} - addItem(parentItemIndex, itemListLoader.item, isAppend);
        else {
            //print("CASE ArgNo > 2");
            var itemNode = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP[parseInt(arguments[0])] :
                                                                        _itemListModel.get(parseInt(arguments[0])); // _itemListModel.get(parentIndex);

            if(itemNode === undefined) {
                print("Error 1 Add Item - Given item node[",arguments[0],"] does not exist !");
                return;
            }

            // -------------------------------------------------------------------------------
            // IDENTIFY itemNode
            //
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                for(i = 1; i < argNo - 2; ++i) { // The last arg is the Item itself
                    if(itemNode.extendedList.at(parseInt(arguments[i])) === null) {
                        print("Error 2 Add Item CPP - Given item node does not exist !", arguments[i])
                        return;
                    }
                    itemNode = itemNode.extendedList.at(parseInt(arguments[i]));
                }
            }

            else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML){
                for(i = 1; i < argNo - 2; ++i) { // The last arg is the Item itself
                    if(itemNode._itemExtendedList.get(parseInt(arguments[i])) === undefined) {
                        print("Error 2 Add Item QML - Given item node does not exist !", arguments[i])
                        return;
                    }
                    itemNode = itemNode._itemExtendedList.get(parseInt(arguments[i]));
                }
            }

            // -------------------------------------------------------------------------------
            // INSERT item into itemNode's extended list
            //
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                if(isAppend) {
                    itemNode.extendedList.push(item);
                }
                else {
                    itemNode.extendedList.push(item);
                    for(i = itemNode.extendedList.count-1; i >=1 ; i--) {
                        //itemNode.extendedList.at(i) = itemNode.extendedList.at(i-1);
                    }
                    //itemNode.extendedList.at(0) = item;
                }
            } // END add extended [QML] Item

            else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML){
                modelItem = {
                            "_itemListIndex"         : item._listIndex,     // By default as -1
                            "_itemOperationNeedIndex": item._isItemOperationNeedIndex, // By default as false
                            "_itemK3DOpId"           : item._CK3DOPID,      // By default as -1
                            "_itemK3DSubOpId"        : item._CK3DSUBOPID,   // By default as -1
                            "_itemK3DItemId"         : item._CK3DITEMID,    // By default as -1
                            "_itemVisible"           : item.visible,        // By default as true
                            "_itemEnabled"           : item.enabled,        // By default as true
                            "_itemChecked"           : item._isChecked,     // By default as false
                            "_itemBtnType"           : item._btnType,
                            "_itemTooltip"           : item._btnTooltip,
                            "_itemTooltipPos"        : item._btnTooltipPos,
                            "_itemText"              : item._btnText,
                            "_itemTextBold"          : item._btnTextBold,
                            "_itemTextStyle"         : item._btnTextStyle,
                            "_itemTextStyleColor"    : item._btnTextStyleColor,
                            "_itemTextFontSource"    : item._btnTextFontSource,
                            "_itemTextColor"         : item._btnTextColor,
                            "_itemTextPixelSizeScale": item._btnTextPixelSizeScale,
                            "_itemTextPosition"      : item._btnTextPosition,
                            "_itemTextRotation"      : item._btnTextRotation,
                            "_itemTextVerticalAlignment"   : item._btnTextVerticalAlignment,
                            "_itemTextHorizontalAlignment" : item._btnTextHorizontalAlignment,

                            "_itemRotationVectorX": item._rotationVectorX,
                            "_itemRotationVectorY": item._rotationVectorY,
                            "_itemRotationVectorZ": item._rotationVectorZ,
                            "_itemRotationAngle": item._rotationAngle,

                            "_itemTooltipRotationVectorX": item._tooltipRotationVectorX,
                            "_itemTooltipRotationVectorY": item._tooltipRotationVectorY,
                            "_itemTooltipRotationVectorZ": item._tooltipRotationVectorZ,
                            "_itemTooltipRotationAngle": item._tooltipRotationAngle,

                            "_itemHoverChangeImage"  : item._isHoverChangeImage,
                            //"_itemBaseSource"        : item._iconBaseSource,
                            "_itemNormalSource"      : item._btnNormalSource,
                            "_itemHoveredSource"     : item._btnHoveredSource,
                            "_itemPressedSource"     : item._btnPressedSource,
                            "_itemCheckedSource"     : item._btnCheckedSource,
                            "_itemFunctionBtnSourceNormal" : item._itemFunctionBtnSourceNormal,
                            "_itemFunctionBtnSourceHover"  : item._itemFunctionBtnSourceHover,
                            // ----------------------------------------------------
                            // Extended
                            "_itemIsExtended"        : item._isExtendedType,
                            "_itemExtendedDirection" : item._extendingState,
                            "_itemShownExtended"     : item._shownExtended,
                            "_itemForceShownExtended": item._isForceShownExtended,

                            "_itemExtListModel"      : item._extListModel,
                            "_itemExtListModelType"  : item._extListModelType,
                            "_itemExtendedList"      : [],

                            // ------------------------------------------------------------------------------
                            // Functional
                            "_itemState"             : item._itemState,
                            "_itemFunctionalMarkSrc" : item._CFUNCTIONAL_MARK_SOURCE
                };

                //console.log('APPEND:',isAppend, 'Model Item:', modelItem._itemText);
                modelItem._itemBtnType               = item._CBUTTON_TYPE_TEXT_ONLY;
                modelItem._itemOperationNeedIndex    = true;

                if(isAppend) {
                    modelItem._itemListIndex         = itemNode._itemExtendedList.count;
                    //console.log('arguments[0]', arguments[0], modelItem._itemListIndex);
                    itemNode._itemExtendedList.append(modelItem);
                }
                else {
                    for(i = 0; i < itemNode._itemExtendedList.count; i++) {
                        itemNode._itemExtendedList.setProperty(i, "_itemListIndex", itemNode._itemExtendedList.get(i)._itemListIndex + 1);
                    }

                    modelItem._itemListIndex = 0;
                    itemNode._itemExtendedList.insert(0, modelItem);
                }
            } // END add extended [QML] Item

            // ---------------------------------------------------------------------------------------------------
            // extItemListFrame: height -> _itemExtendedList.count * (itemFrame.height)
        }

        // =======================================================================================================
        // [SIGNAL] modelChanged()
        itemListBaseFrame.modelChanged();
    } // End addItem()

    // ADD LIST ITEM (K3DItem[QML] / K3DQMLItemInfo[C++])
    function insertItem() { // arg[argNo-1] : item, arg[0]: itemIndex. arg[1]: itemSubIndex - By default: _CK3DOPID:-1
//        itemListBaseFrame._listWidth = 0;
//        print("ITEM LIST WIDTH ZERO", itemListBaseFrame._listWidth);
        var argNo = arguments.length;
        if(argNo <= 1) {
            //console.log('Add Item: No Arg');
            return;
        }

        var itemIndex = arguments[0]; // Append[true] | Prepend[false]
        var item  = arguments[argNo - 1]; // !!! ALWAYS THE LAST ARG - [QML]: K3DItem / [CPP]: K3DQMLItemInfo
        var i;
        var modelItem;

        // [argNo == 2] : MAIN LIST {_itemListModel}
        if(argNo === 2) {
            // [CPP]
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
                _itemListModelCPP.splice(itemIndex, 0, item);
                //print("_itemListModelCPP 2 : ", _itemListModelCPP[0].text);
            } // END add [CPP] Item

            // [QML]
            else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML) {
                modelItem = {
                            "_itemListIndex"         : item._listIndex,     // By default as -1
                            "_itemOperationNeedIndex": item._isItemOperationNeedIndex, // By default as false
                            "_itemK3DOpId"           : item._CK3DOPID,      // By default as -1
                            "_itemK3DSubOpId"        : item._CK3DSUBOPID,   // By default as -1
                            "_itemK3DItemId"         : item._CK3DITEMID,    // By default as -1
                            "_itemVisible"           : item.visible,        // By default as true
                            "_itemEnabled"           : item.enabled,        // By default as true
                            "_itemChecked"           : item._isChecked,     // By default as false
                            "_itemBtnType"           : item._btnType,
                            "_itemTooltip"           : item._btnTooltip,
                            "_itemTooltipPos"        : item._btnTooltipPos,
                            "_itemText"              : item._btnText,
                            "_itemTextColor"         : item._btnTextColor,
                            "_itemTextBold"          : item._btnTextBold,
                            "_itemTextStyle"         : item._btnTextStyle,
                            "_itemTextStyleColor"    : item._btnTextStyleColor,
                            "_itemTextFontSource"    : item._btnTextFontSource,
                            "_itemTextPixelSizeScale": item._btnTextPixelSizeScale,
                            "_itemTextPosition"      : item._btnTextPosition,
                            "_itemTextRotation"      : item._btnTextRotation,
                            "_itemTextVerticalAlignment"   : item._btnTextVerticalAlignment,
                            "_itemTextHorizontalAlignment" : item._btnTextHorizontalAlignment,

                            "_itemRotationVectorX": item._rotationVectorX,
                            "_itemRotationVectorY": item._rotationVectorY,
                            "_itemRotationVectorZ": item._rotationVectorZ,
                            "_itemRotationAngle": item._rotationAngle,

                            "_itemTooltipRotationVectorX": item._tooltipRotationVectorX,
                            "_itemTooltipRotationVectorY": item._tooltipRotationVectorY,
                            "_itemTooltipRotationVectorZ": item._tooltipRotationVectorZ,
                            "_itemTooltipRotationAngle": item._tooltipRotationAngle,

                            "_itemHoverChangeImage"  : item._isHoverChangeImage,
                            //"_itemBaseSource"        : item._iconBaseSource,
                            "_itemNormalSource"      : item._btnNormalSource,
                            "_itemHoveredSource"     : item._btnHoveredSource,
                            "_itemPressedSource"     : item._btnPressedSource,
                            "_itemCheckedSource"     : item._btnCheckedSource,
                            "_itemFunctionBtnSourceNormal" : item._itemFunctionBtnSourceNormal,
                            "_itemFunctionBtnSourceHover"  : item._itemFunctionBtnSourceHover,
                            // ------------------------------------------------------------------------------
                            // Extended
                            "_itemIsExtended"        : item._isExtendedType,
                            "_itemExtendedDirection" : item._extendingState,
                            "_itemShownExtended"     : item._shownExtended,
                            "_itemForceShownExtended": item._isForceShownExtended,

                            "_itemExtListModel"      : item._extListModel,
                            "_itemExtListModelType"  : item._extListModelType,
                            "_itemExtendedList"      : [],

                            // ------------------------------------------------------------------------------
                            // Functional
                            "_itemState"             : item._itemState,
                            "_itemFunctionalMarkSrc" : item._CFUNCTIONAL_MARK_SOURCE
                };

                //console.log('APPEND:',isAppend, 'Model Item:', modelItem._itemText);
                for(i = itemIndex + 1; i < itemListBaseFrame._count; i++) {
                    _itemListModel.setProperty(i, "_itemListIndex", _itemListModel.get(i)._itemListIndex + 1);
                }

                modelItem._itemListIndex = itemIndex + 1;
                _itemListModel.insert(itemIndex, modelItem);
            } // END add [QML] Item

            // ---------------------------------------------------------------------------------------------------
            //print ('LIST BASE FRAME ITEM HEIGHT:', itemListBaseFrame._cITEM_HEIGHT, itemListBaseFrame._itemsEnabledCount);
            // [SIGNAL] modelChanged()
            itemListBaseFrame.modelChanged(); // UPATE COUNT
            // ---------------------------------------------------------------------------------------------------
            itemListBaseFrame.updateWidthHeight();
//            if(itemListBaseFrame._isHorizontal)
//                itemListBaseFrame.width  = itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_WIDTH  + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
//            else {
//                //print('NEW ADDED:', itemListBaseFrame._count);
//                itemListBaseFrame.height = itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_HEIGHT + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
//            }

            return;
        }

        // [argNo > 2] : EXT LIST {itemNode._itemExtendedList} - addItem(parentItemIndex, itemSubIndex, item);
        else {
            //print("CASE ArgNo > 2");
            var itemNode = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP[parseInt(arguments[0])] :
                                                                        _itemListModel.get(parseInt(arguments[0])); // _itemListModel.get(parentItemIndex);

            var itemSubIndex = arguments[argNo-2];
            if(itemNode === undefined) {
                print("Error 1 Insert Item - Given item node[",arguments[0],"] does not exist !");
                return;
            }

            // -------------------------------------------------------------------------------
            // IDENTIFY itemNode
            //
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                for(i = 1; i < argNo - 2; ++i) { // The last arg is the Item itself
                    if(itemNode.extendedList.at(parseInt(arguments[i])) === null) {
                        print("Error 2 Insert Item CPP - Given item node does not exist !", arguments[i])
                        return;
                    }
                    itemNode = itemNode.extendedList.at(parseInt(arguments[i]));
                }
            }

            else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML){
                for(i = 1; i < argNo - 2; ++i) { // The last arg is the Item itself
                    if(itemNode._itemExtendedList.get(parseInt(arguments[i])) === undefined) {
                        print("Error 2 Insert Item QML - Given item node does not exist !", arguments[i])
                        return;
                    }
                    itemNode = itemNode._itemExtendedList.get(parseInt(arguments[i]));
                }
            }

            // -------------------------------------------------------------------------------
            // INSERT item into itemNode's extended list
            //
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                itemNode.extendedList.insert(itemSubIndex, item);
            } // END add extended [QML] Item

            else if(_modelType === K3DRC.LIST_MODEL_TYPE.QML){
                modelItem = {
                            "_itemListIndex"         : item._listIndex,     // By default as -1
                            "_itemOperationNeedIndex": item._isItemOperationNeedIndex, // By default as false
                            "_itemK3DOpId"           : item._CK3DOPID,      // By default as -1
                            "_itemK3DSubOpId"        : item._CK3DSUBOPID,   // By default as -1
                            "_itemK3DItemId"         : item._CK3DITEMID,    // By default as -1
                            "_itemVisible"           : item.visible,        // By default as true
                            "_itemEnabled"           : item.enabled,        // By default as true
                            "_itemChecked"           : item._isChecked,     // By default as false
                            "_itemBtnType"           : item._btnType,
                            "_itemTooltip"           : item._btnTooltip,
                            "_itemTooltipPos"        : item._btnTooltipPos,
                            "_itemText"              : item._btnText,
                            "_itemTextBold"          : item._btnTextBold,
                            "_itemTextStyle"         : item._btnTextStyle,
                            "_itemTextStyleColor"    : item._btnTextStyleColor,
                            "_itemTextFontSource"    : item._btnTextFontSource,
                            "_itemTextColor"         : item._btnTextColor,
                            "_itemTextPixelSizeScale": item._btnTextPixelSizeScale,
                            "_itemTextPosition"      : item._btnTextPosition,
                            "_itemTextRotation"      : item._btnTextRotation,
                            "_itemTextVerticalAlignment"   : item._btnTextVerticalAlignment,
                            "_itemTextHorizontalAlignment" : item._btnTextHorizontalAlignment,

                            "_itemRotationVectorX": item._rotationVectorX,
                            "_itemRotationVectorY": item._rotationVectorY,
                            "_itemRotationVectorZ": item._rotationVectorZ,
                            "_itemRotationAngle": item._rotationAngle,

                            "_itemTooltipRotationVectorX": item._tooltipRotationVectorX,
                            "_itemTooltipRotationVectorY": item._tooltipRotationVectorY,
                            "_itemTooltipRotationVectorZ": item._tooltipRotationVectorZ,
                            "_itemTooltipRotationAngle": item._tooltipRotationAngle,

                            "_itemHoverChangeImage"  : item._isHoverChangeImage,
                            //"_itemBaseSource"        : item._iconBaseSource,
                            "_itemNormalSource"      : item._btnNormalSource,
                            "_itemHoveredSource"     : item._btnHoveredSource,
                            "_itemPressedSource"     : item._btnPressedSource,
                            "_itemCheckedSource"     : item._btnCheckedSource,
                            "_itemFunctionBtnSourceNormal" : item._itemFunctionBtnSourceNormal,
                            "_itemFunctionBtnSourceHover"  : item._itemFunctionBtnSourceHover,
                            // ----------------------------------------------------
                            // Extended
                            "_itemIsExtended"        : item._isExtendedType,
                            "_itemExtendedDirection" : item._extendingState,
                            "_itemShownExtended"     : item._shownExtended,
                            "_itemForceShownExtended": item._isForceShownExtended,

                            "_itemExtListModel"      : item._extListModel,
                            "_itemExtListModelType"  : item._extListModelType,
                            "_itemExtendedList"      : [],

                            // ------------------------------------------------------------------------------
                            // Functional
                            "_itemState"             : item._itemState,
                            "_itemFunctionalMarkSrc" : item._CFUNCTIONAL_MARK_SOURCE
                };

                //console.log('APPEND:',isAppend, 'Model Item:', modelItem._itemText);
                modelItem._itemBtnType               = item._CBUTTON_TYPE_TEXT_ONLY;
                modelItem._itemOperationNeedIndex    = true;

                for(i = itemSubIndex + 1; i < itemNode._itemExtendedList.count; i++) {
                    itemNode._itemExtendedList.setProperty(i, "_itemListIndex", itemNode._itemExtendedList.get(i)._itemListIndex + 1);
                }

                modelItem._itemListIndex = itemSubIndex + 1;
                itemNode._itemExtendedList.insert(itemSubIndex, modelItem);
            } // END add extended [QML] Item

            // ---------------------------------------------------------------------------------------------------
            // extItemListFrame: height -> _itemExtendedList.count * (itemFrame.height)
        }

        // =======================================================================================================
        // [SIGNAL] modelChanged()
        itemListBaseFrame.modelChanged();
    } // End insertItem()

    function remAllItems() {
        var argNo = arguments.length;
        var i = 0;
        if(argNo === 0) {
            //console.log('Rem All Items: No Arg - Remove ALL');
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                _itemListModelCPP.splice(0, itemListBaseFrame._count);
            }
            else{
                for(i = itemListBaseFrame._count-1; i >= 0; i--) {
                    _itemListModel.remove(i);
                }
            }

            // ---------------------------------------------------------------------------------------------------
            // [SIGNAL] modelChanged()
            itemListBaseFrame.modelChanged();
            // ---------------------------------------------------------------------------------------------------
            itemListBaseFrame.updateWidthHeight();

//            if(itemListBaseFrame._isHorizontal)
//                itemListBaseFrame.width  =  itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_WIDTH + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
//            else
//                itemListBaseFrame.height =  itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_HEIGHT + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;

            return;
        }
        else {
            var itemNode = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP[parseInt(arguments[0])] :
                                                                        _itemListModel.get(parseInt(arguments[0])); // _itemListModel.get(parentIndex);

            if(itemNode === undefined) {
                print("Error 1 RemAllItems - Given item node[",arguments[0],"] does not exist !");
                return;
            }

            // ---------------------------------------------------------------------------------------------------
            if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                itemNode.extendedList.clear();
            }
            if(_modelType === K3DRC.LIST_MODEL_TYPE.QML){
                for(i = itemNode._itemExtendedList.count -1; i >=0; i--) {
                     itemNode._itemExtendedList.remove(i);
                }
            }

            // ---------------------------------------------------------------------------------------------------
            // extItemListFrame: height -> _itemExtendedList.count * (itemFrame.height)
        }

        // =======================================================================================================
        // [SIGNAL] modelChanged()
        itemListBaseFrame.modelChanged();
    } // End remAllItems()

    // Remove List Item : remItem(parentItemIndex, extItemIndex);
    function remItem() {
        var argNo = arguments.length;
        print("Arg No = ", argNo, arguments); // =1
        if(argNo === 0) {
            //console.log('Add Item: No Arg');
            return;
        }
        // ---------------------------------------------------------------------------------------------------

        var itemIndex  = arguments[argNo - 1]; // !!! ALWAYS THE LAST ARG ==4
        print("ITEM INDEX : ", itemIndex);
        var i = 0;
        // [argNo == 1]: MAIN LIST {_itemListModel}
        if(argNo === 1) {
            //print("Arg No 1: ",itemListBaseFrame._count,_itemListModelCPP, _itemListModel);
            if(itemListBaseFrame._modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                _itemListModelCPP.splice(itemIndex, 1);
                //print(" _itemListModelCPP 3 : ", _itemListModelCPP);
            }
            else{
                for(i = itemIndex +1; i < itemListBaseFrame._count; i++) {
                    _itemListModel.setProperty(i, "_itemListIndex", _itemListModel.get(i)._itemListIndex-1);
                }
                _itemListModel.remove(itemIndex);
            }

            // ---------------------------------------------------------------------------------------------------
            // [SIGNAL] modelChanged()
            itemListBaseFrame.modelChanged();
            // ---------------------------------------------------------------------------------------------------
            itemListBaseFrame.updateWidthHeight();

//            if(itemListBaseFrame._isHorizontal)
//                itemListBaseFrame.width  =  itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_WIDTH + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
//            else
//                itemListBaseFrame.height =  itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_HEIGHT + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;

            return;
        }
        // [argNo > 1]: EXT LIST {itemNode._itemExtendedList} -- remItem(parentItemIndex, itemIndex)
        else {
            print("Arg No > 1");
            var itemNode = (_modelType === K3DRC.LIST_MODEL_TYPE.CPP) ? _itemListModelCPP[parseInt(arguments[0])] :
                                                                        _itemListModel.get(parseInt(arguments[0])); // _itemListModel.get(parentIndex);

            if(itemNode === undefined) {
                print("Error 1 Rem Item - Given item node[",arguments[0],"] does not exist !");
                return;
            }

            // ---------------------------------------------------------------------------------------------------
            // IDENTIFY itemNode
            //
            if(itemListBaseFrame._modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                for(i = 1; i < argNo - 1; ++i) { // The last arg is the Item iteslf
                    if(itemNode.extendedList.at(parseInt(arguments[i])) === null) {
                        //console.log("Error 2 - Given item node does not exist !", arguments[i])
                        return;
                    }
                    itemNode = itemNode.extendedList.at(parseInt(arguments[i]));
                    print("ITEM NODE 2: ", itemNode);
                }
            }

            else if(itemListBaseFrame._modelType === K3DRC.LIST_MODEL_TYPE.QML) {
                for(i = 1; i < argNo - 1; ++i) { // The last arg is the Item iteslf
                    if(itemNode._itemExtendedList.get(parseInt(arguments[i])) === undefined) {
                        //console.log("Error 2 - Given item node does not exist !", arguments[i])
                        return;
                    }
                    itemNode = itemNode._itemExtendedList.get(parseInt(arguments[i]));
                    print("ITEM NODE 2: ", itemNode);
                }
            }

            print("ITEM NODE : ", itemNode);

            // ---------------------------------------------------------------------------------------------------
            // REMOVE item at itemIndex from itemNode's extended list
            //
            // [CPP]
            if(itemListBaseFrame._modelType === K3DRC.LIST_MODEL_TYPE.CPP){
                for(i = itemIndex; i <= itemNode._itemExtendedList.count-2; i++) {
                    //itemNode.extendedList.at(i) = itemNode.extendedList.at(i+1);
                }
                //itemNode.extendedList.at(itemNode._itemExtendedList.count-1) = null;
            }
            // [QML]
            else {
                // ---------------------------------------------------------------------------------------------------
                for(i = itemIndex + 1; i < itemNode._itemExtendedList.count; i++) {
                    itemNode._itemExtendedList.setProperty(i, "_itemListIndex", itemNode._itemExtendedList.get(i)._itemListIndex-1);
                    //(_itemListModel.get(i)._itemListIndex--;)
                }
                //console.log('Rem Extended Item:', itemNode._itemExtendedList.get(itemIndex)._itemListIndex, itemNode._itemExtendedList.get(itemIndex)._itemText);
                itemNode._itemExtendedList.remove(itemIndex);
            }

            // ---------------------------------------------------------------------------------------------------
            // extItemListFrame: height -> _itemExtendedList.count * (itemFrame.height)
        }

        // =======================================================================================================
        // [SIGNAL] modelChanged()
        itemListBaseFrame.modelChanged();
    } // End remItem()

    Component.onCompleted: {
    }
    function updateWidthHeight(){
        if(itemListBaseFrame._isHorizontal) {
            itemListBaseFrame.width  = itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_WIDTH  + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
            //print("LISTBASE AUTO RESIZE WIDTH:", itemListBaseFrame._itemsEnabledCount, itemListBaseFrame.width);
        }
        else {
            itemListBaseFrame.height = itemListBaseFrame._itemsEnabledCount * itemListBaseFrame._cITEM_HEIGHT + (itemListBaseFrame._itemsEnabledCount-1) * itemListBaseFrame.spacing;
            //print("LISTBASE AUTO RESIZE HEIGHT:", itemListBaseFrame._itemsEnabledCount, itemListBaseFrame.height);
        }
    }

    function getItemId(index) {
        if(index < 0 || index >= _count)
            return K3DQMLAdapter.K3D_ACTION_VOID - 2;

        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            return _itemListModelCPP[index].itemId;
        }
        else {
            return _itemListModel.get(index)._itemK3DItemId;
        }
    }

    function isItemAvailable(itemId) {
        var i;

        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
            for(i = 0; i < itemListBaseFrame._count; i++) {
                if(_itemListModelCPP[i].itemId === itemId)
                    return true;
            }
        }
        else {
            for(i = 0; i < itemListBaseFrame._count; i++) {
                if(_itemListModel.get(i)._itemK3DItemId === itemId)
                    return true;
            }
        }

        return false;
    }

    // Hide List Item
    function setItemVisible(itemId, isVisible) {
        var i;

        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
            for(i = 0; i < itemListBaseFrame._count; i++) {
                if(_itemListModelCPP[i].itemId === itemId) {
                    _itemListModelCPP[i].visible = isVisible;
                    return;
                }
            }
        }
        else {
            for(i = 0; i < itemListBaseFrame._count; i++) {
                if(_itemListModel.get(i)._itemK3DItemId === itemId) {
                     _itemListModel.setProperty(i, "_itemVisible", isVisible);
                    return;
                }
            }
        }
    }

    function getExtItemCount(itemIndex) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            return  _itemListModelCPP[itemIndex].extendedList.count;
        else
            return  _itemListModel.get(itemIndex)._itemExtendedList.count;
    }

    function isItemLimited(itemIndex) {
        return MAINGB.isAppExpressEdition() &&
               MAINGB._K3D_QML_ADAPTER.isActionLimited(getItemId(itemIndex));
    }

    function setMainItemEnabled(itemIndex, isEnabled) {
        if(isEnabled && isItemLimited(itemIndex)) {
            isEnabled = false;
        }

        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            _itemListModelCPP[itemIndex].enabled = isEnabled;
        }
        else {
            _itemListModel.setProperty(itemIndex, "_itemEnabled", isEnabled);
        }
        _itemsEnabledCount = getItemEnabledCount();
    }

    function isMainItemEnabled(itemIndex) {
        var i;
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            return _itemListModelCPP[itemIndex].enabled;
        }
        else {
            return _itemListModel.get(itemIndex)._itemEnabled;
        }
    }

    function setItemEnabled(itemId, isEnabled) {
        var i;
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            for(i = 0; i < _count; i ++) {
                if(_itemListModelCPP[i].itemId === itemId) {
                    _itemListModelCPP[i].enabled = isEnabled;
                    break;
                }
            }
        }
        else {
            for(i = 0; i < _count; i ++) {
                if(_itemListModel.get(i)._itemK3DItemId === itemId) {
                    _itemListModel.setProperty(i, "_itemEnabled", isEnabled);
                    break;
                }
            }
        }
        _itemsEnabledCount = getItemEnabledCount();
    }

    function isItemEnabled(itemId) {
        var i;
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            for(i = 0; i < _count; i ++) {
                if(_itemListModelCPP[i].itemId === itemId) {
                    return _itemListModelCPP[i].enabled;
                }
            }
        }
        else {
            for(i = 0; i < _count; i ++) {
                if(_itemListModel.get(i)._itemK3DItemId === itemId) {
                    return _itemListModel.get(i)._itemEnabled;
                }
            }
        }

        return false;
    }

    function setItemChecked__(index, isChecked) {
        if(index < 0 || index >= _count)
            return;

        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            _itemListModelCPP[index].checked = isChecked;
        }
        else {
            _itemListModel.setProperty(index, "_itemChecked", isChecked);
        }
    }

    function setItemChecked(itemId, isChecked) {
        var i;
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            for(i = 0; i < _count; i ++) {
                if(_itemListModelCPP[i].itemId === itemId) {
                    _itemListModelCPP[i].checked = isChecked;
                    break;
                }
            }
        }
        else {
            for(i = 0; i < _count; i ++) {
                if(_itemListModel.get(i)._itemK3DItemId === itemId) {
                    _itemListModel.setProperty(i, "_itemChecked", isChecked);
                    break;
                }
            }
        }
    }

    function isItemChecked__(index) {
        if(index < 0 || index >= _count)
            return;

        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            return _itemListModelCPP[index].checked;
        }
        else {
            return _itemListModel.get(index)._itemChecked;
        }
    }

    function isItemChecked(itemId) {
        var i;
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            for(i = 0; i < _count; i ++) {
                if(_itemListModelCPP[i].itemId === itemId) {
                    return _itemListModelCPP[i].checked;
                }
            }
        }
        else {
            for(i = 0; i < _count; i ++) {
                if(_itemListModel.get(i)._itemK3DItemId === itemId) {
                    return _itemListModel.get(i)._itemChecked;
                }
            }
        }

        return false;
    }

    function setExtItemEnabled(itemIndex, subItemIndex, isEnabled) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            _itemListModelCPP[itemIndex].extendedList.at(subItemIndex).enabled = isEnabled;
        else
            _itemListModel.get(itemIndex)._itemExtendedList.setProperty(subItemIndex, "_itemEnabled", isEnabled);
        _itemsEnabledCount = getItemEnabledCount();
    }

    function isExtItemEnabled(itemIndex, subItemIndex) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            return _itemListModelCPP[itemIndex].extendedList.at(subItemIndex).enabled;
        else
            return _itemListModel.get(itemIndex)._itemExtendedList.get(subItemIndex)._itemEnabled;
    }

    function getItemText(itemIndex) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            return _itemListModelCPP[itemIndex].text;
        else
            return _itemListModel.get(itemIndex)._itemText;
    }

    function setItemText(itemIndex, newName) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            _itemListModelCPP[itemIndex].text = newName;
        }
        else {
            _itemListModel.setProperty(itemIndex, "_itemText", newName);
            var itemListModel = _itemListModel;
            _itemListModel = null;
            _itemListModel = itemListModel
        }
    }

    function getExtItemText(itemIndex, subItemIndex) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            return _itemListModelCPP[itemIndex].extendedList.at(subItemIndex).text;
        else
            return _itemListModel.get(itemIndex)._itemExtendedList.get(subItemIndex)._itemText;
    }

    function setExtItemText(itemIndex, subItemIndex, newName) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            _itemListModelCPP[itemIndex].extendedList.at(subItemIndex).text = newName;
        else
            _itemListModel.get(itemIndex)._itemExtendedList.setProperty(subItemIndex, "_itemText", newName);
    }

    function setItemTooltip(itemIndex, newTooltip) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            _itemListModelCPP[itemIndex]._itemTooltip = newTooltip;
        else
            _itemListModel.setProperty(itemIndex, "_itemTooltip", newTooltip);
    }

    function getExtItemTooltip (itemId, subItemId){
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP)
            return _itemListModelCPP[itemId].extendedList.at(subItemId).text;
        else
            return _itemListModel.get(itemId)._itemExtendedList.get(subItemId)._itemTooltip;
    }

    function setExtItemProperty(itemIndex, subItemIndex, propName, propValue) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP){
            //_itemListModelCPP[itemIndex]._itemExtendedList.setProperty(subItemIndex, propName, propValue);
        }
        else
            _itemListModel.get(itemIndex)._itemExtendedList.setProperty(subItemIndex, propName, propValue);
    }

    function setItemRotationVector(itemIndex, vector3d) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            //_itemListModelCPP[itemIndex].rotationVector = vector3d;
        }
        else {
            _itemListModel.setProperty(itemIndex, "_itemRotationVectorX", vector3d.x);
            _itemListModel.setProperty(itemIndex, "_itemRotationVectorY", vector3d.y);
            _itemListModel.setProperty(itemIndex, "_itemRotationVectorZ", vector3d.z);
        }
    }

    function setItemRotationAngle(itemIndex, angle) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            //_itemListModelCPP[itemIndex].rotationAngle = angle;
        }
        else {
            _itemListModel.setProperty(itemIndex, "_itemRotationAngle", angle);
        }
    }

    function setItemTooltipRotationVector(itemIndex, vector3d) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            //_itemListModelCPP[itemIndex].tooltipRotationVector = vector3d;
        }
        else {
            _itemListModel.setProperty(itemIndex, "_itemTooltipRotationVectorX", vector3d.x);
            _itemListModel.setProperty(itemIndex, "_itemTooltipRotationVectorY", vector3d.y);
            _itemListModel.setProperty(itemIndex, "_itemTooltipRotationVectorZ", vector3d.z);
        }
    }

    function setItemtooltipRotationAngle(itemIndex, angle) {
        if(_modelType === K3DRC.LIST_MODEL_TYPE.CPP) {
            //_itemListModelCPP[itemIndex].tooltipRotationAngle = angle;
        }
        else {
            _itemListModel.setProperty(itemIndex, "_itemTooltipRotationAngle", angle);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------------
    // -- STATES
    //http://qt-project.org/doc/qt-5/qtquick-statesanimations-animations.html
    //http://qt-project.org/doc/qt-5/qtquick-usecase-animations.html
    //http://qt-project.org/doc/qt-5/qml-qtquick-scriptaction.html
    //http://qt-project.org/doc/qt-4.8/qml-propertyanimation.html#details
    //http://qt-project.org/doc/qt-4.8/qdeclarativeanimation.html
    //property variant _CSTATES : ["shown", "hidden"]
    state                                       : K3DRC.STATE.SHOWN
    states: [
                // -------------------------------------------------------------------------------------
                State {
                    name: K3DRC.STATE.SHOWN

                    PropertyChanges {
                        target: itemListBaseFrame
                        opacity : 1
                    }

                    StateChangeScript {
                        name: "ShownScript"
                        script: {
                        }
                    }
                },
                State {
                    name: K3DRC.STATE.HIDDEN
                    PropertyChanges {
                        target: itemListBaseFrame
                        opacity: 0
                    }
                    StateChangeScript {
                        name: "HiddenScript"
                        script: {
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
                         NumberAnimation   { properties: "opacity"; duration:200}
                         ScriptAction      { scriptName: "ShownScript" }
                     },
                     Transition {
                         //from: K3DRC.STATE.SHOWN
                         to   : K3DRC.STATE.HIDDEN
                         NumberAnimation   { properties: "opacity"; duration:200}
                         ScriptAction      { scriptName: "HiddenScript" }
                     }
                ]
    // End TRANSITIONS
} // End Menu Extended List
