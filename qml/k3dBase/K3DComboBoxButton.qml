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

K3DButton{
    id : thisComboBoxButton
    objectName : K3DRC.TEXT.K3D_MACHINE_BUTTON_OBJECT_NAME
    _stayPut   : true
    property alias _color                     : thisComboBoxButton.color
    property var   _itemList                  : [] // NON-EDITABLE!
    property var   _customItemNameList        : [] // EDITABLE!

    property int   _defaultItem               : -1
    property alias _currentItem               : modelListView.currentIndex //_defaultItem < 0 ? 0 : _defaultItem // Initially!

    property int   _CMAX_TEXT_LENGTH          : 100 // Now we don't need to use it anymore
    property int   _CMAX_ITEM_TEXT_LENGTH     : 20 // Still keep to set outside

    property string _CITEM_UNIT               : ""
    property string _CADD_ITEM_BUTTON_TEXT    : ""

    property alias  _addButtonOpId            : addItemButton._CK3DOPID
    property string _currentEditedItemText

    property real   _addItemButtonTextWidth   : addItemButton._addItemTextWidth
    property real   _dropDownMinWidth               : _addItemButtonTextWidth > 0 ? (_addItemButtonTextWidth * 1.2) : thisComboBoxButton._originalWidth > 0 ? thisComboBoxButton._originalWidth -2 : thisComboBoxButton.width -2 // If keep this binding, dropdownWidth will incorrect
    property real   _dropDownMaxWidth               : _addItemButtonTextWidth > 0 ? (_addItemButtonTextWidth * 2)   : thisComboBoxButton._originalWidth > 0 ? thisComboBoxButton._originalWidth    : thisComboBoxButton.width

    property real   _maxTextWidth                   : Math.abs(thisComboBoxButton.width - _textLeftRightMargin)// > _textRightSpacing ? (thisComboBoxButton.width - _textLeftRightMargin) : thisComboBoxButton.width* 0.6
    property string _normalImageSource              : ""
    property string _hoverImagesource               : ""
    property var    _dropDownItemsImageNormalList   : []
    property var    _dropDownItemsImageHoverList    : []

    property bool   _setBtnImageSourceBySlectedItem : false

    // Define
    _CEXT_MARK_RIGHT_POS_LEFT_MARGIN      : _isExtendedType ? (- _extendedMark.width * 3) : 0
    property real _textRightSpacing       : MAINGB._BASE_FONT_SIZE // 12 // = width of one normal letter // Just now, up to qt 5.5.1 it will be thisComboBoxButton.height / 3
    // set _textLeftRightMargin = 0 to set text fill full button if it can, not be cut.
    property real _textLeftRightMargin    : Math.abs(_imageLeftMargin) + MAINGB._BASE_FONT_SIZE  + _CICON_TEXT_LEFT_MARGIN + Math.abs(_CEXT_MARK_RIGHT_POS_LEFT_MARGIN) + _textRightSpacing
        //print("K3DCOMBOBOX BUTTON PARAM:", _imageLeftMargin, thisComboBoxButton.height /3, _CICON_TEXT_LEFT_MARGIN,  Math.abs(_CEXT_MARK_RIGHT_POS_LEFT_MARGIN) ,  _textRightSpacing);
    //(thisComboBoxButton.height /3 > 0 ? thisComboBoxButton.height /3 : 12), thisBtn.height = 0 when start up
    _touchButtonImage.sourceSize.height   : thisComboBoxButton.height /3
    property real _verticalOffset         : 0
    property alias _imageLeftMargin       : thisComboBoxButton._touchButtonImageLeftMargin
    _touchButtonImageVerticalOffset       : _verticalOffset
    _btnNormalSource                      : thisComboBoxButton._normalImageSource
    _btnHoveredSource                     : thisComboBoxButton._hoverImagesource
    _btnPressedSource                     : thisComboBoxButton._normalImageSource
    _btnTextVerticalAlignment             : Text.AlignVCenter
    _btnTextHorizontalAlignment           : Text.AlignLeft
    _btnTextPosition                      : K3DRC.BUTTON_TEXT_POS.RIGHT
    _btnType                              : _CBUTTON_TYPE_ICON_TEXT
    _btnTextWrap                          : Text.NoWrap

    //_CTEXT_ONLY_LEFT_MARGIN               : 10//buttonImage.width + 5
    //_btnTextPosition                      : K3DRC.BUTTON_TEXT_POS.TEXTONLY_LEFT
    _btnTextBold                          : true
    _btnTextColor                         : (_containsMouse && _itemsCount > 0) ? K3DRC.COLOR.ORANGE : K3DRC.COLOR.WHITE
    _btnTextPixelSizeScale                : 1.2

    // Operation
    property string _cFULL_TEXT           : ""
    property int  _itemsCount             : _itemList.length // _itemsCount do no change when _itemList.length change !!!!!!!!!!!!!

    on_CurrentItemChanged: {
        _cFULL_TEXT = (_itemList.length === 0                              ? // ! ACTUALLY, WE NEVER LET _currentItem < 0, THEREFORE _CFULL_TEXT IS "" ONLY IF NO _ITEMS
                       ""        : thisComboBoxButton._currentItem < 0 ?
                       _itemList[0].text : _itemList[thisComboBoxButton._currentItem].text);
        //print("_cFULL_TEXT  1 : ", _itemList.length,_cFULL_TEXT,_currentItem, _itemList );
        //print("_cFULL_TEXT 3 : ",_itemList.length, _cFULL_TEXT, _currentItem );
    }

    // --------------------------------------------------------------------------------------
    // Solution 1: Cut text until text.width < button.width
    //
    property string _cDEFAULT_TEXT        : "N/A"
    property string _itemSourceText       : (_cFULL_TEXT.length === 0) ? _cDEFAULT_TEXT : _cFULL_TEXT
    property real   _originalWidth        : 0
    Component.onCompleted: {
        //_cDEFAULT_TEXT = "N/A";
    }
    on_CDEFAULT_TEXTChanged: {
        _itemSourceText = _cDEFAULT_TEXT; // Just for Maintabbar when create new proj
    }
    on_CFULL_TEXTChanged: {
        _itemSourceText = (_cFULL_TEXT.length === 0) ? _cDEFAULT_TEXT : _cFULL_TEXT
    }


    on_OriginalWidthChanged: {
        thisComboBoxButton.width = _originalWidth;
        thisComboBoxButton._maxTextWidth = thisComboBoxButton.width - thisComboBoxButton._textLeftRightMargin;
        //print("_originalWidth", _originalWidth, thisComboBoxButton.width , thisComboBoxButton._maxTextWidth);
        var itemtext = (_cFULL_TEXT.length === 0) ? _cDEFAULT_TEXT : _cFULL_TEXT;
        calculateBtnWidth(itemtext);
        if(K3DUTIL.TEXT.isEmpty(textComparision.text))
            return;
        else {
            cutText(textComparision); // just for when rescale down window after start up
        }
        if(_originalWidth <= _textLeftRightMargin) {
            thisComboBoxButton._btnText = "";
        }
        //print("ORIGINAL WIDTH CHANGE : ", itemtext,_originalWidth, thisComboBoxButton.width, _textLeftRightMargin,textComparision.width, thisComboBoxButton._maxTextWidth, _itemSourceText, textComparision.text );
    }

    function calculateBtnWidth(itemSourceText) {
        _stopCutText = true;
        textComparision.text = itemSourceText;
        if(!_isNeedToCalculateWidth)
            return;
        if(_originalWidth > 0 && textComparision.width <= (_originalWidth - _textLeftRightMargin) && textComparision.width > 0) {
            thisComboBoxButton.width = textComparision.width + _textLeftRightMargin ;//+ _textRightSpacing;
            //print("BTN WIDTH CHANGE: ", thisComboBoxButton.width, textComparision.width, _textLeftRightMargin, thisComboBoxButton._maxTextWidth, _itemSourceText, textComparision.text );
        }
    }
    property bool _isNeedToCalculateWidth: true
    property bool _stopCutText : false // to stop binding loop for _maxTextWidth after calculateBtnWidth() call when set btnText to _cDEFAULT_TEXT,  if this button resize automatic, we have to disable this flag
    on_ItemSourceTextChanged: {
        //print("_cFULL_TEXT 2 : ", _cDEFAULT_TEXT,_itemSourceText, _cFULL_TEXT,_itemList );
        if(_originalWidth > 0 && thisComboBoxButton.width < _originalWidth) {
            thisComboBoxButton.width = _originalWidth;
            thisComboBoxButton._maxTextWidth = thisComboBoxButton.width - thisComboBoxButton._textLeftRightMargin;
        }
        //print("textComparision.text: ", textComparision.width, thisComboBoxButton._maxTextWidth, _itemSourceText, textComparision.text );
        calculateBtnWidth(_itemSourceText);
    }

    on_MaxTextWidthChanged: { //run before on_ItemSourceTextChanged
        textComparision.text = _itemSourceText;
        //print("on_MaxTextWidthChanged 1: ", textComparision.width, thisComboBoxButton._maxTextWidth, _itemSourceText, textComparision.text );
        if(!_stopCutText) // Now we do not auto resize this button anymore, so we can use this flag
            cutText(textComparision);
        //print("on_MaxTextWidthChanged 2: ", textComparision.width, thisComboBoxButton._maxTextWidth, _itemSourceText, textComparision.text );

    }

    function cutText(textLabel){
        //print("textLabel cut : ", textLabel.text,textLabel.width, thisComboBoxButton._maxTextWidth);
        if(thisComboBoxButton._maxTextWidth > 0 && Math.round(textLabel.width) > Math.round(thisComboBoxButton._maxTextWidth)){ // NEED to optimize
            _stopCutText = false; // Text cut when textlabel.width > maxtextWidth
            textLabel.text = K3DUTIL.TEXT.cutTextItemLastElementNoDot(textLabel.text);
            //print("TEXT CUT : ", thisComboBoxButton._btnText,textLabel.text, textLabel.width, thisComboBoxButton._maxTextWidth);
        }
        else {
            if(textLabel.text === thisComboBoxButton._cDEFAULT_TEXT || textLabel.text === "..." || textLabel.text === "" && thisComboBoxButton._cDEFAULT_TEXT !== "" ) {
                thisComboBoxButton._btnText = thisComboBoxButton._cDEFAULT_TEXT;
                //print("make binding loop", _cDEFAULT_TEXT);
                calculateBtnWidth(_cDEFAULT_TEXT);
            }
            else {
                thisComboBoxButton._btnText = textLabel.text + _CITEM_UNIT;
            }
            //print("thisComboBoxButton._btnText : ",thisComboBoxButton._btnText);
        }
    }
    K3DText {
        id : calculateText
        visible : false
        height  : 0
        text : thisComboBoxButton._cDEFAULT_TEXT
        font.bold                     : true
        _fontSizeScale                : thisComboBoxButton._btnTextPixelSizeScale
    }

    K3DText {
        id: textComparision
        visible : false
        height  : 0
        font.bold                     : true
        _fontSizeScale                : thisComboBoxButton._btnTextPixelSizeScale
        onWidthChanged: {
            //print("ON WIDTH CHANGE:", width);
            cutText(textComparision);
        }
    }
    property string _btnToolTipSetOutSide : ""
    function setButtonToolTip(){
        if (_btnToolTipSetOutSide.length > 0){
            return _btnToolTipSetOutSide;
        }
        else {
            if (_itemSourceText !== thisComboBoxButton._btnText)
                return _itemSourceText;
            else
                return "";
        }
    }
    _btnTooltip : setButtonToolTip()
    on_BtnToolTipSetOutSideChanged: {
        setButtonToolTip();
    }
    on_BtnTextChanged: {
        setButtonToolTip();
    }

    //-----------------------------------------------------------------------------------------------
    // Solution 2: Cut text by _CMAX_TEXT_LENGTH // Maybe it is better for new design
    //
//    _btnText                        : {
//        if(_btnTooltip.length > 0) // _btnTooltip === _cFULL_TEXT
//            return K3DUTIL.TEXT.getItemTextSetMaxLength(_cFULL_TEXT, _CMAX_TEXT_LENGTH) + _CITEM_UNIT;
//        else if(_cFULL_TEXT.length === 0)
//            return "N/A";
//        else
//            return _cFULL_TEXT + _CITEM_UNIT;
//    }
//    _btnTooltip                     : {
//        if(_cFULL_TEXT.length <= _CMAX_TEXT_LENGTH)
//            return "";
//        else
//            return _cFULL_TEXT;
//    }

//    on_ItemsChanged: { // This do not really change when _itemList change ?????????? just for start up
//        //modelListView.model = thisComboBoxButton._itemList;
//        print("ITEMS : ", _itemList);
//    }
    onClicked: {
        //print("_customItems: ", _itemList, _customItems);
        // --------------------------------------------------------------------------
        //thisComboBoxButton._setMaxWidthToZero = -1;
        // --------------------------------------------------------------------------
        if (thisComboBoxButton._itemList.length > 0) {
            thisComboBoxButton.state = thisComboBoxButton.state === K3DRC.STATE.SHOWN ?
                                      K3DRC.STATE.HIDDEN : K3DRC.STATE.SHOWN;
        }
        else {// It not true by both solution
            thisComboBoxButton.state = K3DRC.STATE.HIDDEN;
        }
    }
    // Set Extended Mark
    property bool _setShowExtendedMark : false
    _isExtendedType                    : true
    _showExtendedMark                  : _setShowExtendedMark ? ( _setShowExtendedMark && thisComboBoxButton._itemList.length > 0)
                                                              : (thisComboBoxButton._containsMouse && thisComboBoxButton._itemsCount > 0)
    _extendingState                    : K3DRC.EXTEND_DIRECTION.DOWNWARD
    // ===============================================================================
    // SIGNALS --
    //
    signal itemSelected(var itemIndex)
    signal itemRemoved(var itemIndex) // Internal -> out
    signal itemOrderedRemoved(var itemIndex) // From <- External

    signal setDefault(var itemIndex)
    signal subFuncButtonClicked(var subOpType)

    signal dropDownStateChanged(bool isShown)

    onItemRemoved: {
        // [MAIN TAB BAR]
        if (thisComboBoxButton._isMainTabBar) {
            //!NOTE: For Main Tab Bar[MainTabBar2] onItemRemoved()
            // => thisComboBoxButton.remItem() ALREADY!
        }

        // []
        else {
            thisComboBoxButton.remOneItem(itemIndex);
        }
    }

    // ===============================================================================
    // METHODS --
    //
    function initializeItemList(itemTextList, itemSubOpId, itemSubTooltip, itemSubNormalSource, itemSubHoverSource) {
        if(itemSubOpId === undefined)
            itemSubOpId = -1;
        if(itemSubTooltip === undefined)
            itemSubTooltip = "";
        if(itemSubNormalSource === undefined)
            itemSubNormalSource = "";
        if(itemSubHoverSource === undefined)
            itemSubHoverSource = "";
        // -------------------------------------------------
        //print("INITIALIZE ITEM: ", itemTextList, itemTextList.length, itemSubOpId, itemSubTooltip, itemSubNormalSource, itemSubHoverSource )
        _itemList = [];
        _itemsCount = 0;
        for(var i = 0; i < itemTextList.length; i++) { // Use _defaultItem, _currentItem as accessing index
            addItem(0, _defaultSubOpType, itemSubOpId, itemSubTooltip, itemSubNormalSource, itemSubHoverSource, itemTextList[i], true);
        }

        modelListView.model = _itemList;
        if(_itemList.length > 0)
            _currentItem = 0;
    }

    function setItemText(index, itemText){
        if(index < 0 || index >= _itemList.length)
            return;
        var cur = modelListView.currentIndex;
        _itemList[index].text = itemText;
        modelListView.model = _itemList;
        modelListView.currentIndex = cur;
        print('CUR INDEX AFTER TEXT CHANGED', _currentItem);
    }

    function addItem(itemGroupType, itemSubOpType, itemSubOpId, itemSubTooltip, itemSubNormalSource, itemSubHoverSource, itemText, isAppend) {
        _setMaxWidthToZero = -1;

        _itemsCount += 1;
        _btnToolTipSetOutSide = "";
        var currentIndex;
        if(isAppend) {
            _itemList.push({ text: itemText, type: itemGroupType, subOpType: itemSubOpType, subOpId: itemSubOpId, subTooltip: itemSubTooltip, subNormalSource: itemSubNormalSource, subHoverSource: itemSubHoverSource });

            // UPDATE CURRENT ITEM:
            currentIndex = _itemList.length -1;
        }
        else {
            _itemList.splice(0, 0, ({ text: itemText, type: itemGroupType, subOpType: itemSubOpType, subOpId: itemSubOpId, subTooltip: itemSubTooltip, subNormalSource: itemSubNormalSource, subHoverSource: itemSubHoverSource }));

            // UPDATE CURRENT ITEM:
            currentIndex = 0;
        }
        modelListView.model = _itemList;
        _currentItem = currentIndex;
        //print ("ADD ITEM : ", _itemsCount, itemGroupType, itemText, isAppend, _itemList, currentIndex , _currentItem);
    }

    function insertItem(index, itemGroupType, itemSubOpType, itemSubOpId, itemSubTooltip, itemSubNormalSource, itemSubHoverSource, itemText) {
        _setMaxWidthToZero = -1;

        _itemsCount += 1;

        _btnToolTipSetOutSide = "";
        if(index < 0 || index > _itemList.length) // index === length: insert = append!
            return;

        _itemList.splice(index, 0, ({ text: itemText, type: itemGroupType, subOpType: itemSubOpType, subOpId: itemSubOpId, subTooltip: itemSubTooltip, subNormalSource: itemSubNormalSource, subHoverSource: itemSubHoverSource }));

        // UPDATE CURRENT ITEM:
        //modelListView.model = null;
        modelListView.model = _itemList;
        _currentItem = index;

       // print ("ADD ITEM : ", itemGroupType, itemText, _currentItem,  _itemList);
    }

    function remItem(itemGroupType, index) {
        if(index < 0 || index > _itemList.length) {
            return;
        }
        // ---------------------------------------
        _setMaxWidthToZero = -1;

        if(_itemList[index].type === itemGroupType) {
            _itemList.splice(index, 1);
            _itemsCount -= 1;

            modelListView.model = null;
            modelListView.model = _itemList;
            _currentItem = _itemList.length > 0 ? 0 : -1;
        }
    }

    function remOneItem(index) {
        if(index < 0 || index > _itemList.length) {
            return;
        }
        // ---------------------------------------
        _setMaxWidthToZero = -1;

        //print("ITEM 1 : ", _itemList);
        _itemList.splice(index, 1);
        _itemsCount -= 1;
        //print("ITEM 2 : ", _itemList);

        //modelListView.model = null;
        modelListView.model = _itemList;
        _currentItem = _itemList.length > 0 ? 0 : -1;
    }

    function remAllItemsTOTALLY() {
        _setMaxWidthToZero = -1

        thisTabList._itemList = [];
        _itemsCount  = 0;

        //modelListView.model = null;
        modelListView.model = _itemList;
        _currentItem = _itemList.length > 0 ? 0 : -1;
        //print("REM ALL : ", _itemList, modelListView.model);
    }

    function remItemsByType(itemGroupType) {
        _setMaxWidthToZero = -1;

        var newItems      = [];

        for(var i = _itemList.length-1; i >= 0; i--) {
            if(_itemList[i].type !== itemGroupType) {
                newItems.push(_itemList[i]);
            }
        }

        _itemsCount = newItems.length;

        //modelListView.model = null;
        modelListView.model = _itemList = newItems;
        _currentItem = _itemList.length > 0 ? 0 : -1;

        //print("REM ALL OF TYPE: ",itemGroupType, _itemList, modelListView.model);
    }

    function getItemsCount(itemGroupType) {
        var count = 0;

        for(var i = _itemList.length-1; i >= 0; i--) {
            if(_itemList[i].type === itemGroupType) {
                count ++;
            }
        }

        return count;
    }

    function hideDropDown() {
        if(thisComboBoxButton.state !== K3DRC.STATE.HIDDEN)
            thisComboBoxButton.state = K3DRC.STATE.HIDDEN;
    }

    property bool   _showSubButton : false
    property bool   _isMainTabBar  : false
    function isItemEditable(index) {
        //print("_customItemNameList: ", _customItemNameList);
        if(_customItemNameList.length === 0) {
            return false;
        }

        return K3DUTIL.UTIL.indexOf(_customItemNameList, _itemList[index].text) !== -1;
    }

    property bool _isHideDropDownListOnItemSelection : false
    onItemSelected: {
        // Already done in itemClicked()
//        if(_isHideDropDownListOnItemSelection)
//            thisComboBoxButton.hideDropDown();
    }
    //property string _nameDropDownItemCannotDelete : ""
    property real _dropDownBtnFontSizeScale     : 1.1
    property int  _dropDownToolTipPos           : K3DRC.TOOLTIP_POS.LEFT
    property real _setMaxWidthToZero
    //property bool _enabelChangeDropDownWidth    : true
    property int    _defaultSubOpType          : K3DRC.ITEM_SUBOP_TYPE.EDIT
    property int    _dropDownItemBtnType        : _CBUTTON_TYPE_TEXT_ONLY

    property real   _dropDownImageSourceSize    : thisComboBoxButton.height / 3
    property real   _dropDownImageLeftMargin    : MAINGB._CDIALOG_BUTTON_HEIGHT / 4
    property bool   _showScrollBar              : false
    // ------------------------------------------------------------------------------
    // DROPDOWN LIST FRAME --
    //
    property int _CANCHORS_LEFT       : 0
    property int _CANCHORS_HORIZONTAL : 1
    property int _CANCHORS_RIGHT      : 2
    property int _dropDownHorizontalAnchors : 0
    on_DropDownMinWidthChanged: {
        modelListRect.width = Math.max(modelListRect._maxWidth, thisComboBoxButton._dropDownMinWidth); // List does not update until model update
        //print("UPDATE MAXWIDTH 1 : ", modelListRect.width, modelListRect._maxWidth, thisComboBoxButton._dropDownMinWidth);
    }
    on_DropDownMaxWidthChanged: { thisComboBoxButton._setMaxWidthToZero = -1;}// to reset maxdropdownn text width to zero

    property int _dropDownItemHeight : thisComboBoxButton.height
    property bool _isDropDownAnchorsRight: false

    property alias _modelListView   : modelListView
    K3DRectangle {
        id : modelListRect
        z : thisComboBoxButton.z - 10

        property int _marginValue        : 5
        property real _maxWidth
        anchors{
            top                          : thisComboBoxButton.bottom
            left                         : thisComboBoxButton.left
            leftMargin                   : _dropDownHorizontalAnchors === _CANCHORS_HORIZONTAL ? -((modelListRect.width - thisComboBoxButton.width) / 2) :
                                           _dropDownHorizontalAnchors === _CANCHORS_RIGHT      ? (thisComboBoxButton.width - modelListRect.width) : 0
        }
        enabled                          : thisComboBoxButton.state === K3DRC.STATE.SHOWN
        width                            : thisComboBoxButton._dropDownMinWidth
//        onWidthChanged: {
//            print("WIDTH CHANGE : ",modelListRect.width, modelListRect._maxWidth, thisComboBoxButton._dropDownMinWidth, thisComboBoxButton._dropDownMaxWidth );
//        }

        on_MaxWidthChanged: {
            modelListRect.width = Math.max(modelListRect._maxWidth, thisComboBoxButton._dropDownMinWidth);
            //print("UPDATE MAXWIDTH : ", modelListRect.width, modelListRect._maxWidth, thisComboBoxButton._dropDownMinWidth);
        }
        height                           : 0
        _backgroundSource                : K3DRC.BACKGROUND.MACHINE_BAR_DROP_DOWN_BG

        clip                             : false
        opacity                          : 0


        Image {
            anchors {
                left       : parent.left
                leftMargin : MAINGB.getRefSize(1)
                right      : parent.right
                rightMargin: MAINGB.getRefSize(1)
                top        : parent.top
            }

            height : MAINGB.getRefSize(2)
            source: K3DRC.BACKGROUND.MACHINE_BAR_TOP_SEPARATOR
            visible: thisComboBoxButton.state === K3DRC.STATE.SHOWN && _itemsCount > 0
        }
        Image {
            anchors {
                left       : parent.left
                leftMargin : MAINGB.getRefSize(1)
                right      : parent.right
                rightMargin: MAINGB.getRefSize(1)
                bottom     : parent.bottom
            }
            height : MAINGB.getRefSize(2)
            source : K3DRC.BACKGROUND.MACHINE_BAR_BOTTOM_SEPARATOR
            visible: thisComboBoxButton.state === K3DRC.STATE.SHOWN && _itemsCount > 0
        }

        // DROPDOWN LIST VIEW --
        //
        ListView {
            id : modelListView
            anchors.fill                : parent
            clip                        : modelListView.moving // have to be false to see tooltip of dropdown
            model                       : thisComboBoxButton._itemList
            enabled                     : thisComboBoxButton.state === K3DRC.STATE.SHOWN
            delegate                    : listDelegate
            highlight                   : highlightComponent
            highlightFollowsCurrentItem : false//true
            //interactive                 : true
            boundsBehavior              : Flickable.StopAtBounds
            displayMarginBeginning      : 0
            displayMarginEnd            : 0
        }
/*
        // ScrollView lose dropdownBtn Tooltip
//        ScrollView{
//            //contentItem : modelListView // By default
//            //horizontalScrollBarPolicy : Qt.ScrollBarAsNeeded // By default
//            id: dropDownScrollView
//            anchors.right: parent.right
//            width        : parent.width
//            height       : parent.height
//            clip         : false
//            style : ScrollViewStyle{
//                scrollBarBackground: Rectangle{
//                    clip : true
//                    visible : _showScrollBar
//                    anchors.right: parent.right
//                    anchors.rightMargin: 1
//                    anchors.top : parent.top
//                    border.color: "#939393"
//                    color               : "#333333"
//                    implicitWidth: 4
//                    implicitHeight: dropDownScrollView.height
//                }

//                handle : K3DRectangle{
//                    anchors.right: parent.right
//                    anchors.rightMargin: 2
//                    anchors.top : parent.top
//                    visible: _showScrollBar
//                    _hoverEnabled : true
//                    implicitWidth : 8
//                    implicitHeight: 30
//                    color : "#3C3C3C"
//                    _CORIG_BORDER_COLOR : _containsMouse ? "white" : "#939393"
//                    _CORIG_BORDER_WIDTH : 1
//                }
//                incrementControl: Rectangle {
//                    implicitWidth: 0//8
//                    implicitHeight: 0//3
//                }
//                decrementControl: Rectangle{
//                    implicitWidth: 0//8
//                    implicitHeight: 0//3
//                }
//            }

//            ListView {
//                id : modelListView
//                anchors.fill                : parent
//                model                       : thisComboBoxButton._itemList
//                enabled                     : thisComboBoxButton.state === K3DRC.STATE.SHOWN
//                delegate                    : listDelegate
//                highlight                   : highlightComponent
//                highlightFollowsCurrentItem : false//true
//                interactive                 : false
//            }
//        }
*/

        Component{
            id :  highlightComponent
            K3DRectangle{
                visible                     : thisComboBoxButton.state === K3DRC.STATE.SHOWN
                //anchors.margins : modelListRect._marginValue
                width      : thisComboBoxButton.state === K3DRC.STATE.HIDDEN  ? 0 : (modelListRect.width - modelListRect._marginValue)
//                onWidthChanged: {
//                    print("HIGHLIGHT WIDTH : ",width, height );
//                }
                height     : thisComboBoxButton.state === K3DRC.STATE.HIDDEN  ? 0 : (thisComboBoxButton._dropDownItemHeight    - modelListRect._marginValue)
                y          : thisComboBoxButton.state === K3DRC.STATE.HIDDEN  ? 0 :  modelListView.currentItem.y          + modelListRect._marginValue/2
                x          : modelListRect._marginValue/2
                color      : "#1F1E1D"
            }
        }

        //List view component
        Component {
            id :  listDelegate

            K3DItem {
                id                          : delegateItem
                z                           : 10
                _stayPut                    : true
                clip                        : false
                width                       : modelListRect.width
                height                      : thisComboBoxButton._dropDownItemHeight
                //anchors.horizontalCenter    : modelListRect.horizontalCenter
                _themeRect.color            : (_touchButtonMouseArea.containsMouse || subFuncButton._containsMouse) ?
                                                  "#3C3C3C" : K3DRC.COLOR.TRANSPARENT
                _widthHeightMargin          : modelListRect._marginValue


                _btnType                    : _dropDownItemBtnType // It's not complete true, because default is _CBUTTON_TYPE_TEXT_ONLY but if we
                //_isHoverChangeImage         : true
                // set image source outside it will be show
                _CTEXT_ONLY_LEFT_MARGIN     : MAINGB._BASE_FONT_SIZE
                _btnTextVerticalAlignment   : Text.AlignVCenter
                _btnTextHorizontalAlignment : Text.AlignLeft
                _btnTextBold                : thisComboBoxButton._btnTextBold
                _btnTextPixelSizeScale      : thisComboBoxButton._dropDownBtnFontSizeScale

                _btnTextPosition : (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? K3DRC.BUTTON_TEXT_POS.TEXTONLY_LEFT :
                                   K3DRC.BUTTON_TEXT_POS.RIGHT

                Component.onCompleted: {
                    textComparision1.text = modelData.text;
                    //
                    // [SIGNAL] itemOrderedRemoved(itemIndex)
                    thisComboBoxButton.itemOrderedRemoved.connect(doSubFunctionRemove);
                }

                property real _addedWidth  : (_btnTextPosition === K3DRC.BUTTON_TEXT_POS.TEXTONLY_LEFT) ? _CTEXT_ONLY_LEFT_MARGIN : _CICON_TEXT_LEFT_MARGIN + _touchButtonImage.sourceSize.height + subFuncButton.width //+ MAINGB._BASE_FONT_SIZE * 1.3
                property real _dropDownMaxWidth : thisComboBoxButton._dropDownMaxWidth
                on_DropDownMaxWidthChanged: {
                    //print("TextComparion change: ", _dropDownMaxWidth, textComparision1.text,modelData.text );
                    textComparision1.text = modelData.text;// to set text to full then cut it
                }

                K3DText {
                    id: textComparision1
                    visible : false
                    height  : 0
                    font.bold                     : true
                    _fontSizeScale                : thisComboBoxButton._btnTextPixelSizeScale
                    onWidthChanged: {
                        //print("textComparision 1 : ", textComparision1.text, textComparision1.width, thisComboBoxButton._setMaxWidthToZero);
                        if(thisComboBoxButton._dropDownMaxWidth > 0 && Math.round(width + _addedWidth) > Math.round(thisComboBoxButton._dropDownMaxWidth )) { // + _addedWidth
                            textComparision1.text = K3DUTIL.TEXT.cutTextItemLastElement(textComparision1.text);
                        }
                        else {
                            // Set Button Text
                            delegateItem._btnText = textComparision1.text + thisComboBoxButton._CITEM_UNIT;

                            // Reset _maxWidth to zero when update model or change _dropDownMaxWidth
                            if (thisComboBoxButton._setMaxWidthToZero === -1)
                                modelListRect._maxWidth = 0;

                            // Calculate _maxWidth.
                            modelListRect._maxWidth = Math.max(modelListRect._maxWidth, Math.max(thisComboBoxButton._dropDownMinWidth, delegateItem._addedWidth + width + MAINGB._BASE_FONT_SIZE * 1.3 ));
                            thisComboBoxButton._setMaxWidthToZero = modelListRect._maxWidth;
                            //print("textComparision 2 : ",modelListRect._maxWidth ,textComparision1.text, textComparision1.width, thisComboBoxButton._setMaxWidthToZero )
                        }
                    }
                }

                _btnTooltip : (modelData.text !== _btnText) ? modelData.text : ""

                _touchButtonImageLeftMargin : (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? 0 : _dropDownImageLeftMargin
                _touchButtonImage.sourceSize.height : (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? 0 : thisComboBoxButton._dropDownImageSourceSize
                _touchButtonImage.sourceSize.width  : (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? 0 : thisComboBoxButton._dropDownImageSourceSize
                _btnNormalSource : (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? "" : (_dropDownItemsImageNormalList.length === _itemList.length && _itemList.length > 0)
                                                                            ? _dropDownItemsImageNormalList[index] : MAINGB.getMainTabBarDropDownIcon(modelData.type, false)
                _btnHoveredSource: (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? "" : (_dropDownItemsImageHoverList.length  === _itemList.length && _itemList.length > 0)
                                                                            ? _dropDownItemsImageHoverList[index]  : MAINGB.getMainTabBarDropDownIcon(modelData.type, true)
                _btnPressedSource: (_btnType === _CBUTTON_TYPE_TEXT_ONLY) ? "" : (_dropDownItemsImageNormalList.length === _itemList.length && _itemList.length > 0)
                                                                            ? _dropDownItemsImageNormalList[index] : MAINGB.getMainTabBarDropDownIcon(modelData.type, false)

                _btnTooltipPos              : thisComboBoxButton._dropDownToolTipPos

                _touchButtonMouseArea.acceptedButtons: Qt.LeftButton | Qt.RightButton
                onClicked: {
                    //choose an item by left click (SELECTION ONLY)
                    if (mouse.button === Qt.LeftButton) {
                        // Hide dropdown itemList
                        if(_isHideDropDownListOnItemSelection)
                            thisComboBoxButton.hideDropDown();
                        // ---------------------------------------
                        if(index !== thisComboBoxButton._currentItem) {
                            thisComboBoxButton._currentItem = index;
                            // ---------------------------------------
                            /* PEDRO SAID: NOW DEFAULT AS CURRENT */
                            thisComboBoxButton._defaultItem = index;
                            thisComboBoxButton.setDefault(index);
                            thisComboBoxButton.itemSelected(index); // Must be emit after _currentItem updated.
                            // ---------------------------------------
                            if(thisComboBoxButton._setBtnImageSourceBySlectedItem) {
                                thisComboBoxButton._iconBaseSource   = thisComboBoxButton._dropDownItemsImageNormalList[index];
                                thisComboBoxButton._btnNormalSource  = thisComboBoxButton._iconBaseSource;
                                thisComboBoxButton._btnHoveredSource = thisComboBoxButton._dropDownItemsImageHoverList[index] ;
                                thisComboBoxButton._btnPressedSource = thisComboBoxButton._iconBaseSource;
                            }
                        }
                    }
                    //print("ITEM CLICKED : ", index,_currentItem,_itemList[index]);
                    /* PEDRO SAID: NOW DEFAULT AS CURRENT :
                    //set default by right click (DEFAULT ONLY)
                    else if (mouse.button === Qt.RightButton) {
                        thisComboBoxButton._defaultItem = index;
                        thisComboBoxButton.setDefault(index);
                    }
                    */
                }

                // DEFAULT MARK --
                //
                /* PEDRO SAID: NOW DEFAULT AS CURRENT */
                /*
                K3DImage {
                    anchors.verticalCenter : parent.verticalCenter
                    anchors.right          : parent.right
                    anchors.rightMargin    : 8
                    height                 : visible? delegateItem.height/2 : 0
                    width                  : height
                    visible                : thisComboBoxButton._defaultItem === index
                    source                 : K3DRC.ICON.MACHINE_BAR_DEFAULT_ICON
                    antialiasing           : true
                    smooth                 : true
                }
                */

                // EDIT MARK BUTTON --
                //
                function doSubFunctionRemove(itemIndex) {
                    if(itemIndex === index && modelData.subOpType === K3DRC.ITEM_SUBOP_TYPE.DEL)
                        subFuncButton.clicked(null);
                }

                function doSubFunctionEdit(itemIndex) {
                    if(itemIndex === index && modelData.subOpType === K3DRC.ITEM_SUBOP_TYPE.EDIT)
                        subFuncButton.clicked(null);
                }

                function doSubFunctionView(itemIndex) {
                    if(itemIndex === index && modelData.subOpType === K3DRC.ITEM_SUBOP_TYPE.VIEW)
                        subFuncButton.clicked(null);
                }

                K3DButton {
                    id : subFuncButton
                    //_CK3DOPID              : modelData.subOpId // Start EditMachineDialogAgent here-in!
                    anchors.verticalCenter : parent.verticalCenter
                    anchors.right          : parent.right
                    anchors.rightMargin    : MAINGB.getRefSize(8)
                    height                 : (thisComboBoxButton._isMainTabBar && index > 0) || // TAB BAR, EXCEPT TAB_MAIN_SCREEN: => Always show
                                             (thisComboBoxButton._showSubButton)?
                                             delegateItem.height/3   : 0

                    width                  : height
                    visible                : false //thisComboBoxButton.checkEditBtVisible ( index)
                    _btnType               : _CBUTTON_TYPE_ICON_ONLY
                    _btnTooltip            : modelData.subTooltip
                    _btnTooltipPos         : K3DRC.TOOLTIP_POS.TOP

                   // thisComboBoxButton.isItemEditable(index)
                    _isHoverChangeImage    : true
                    _btnNormalSource       : modelData.subNormalSource
                    _btnHoveredSource      : modelData.subHoverSource
                    _btnPressedSource      : modelData.subNormalSource

                    onClicked: {
                        thisComboBoxButton.hideDropDown(); // When delete dropdown Height do not update
                        if(modelData.subOpType === K3DRC.ITEM_SUBOP_TYPE.DEL) {
                            // DELETE ITEM HERE
                            //print("DELETE ITEM", index);

                            // OPERATION FUNCTION --
                            MAINGB.k3dRunOp(modelData.subOpId);

                            // 1- (To let handler read _itemList[index] & _itemList[index].type value!
                            thisComboBoxButton.itemRemoved(index);

                            // !NOTE THAT: FOR K3DRC.ITEM_SUBOP_TYPE.DEL, ITEM WAS ALREADY DELETED
                            // WHEN THE SIGNAL IS RECEIVED!
                        }
                        else if(modelData.subOpType === K3DRC.ITEM_SUBOP_TYPE.EDIT ||
                                modelData.subOpType === K3DRC.ITEM_SUBOP_TYPE.VIEW) {
                            //print("EDIT ITEM", index, thisComboBoxButton._itemList[index].text);
                            // Set the currently edited item
                            thisComboBoxButton._currentEditedItemText = thisComboBoxButton._itemList[index].text;

                            // OPERATION FUNCTION --
                            MAINGB.k3dRunOp(modelData.subOpId);

                            // [SIGNAL] subFuncButtonClicked() --
                            thisComboBoxButton.subFuncButtonClicked(modelData.subOpType);
                        }
                    }
                }

                // SEPERATOR
//                K3DRectangle {
//                    anchors.top              : delegateItem.bottom
//                    anchors.left             : delegateItem.left
//                    anchors.leftMargin       : 7
//                    anchors.right            : delegateItem.right
//                    anchors.rightMargin      : 7
//                    width                    : delegateItem.width * 0.9
//                    height                   : (index === _itemList.length - 1) ? 0 : 2
//                    _backgroundSource        : "qrc:///res/toolbar/mainToolBarSeparator.svg"
//                }
            } // End delegateItem
        } // End listDelegate Component

        // ----------------------------------------------------------------------------------------
        // ADD ITEM BUTTON --
        //
        K3DItem{
            id : addItemButton
            //clip : true
            anchors{
                top                     : parent.bottom
                topMargin               : -2
                horizontalCenter        : parent.horizontalCenter
            }
            enabled                     : thisComboBoxButton.state === K3DRC.STATE.SHOWN
            visible                     : thisComboBoxButton.state === K3DRC.STATE.SHOWN
            width                       : thisComboBoxButton.state === K3DRC.STATE.HIDDEN ? 0 : modelListRect.width

            height                      : thisComboBoxButton.state === K3DRC.STATE.HIDDEN ? 0 :
                                          (thisComboBoxButton._CADD_ITEM_BUTTON_TEXT.length > 0) ?
                                           thisComboBoxButton._dropDownItemHeight : 0
            _themeSource                : K3DRC.BACKGROUND.MACHINE_BAR_ADD_ITEM_BG
            _btnType                    : _CBUTTON_TYPE_TEXT_ONLY
            _btnTextColor               : _touchButtonMouseArea.containsMouse ? "white" : "#C45325"
            _btnText                    : thisComboBoxButton.state === K3DRC.STATE.HIDDEN ? "" : thisComboBoxButton._CADD_ITEM_BUTTON_TEXT
            property alias _addItemTextWidth : addItemText.width
            K3DText {
                id : addItemText
                visible : false
                height  : 0
                font.bold                     : addItemButton._btnTextBold
                _fontSizeScale                : addItemButton._btnTextPixelSizeScale
                text                          : thisComboBoxButton._CADD_ITEM_BUTTON_TEXT
            }

            _btnTextPosition            : K3DRC.BUTTON_TEXT_POS.TEXTONLY_CENTER
            _btnTextBold                : true
            _btnTextPixelSizeScale      : 1.1
        }
    }

    property real _thisButtonYPos : thisComboBoxButton.y // Need set outSide with Btn on Dialog ( = Dialog.y + btn.y)
    property real _mainHeightRef  : MAINGB._QAPP_HEIGHT - (3 * MAINGB._SCREEN_HEIGHT/30)
    property real _dropdownHeight : {
        if((_mainHeightRef - _thisButtonYPos > 0) &&
           (thisComboBoxButton._itemsCount * thisComboBoxButton._dropDownItemHeight > _mainHeightRef  - _thisButtonYPos)) {
            // _showScrollBar = true;
            return _mainHeightRef  - _thisButtonYPos;
        }
        else {
            //_showScrollBar = false;
            return thisComboBoxButton._dropDownItemHeight * thisComboBoxButton._itemsCount;//    _itemList.length; Using _itemcount for MainTabBar 2 Dropdown List
        }
    }

    //set default state is hidden
    state : K3DRC.STATE.HIDDEN
    onStateChanged: {
        thisComboBoxButton.dropDownStateChanged(K3DRC.STATE.SHOWN === state);
    }

    //defind state
    states: [
            State {
                name: K3DRC.STATE.SHOWN
                PropertyChanges { target: thisComboBoxButton;    _extendingState: K3DRC.EXTEND_DIRECTION.UPWARD }
                PropertyChanges { target: modelListRect;         opacity :1; height : _dropdownHeight}
            },
            State {
                name: K3DRC.STATE.HIDDEN
                //when: thisMachineButton._itemList.length === 0
                PropertyChanges { target: thisComboBoxButton;    _extendingState: K3DRC.EXTEND_DIRECTION.DOWNWARD }
                PropertyChanges { target: modelListRect;         height :0; opacity :0 }
            }
    ]
    transitions: Transition {
        //ParallelAnimation {
            NumberAnimation { target: modelListRect;        properties: "height, opacity"; easing.type:
                              Easing.InSine;               duration: 100}
        //}
    }
}
