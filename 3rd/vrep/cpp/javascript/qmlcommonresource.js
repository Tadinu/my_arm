// qmlcommonresource.js
.pragma library

var LOGIN_NFC = "https://my.nyomo.com/api/vendor/login"//?email=admin@nyomo.com&password=123456"
var REGISTER_NFC = "https://my.nyomo.com/api/nfc/register"//?user_id=xxx&access_token=yyy&serial=card-serial-come-here&lot_id=1"
var GET_CURRENT_NFC_INFO = "https://my.nyomo.com/api/vendor/materials"

var SIZE = {
    SCREEN_FULL_HD_WIDTH: 1920,
    SCREEN_FULL_HD_HEIGHT : 1080,
    SCREEN_MIN_WIDTH    : 800,
    SCREEN_MIN_HEIGHT   : 480,
    TEXT_BASE_SIZE      : 449/87,  // SCREEN FULL HD (1080) - Text Following Height
    TEXT_STANDARD_SIZE  : 1049/87, // [12]: 87 = (1080-31)/12
    ITEM_CONTEXT_MENU_MIN_HEIGHT : 15, // 35 Width FULL HD (1080) ( 15 = 449 * 35 / 1049)
    STANDARD_SCALE      : 1/87,
    STANDARD_SCALE_II   : 1/80,
    ITEM_TEXT_MAX_LENGTH: 30
};

var VAL = {
    INCHTOMM : 25.4,
    MMTOINCH : 0.0393701
};

var MARGIN = {
    BUTTON_ICON_TEXT_HORIZONTAL: 7
};

var ANIMATION_TYPE = {
    GEOMETRY : 1,
    IMAGE    : 2
};

var ANIMATION_DURATION = {
    FLICKABLE     : 200,
    TEXT          : 100,
    IMAGE         : 200,
    OPACITY       : 50,
    SHRINK_EXTEND : 100
};

var COLOR = {
    TRANSPARENT: "transparent",
    WHITE      : "white"      ,
    BLUE       : "#3A90DF"    ,
    GREEN      : "#4FB5AD"    ,
    RED        : "#FD595B"    ,
    GRAY       : "#AAAFB3"    ,
    LIGHTGRAY  : "#E2E7EB"    ,
    BLACK      : "#2B2B29"    ,
};

var TOOLTIP_POS = {
    TOP           : 1,
    LEFT          : 2,
    BOTTOM        : 3,
    BOTTOM_LEFT   : 4,
    RIGHT         : 5,
    TOPCORNER     : 6,
    BOTTOMCORNER  : 7
};

var BUTTON_TEXT_POS = {
    TOP   : 1,
    LEFT  : 2,
    BOTTOM: 3,
    RIGHT : 4,
    TEXTONLY_LEFT   : -1,
    TEXTONLY_RIGHT  : -2,
    TEXTONLY_CENTER : -3
};

var NOTIFY_BUTTON_TYPE = {
    STANDARD : 1,
    CUSTOM   : 2
};

var SWITCH_TEXT_POS = {
    RIGHT: 1,
    LEFT: 2
}

var LIST_MODEL_TYPE = {
    UNKNOWN  : -1,
    QML      : 1,
    CPP      : 2,
};

var EXTEND_DIRECTION = {
    UPWARD   : 1,
    DOWNWARD : 2
}

var STATE = {
    NONE        : "None",
    HIDDEN      : "Hidden",
    SHOWN       : "Shown",
    PROGRESSING : "Progressing"
};

var ITEM_STATE = {
    NORMAL     : 0x01,
    FUNCTIONAL : 0x02,
    OK         : 0x04,
    NOK        : 0x08,
    WARN       : 0x10
};

var ITEM_SUBFUNC_TYPE = {
    EDIT : 1,
    DEL  : 2
};

var SPINBOX_TYPE = {
    NONE   : 0,
    DEGREE : 1,
    METRIC : 2
};

var CONST_STRING = ["N/A"];

var PROJ_OBJ_FORMAT = [".stl",".kspx",".ksp",".slc",".ply",".3ds",".dae",".obj",".fbx",".max",".dxf"];

// LIST MODEL ITEM PROPERTY NAME ===================================================
var MODEL_PROP = {
    ITEM_LIST_INDEX           : "_itemListIndex"         ,
    ITEM_OPERATION_NEED_INDEX : "_itemOperationNeedIndex",
    ITEM_K3DOP_ID             : "_itemK3DOpId"           ,
    ITEM_K3DOP_SUB_ID         : "_itemK3DSubOpId"        ,
    ITEM_K3DITEM_ID           : "_itemK3DItemId"         ,
    ITEM_VISIBLE              : "_itemVisible"           ,
    ITEM_ENABLED              : "_itemEnabled"           ,
    ITEM_BUTTON_TYPE          : "_itemBtnType"           ,
    ITEM_TOOLTIP              : "_itemTooltip"           ,
    ITEM_TOOLTIP_POS          : "_itemTooltipPos"        ,
    ITEM_TEXT                 : "_itemText"              ,
    ITEM_TEXT_BOLD            : "_itemTextBold"          ,
    ITEM_TEXT_STYLE           : "_itemTextStyle"         ,
    ITEM_TEXT_STYLE_COLOR     : "_itemTextStyleColor"    ,
    ITEM_TEXT_FONT_SOURCE     : "_itemTextFontSource"    ,
    ITEM_TEXT_COLOR           : "_itemTextColor"         ,
    ITEM_TEXT_PIXEL_SIZE_SCALE: "_itemTextPixelSizeScale",
    ITEM_TEXT_POS             : "_itemTextPosition"      ,
    ITEM_TEXT_ROTATION        : "_itemTextRotation"      ,
    ITEM_TEXT_V_ALIGN         : "_itemTextVerticalAlignment",
    ITEM_TEXT_H_ALIGN         : "_itemTextHorizontalAlignment",
    ITEM_BASE_SOURCE          : "_itemBaseSource",
    ITEM_NORMAL_SOURCE        : "_itemNormalSource"      ,
    ITEM_HOVERED_SOURCE       : "_itemHoveredSource"     ,
    ITEM_PRESSED_SOURCE       : "_itemPressedSource"     ,
    ITEM_FUNC_BUTTON_NORMAL_SOURCE : "_itemFunctionBtnSourceNormal",
    ITEM_FUNC_BUTTON_HOVER_SOURCE  : "_itemFunctionBtnSourceHover" ,

    ITEM_IS_EXTENDED          : "_itemIsExtended"        ,
    ITEM_EXTENDED_DIRECTION   : "_itemExtendedDirection" ,
    ITEM_SHOWN_EXTENDED       : "_itemShownExtended"     ,
    ITEM_FORCE_SHOWN_EXTENDED : "_itemForceShownExtended",
    ITEM_EXT_LIST_MODEL       : "_itemExtListModel"      ,
    ITEM_EXT_LIST_MODEL_TYPE  : "_itemExtListModelType"  ,
    ITEM_EXTENDED_LIST        : "_itemExtendedList"      ,

    // ---------------------------
    // Functional
    ITEM_STATE                : "_itemState"             ,
    ITEM_FUNCTION_MARK_SOURCE : "_itemFunctionalMarkSrc"
};

// FONT RESOURCES ==================================================================
var FONT = {
    MAVEN_PRO_REGULAR           : "qrc:///font/MavenPro-Regular.ttf",
    MAVEN_PRO_MEDIUM            : "qrc:///font/MavenPro-Medium.ttf",
    MAVEN_PRO_BOLD              : "qrc:///font/MavenPro-Bold.ttf",

    DIGITAL                     : "qrc:///font/Digital.ttf",
    DIGITAL_ITALIC              : "qrc:///font/Digital_Italic.ttf"
};

// DATE/TIME RESOURCES =============================================================
var DATE = {
    DATE_FORMAT                 : "yyyy-MM-dd"
};

// TEXT RESOURCES ==================================================================
var TEXT = {
    EMPTY                       : "",
    MICROMETER                  : "µm",
    MILLIMETER                  : "mm",   // cmmInf::ks_Metric_MM
    INCH                        : "inch", // cmmInf::ks_Metric_Inch
    DEGREE                      : "°",

    ON                          : qsTr("On"),
    OFF                         : qsTr("Off"),

    ITEM_MAX_LEN                : 20,

    BTN_OK     : qsTr("Ok"),
    BTN_YES    : qsTr("Yes"),
    BTN_NO     : qsTr("No"),
    BTN_NO_ALL : qsTr("No All"),
    BTN_CANCEL : qsTr("Cancel")
};

// TEXT RESOURCES ========================================================================

// WARNING MESSAGE =======================================================================

// ERROR MESSAGE =========================================================================

// CRITICAL MESSAGE ======================================================================


