import QtQuick 2.3

// http://qt-project.org/doc/qt-5/qtquickcontrols-index.html
import QtQuick.Controls 1.2 // Provide Qt control widgets
import QtQuick.Controls.Styles 1.2
import QtQuick.Window 2.1

import com.k3d.qmladapter 1.0
import MainGBSingletonObject 1.0

import "qrc:///javascript/qmlcommonresource.js" as K3DRC
//import "qrc:///javascript/qmlcommonutil.js" as K3DUTIL
/* Use Rectangle instead of ToolButton for freedom in scaling the button icon */

Rectangle {
    id: functionButton

    width: buttonImage.width + buttonText.width + _CTEXT_MARGIN

    property alias  _iconSource       : buttonImage.source // Use internally only!
    property string _iconBaseSource   : _btnNormalSource
    property string _btnNormalSource
    property string _btnHoveredSource
    property string _btnPressedSource

    // Mouse Area
    property bool   _isWheelPropagated   : true
    property bool   _containsMouse       : false

    property real   _disabledOpacity   : 0.8
    visible : true
    opacity : enabled ? 1 : functionButton._disabledOpacity
    clip    : false
    color   : K3DRC.COLOR.TRANSPARENT
    antialiasing: true
    smooth: true
    property int    _CORIG_BORDER_WIDTH : 0
    property string _CORIG_BORDER_COLOR : "white"
    border.width: _CORIG_BORDER_WIDTH
    border.color: _CORIG_BORDER_COLOR

    property bool  _isHighlightBorderOnHover : false
    property int    _CHIGHLIGHT_BORDER_WIDTH : 3
    property string _CHIGHLIGHT_BORDER_COLOR : "#939393"//"#B5B5B5"

    // == Signals =====================
    //
    signal clicked(var mouse)
    signal pressed
    signal released
    signal mouseIn
    signal mouseOut

    property alias _functionButtonImage : functionButton
    property alias _btnRotation  : functionButton.rotation
    K3DImage {
        id: buttonImage
        fillMode: Image.PreserveAspectFit
        smooth: true
        antialiasing: true
        mipmap: true
        source: functionButton._iconBaseSource
        height: functionButton.height
        sourceSize.height: functionButton.height
        sourceSize.width: functionButton.height
        Component.onCompleted: {
            switch(functionButton._btnTextPosition)
            {
            case K3DRC.BUTTON_TEXT_POS.RIGHT:
                anchors.verticalCenter = functionButton.verticalCenter;
                anchors.left           = functionButton.left;
                break;
            case K3DRC.BUTTON_TEXT_POS.LEFT:
                anchors.verticalCenter = functionButton.verticalCenter;
                anchors.right          = functionButton.right;
                break;
            default:
                break;
            }
        }
    }

    property alias  _btnText                   : buttonText.text
    property alias  _btnTextBold               : buttonText.font.bold
    property int    _btnTextPosition           : K3DRC.BUTTON_TEXT_POS.RIGHT
    property alias  _btnTextRotation           : buttonText.rotation
    property alias  _btnTextStyle              : buttonText.style
    property string _btnTextStyleColor         : "white"
    property alias  _btnTextFontSource         : buttonText._fontSource

    property string _btnTextColor              : "white" //: buttonText.color // Cannot use alias for color type...
    property alias  _btnTextPixelSize          : buttonText.font.pixelSize
    property alias  _btnTextPixelSizeScale     : buttonText._fontSizeScale
    property alias  _btnTextLetterSpacing      : buttonText.font.letterSpacing
    property alias  _btnTextFontCapitalization : buttonText.font.capitalization
    property alias  _btnTextVerticalAlignment  : buttonText.verticalAlignment
    property alias  _btnTextHorizontalAlignment: buttonText.horizontalAlignment

    property alias  _buttonText                : buttonText
    property real   _CTEXT_MARGIN              : MAINGB._SCREEN_HEIGHT / 162
    property alias  _btnTextWrap               : buttonText._wrapMode
    property alias  _btnMaxTextWidth           : buttonText._maxTextWidth
    K3DText {
        id: buttonText
        z : 65534
        font.letterSpacing : 1
        color              : functionButton._btnTextColor
        styleColor         : functionButton._btnTextStyleColor

        Component.onCompleted: {
            switch(functionButton._btnTextPosition)
            {
            case K3DRC.BUTTON_TEXT_POS.RIGHT:
                anchors.verticalCenter = functionButton.verticalCenter;
                anchors.left           = buttonImage.right;
                anchors.leftMargin     = _CTEXT_MARGIN;
                break;
            case K3DRC.BUTTON_TEXT_POS.LEFT:
                anchors.verticalCenter = functionButton.verticalCenter;
                anchors.right          = buttonImage.left;
                anchors.rightMargin    = _CTEXT_MARGIN;
                break;
            default:
                break;
            }
        }
    }

    property alias _functionButtonMouseArea : btnMouseArea
    property alias _btnHoverEnabled      : btnMouseArea.hoverEnabled
    MouseArea { // ! Don't use K3DMouseArea here-in!!! (For calling refreshUI() unexpectedly
        id:btnMouseArea
        anchors.fill: functionButton
        // Clicked --
        onClicked: {
            functionButton.clicked(mouse);
        }
        // Pressed --
        onPressed: {
            functionButton.pressed();
            functionButton._iconBaseSource = functionButton._btnPressedSource;
            // -- Pressed background
        }
        onReleased:{
            functionButton.released();
            // -- Released background
            functionButton._iconBaseSource =  functionButton._btnNormalSource;
        }

        onWheel: {
            wheel.accepted = !functionButton._isWheelPropagated;
        }
        // Hovered --
        hoverEnabled: true // FOR USING onEntered & onExited

        //Change cursor key to pointinghand
        cursorShape: hoverEnabled ? Qt.PointingHandCursor : Qt.ArrowCursor

        onEntered: {
            functionButton._containsMouse = true;
            functionButton.mouseIn();
            // ------------------------------------------

            if(functionButton._isHighlightBorderOnHover) {
                functionButton.border.color = functionButton._CHIGHLIGHT_BORDER_COLOR;
            }
            // -- Hovered background
            functionButton._iconBaseSource = functionButton._btnHoveredSource;
            _btnTextStyle = Text.Sunken;
        }
        onExited:{
            functionButton._containsMouse = false;
            functionButton.mouseOut();
            // ------------------------------------------
            if(functionButton._isHighlightBorderOnHover) {
                functionButton.border.color = functionButton._CORIG_BORDER_COLOR;
            }
            // -- Normal background
            functionButton._iconBaseSource = functionButton._btnNormalSource;
            _btnTextStyle = Text.Normal;
        }
    }
}
