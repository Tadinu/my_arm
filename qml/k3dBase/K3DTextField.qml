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

TextField {
    id: thisTextField
    // title:
    Layout.fillWidth: true

    property int _implicitWidth  : _implicitHeight * 4
    property int _implicitHeight : MAINGB._BASE_FONT_SIZE * 2
    property string _backgroundColor : K3DRC.COLOR.TRANSPARENT//"#1E1D1C"
    property int _fontSize       : MAINGB._BASE_FONT_SIZE * _fontSizeScale
    property real _fontSizeScale : 1.0
    property bool _fontBold      : false
    property string _background
    property var _fontSource      : K3DRC.FONT.MAVEN_PRO_REGULAR
    property string _textColor    : "#DADAD9"
    property string _borderColor  : "gray"
    property real   _borderWidth  : 1
    property string _selectedTextColor : "black"
    property string _selectionColor    : "white"
    property bool _isCursorVerticalCenter  : true // !!!! true -> padding.top = 0 -> cursor will be AlignVCenter; false -> padding.top = ... -> text will be AlignVCenter
    // Select text to see verticalAlignment.
    property string _backgroundSource : ""
    property real   _backgroundRadius : 2
    verticalAlignment  : Text.AlignVCenter
    //horizontalAlignment: Text.AlignHCenter // Default Left

    FontLoader { id: customFont; source: _fontSource }
    font.pixelSize : thisTextField._fontSize
    font.family    : customFont.name
    font.bold      : thisTextField._fontBold
    property real _leftMargin : font.pixelSize / 2
    property real _topMargin  : _isCursorVerticalCenter ? 0 : (implicitHeight - font.pixelSize) / 2
    style: TextFieldStyle {
        textColor: control.enabled ? _textColor : "#6F6E6E"
        selectedTextColor: _selectedTextColor
        selectionColor: _selectionColor

        padding {
            top   : thisTextField._topMargin
            left  : thisTextField._leftMargin
            right : padding.left
            bottom: padding.top
        }

        background: K3DRectangle {
            radius: thisTextField._backgroundRadius
            implicitWidth: thisTextField._implicitWidth
            implicitHeight: thisTextField._implicitHeight
            border.color: thisTextField._borderColor
            border.width: thisTextField._borderWidth
            color: thisTextField._backgroundColor
            _backgroundSource : thisTextField._backgroundSource
        }
    }
}
