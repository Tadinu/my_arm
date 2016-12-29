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

Text {
    id                 : thisText
    //text
    z: 65534
    antialiasing       : true
    smooth             : true
    property var _fontSource : K3DRC.FONT.MAVEN_PRO_REGULAR
    // Width, height: auto defined
    //width
    //height
    property real _fontSizeScale  : 1.0
    font.pixelSize                : MAINGB._BASE_FONT_SIZE * _fontSizeScale

    property alias _styleText     : thisText.style
    property alias _styleColor    : thisText.styleColor


    FontLoader { id: customFont; source: _fontSource }

    font.family        : customFont.name
    font.letterSpacing : 1
    color              : "white"
    //style              : Text.Normal
    //styleColor         : "white"
    property int _wrapMode : Text.WordWrap
    wrapMode           : thisText._wrapMode
    verticalAlignment  : Text.AlignVCenter
    horizontalAlignment: Text.AlignHCenter
    lineHeightMode     : Text.ProportionalHeight
    //renderType         : Text.NativeRendering

    // ------------------------------------------------------------------------------------------------------
    // CUT TEXT
    //
    property string _originalText  : text
    property real _maxTextWidth    : 0

    onWidthChanged: {
        _originalText = _originalText;// save original text before cut

        if(_maxTextWidth > 0 && thisText.width > _maxTextWidth) {
            thisText.text = K3DUTIL.TEXT.cutTextItemLastElement(thisText.text );
            //print("K3D TEXT WIDTH CHANGE", thisText.text,_originalText);
        }
    }

    function autoWidthAdapt(refWidth) {
        var sizeScale = thisText.width / refWidth;
        //print('SIZE SCALE', width, refWidth);
        if(sizeScale > 1){
            _fontSizeScale = _fontSizeScale / sizeScale;
        }
    }

//    MouseArea {
//        id: textMouseArea
//        anchors.fill: thisText
//    }
}
