import QtQuick 2.0
import QtQuick.Window 2.1
import QtQuick.Controls 1.2  // Provide Qt control widgets
import QtQuick.Controls.Private 1.0

import QtQuick.Controls.Styles 1.3
import QtQuick.Layouts  1.1  // Provide Qt layouts

import MainGBSingletonObject 1.0
//import "qrc:///javascript/qmlcommonresource.js" as GEORC
////import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import "qrc:///qml/k3dBase"
Rectangle {
    id: thisLabel

    radius: MAINGB.getRefSize(3)
    clip  : true

    width : labelText.width * _sizeScale
    height: labelText.height * _sizeScale
    opacity: enabled ? 1 : 0.5
    property real  _sizeScale : 1.5
    property alias _label     : labelText.text
    property alias _labelScale: labelText._fontSizeScale
    property alias _color     : labelText.color
    property alias _fontSource: labelText._fontSource

    K3DText {
        id: labelText
        font.bold: true
        font.letterSpacing: 0
        anchors.centerIn: parent
    }
}
