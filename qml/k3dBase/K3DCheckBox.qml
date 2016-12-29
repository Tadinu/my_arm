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
import "qrc:///qml/k3dBase"

CheckBox {
    id: thisCheckBox
    Layout.fillWidth: true
    property int _spacing : K3DRC.MARGIN.BUTTON_ICON_TEXT_HORIZONTAL
//    property int _implicitWidth  : 100
//    property int _implicitHeight : 24
    property real _labelFontSizeScale : 1
    property bool _labelFontBold      : false
    property color _colorLabel: "white"
    //decimals: 0
    //stepSize: 1
    style: CheckBoxStyle {
        indicator: Rectangle {
                        implicitWidth: MAINGB._CDIALOG_CHECKBOX_HEIGHT
                        implicitHeight: implicitWidth
                        radius: implicitWidth/2
                        border.color: control.activeFocus || control.hovered ? "white" : "#4B4B4B"
                        border.width: 1
                        color: "black"
                        smooth: true
                        antialiasing: true
                        Rectangle {
                            visible: control.checked
                            color: "#3FC6F1"
                            width: parent.width/2
                            height: width
                            radius: width/2
                            anchors.centerIn: parent
                            smooth: true
                            antialiasing: true
                        }
        }
        label: K3DText {
            color: thisCheckBox._colorLabel
            text : control.text // thisCheckBox.text
            _fontSizeScale : thisCheckBox._labelFontSizeScale
            font.bold      : thisCheckBox._labelFontBold
        }
        spacing: thisCheckBox._spacing
    }
}
