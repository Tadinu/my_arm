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


GroupBox {
    id: thisGroupBox
    // title:
    Layout.fillWidth: true

    property string _themeColor : "#2B2B2B"
    style: Style {
               property Component panel: Rectangle {
                    anchors.leftMargin: -5
                    anchors.rightMargin: -5
                    anchors.bottomMargin: -2
                    anchors.topMargin: - titleText.height

                    border.color: "gray"
                    border.width: 1

                    color: K3DRC.COLOR.TRANSPARENT

                    Rectangle {
                        id: titleRect
                        anchors.fill: titleText
                        anchors.leftMargin: -3
                        anchors.rightMargin: -5
                        anchors.bottomMargin: -1
                        anchors.topMargin: -1
                        color: thisGroupBox._themeColor //K3DRC.COLOR.TRANSPARENT
                    }

                   Text {
                       id: titleText
                       anchors.left: parent.left
                       anchors.leftMargin: 10
                       anchors.bottom: parent.top
                       anchors.bottomMargin: -titleText.height/2
                       text: control.title
                       font.pixelSize: MAINGB._BASE_FONT_SIZE
                       color: control.enabled ? "white" : "gray"
                       renderType: Text.NativeRendering
                       font.italic: !control.enabled
                       //font.weight: Font.Bold
                       //font.pointSize: 8
                   }
               }
           }
}
