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

K3DButton {
    id: touchTabButton
    _btnType: 1 // Text-only

    //width : _touchButtonText.width + tabButtonHightlightBar.width + 15
    //height: _touchButtonText.height

    // Text properties
    _btnTextLetterSpacing     : 2
    _btnTextFontCapitalization: Font.AllUppercase
    _btnTextColor             : "black"
    _btnTextRotation          : -90
    _isHoverChangeImage       : true

    property int _tabId
    onClicked: {
        parent._highlighted = _tabId;
        parent.highlightDehighlight(parent._highlighted);
        tabButtonHightlightBar.visible = true;
    }

    // Hightlight Bar
    property alias _tabButtonHighlightBar: tabButtonHightlightBar
    Rectangle {
        id: tabButtonHightlightBar

        width : 1
        height: 2*touchTabButton.height
        z:65535

        anchors.right: touchTabButton.right
        anchors.verticalCenter: touchTabButton.verticalCenter
        antialiasing: true
        color: "black"

        Component.onCompleted: {
            visible = _tabId == 0;
        }
    }
}
