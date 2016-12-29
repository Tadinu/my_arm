import QtQuick 2.5

import MainGBSingletonObject 1.0

import "qrc:///javascript/qmlcommonresource.js" as K3DRC
MouseArea {
    id: mouseArea
    anchors.fill: parent

    onClicked: {
        // !!! REFRESH UI
        MAINGB.refreshMainWindowUI();
    }
}
