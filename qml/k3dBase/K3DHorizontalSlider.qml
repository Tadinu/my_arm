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

Slider {
    id: thisSlider
    //anchors.centerIn: parent

//    stepSize: 0.1
//    maximumValue: 5
//    minimumValue: -5

    property string _sliderBackground : K3DRC.BACKGROUND.SLIDER_LINE_BACKGROUND
    property string _sliderHandleIcon : K3DRC.BACKGROUND.SLIDER_BUTTON_BACKGROUND
    property real   _sliderGrooveImplicitHeight : MAINGB._CDIALOG_SLIDER_BG_HEIGHT
    property real   _sliderHandleImpliciHeight  : MAINGB._CDIALOG_CHECKBOX_HEIGHT
    //tickmarksEnabled: true
    orientation: Qt.Horizontal

    signal clicked()
    style: SliderStyle {
        id: sliderStyle
        groove: K3DRectangle {
            implicitWidth : thisSlider.width
            implicitHeight: _sliderGrooveImplicitHeight
            _backgroundSource: thisSlider._sliderBackground
            onBaseClicked      : thisSlider.clicked()
        }
        handle: K3DRectangle {
            //anchors.centerIn: parent
            implicitWidth : implicitHeight
            implicitHeight: _sliderHandleImpliciHeight
            radius        : implicitHeight * 3 / 4
            _backgroundSource: thisSlider._sliderHandleIcon
        }
    }
}
