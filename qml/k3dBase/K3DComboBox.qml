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

ComboBox {
    id: thisComboBox
    Layout.fillWidth: true

    property int _implicitWidth  : 100
    property int _implicitHeight : 24
    property int _fontSize : MAINGB._BASE_FONT_SIZE * _fontSizeScale
    property real _fontSizeScale : 1.25
    property string _background
    property real _boderWidth : 2
    property int _rightMarginButton : 11
    activeFocusOnPress: true
    property string _defaultText
    property bool   _isrotated : thisComboBox.pressed
    FontLoader { id: customFont; source: K3DRC.FONT.MAVEN_PRO_REGULAR }
    //property alias _currentLabelText : control.currentText
    property string _labelTextColor : "white"
    signal clicked(var mouse)
    property real _labelOpacity : 1
    property string _downArrow: K3DRC.ICON.COMBOBOX_EXTENSION_MARK
    property string _downArrowHover: K3DRC.ICON.COMBOBOX_EXTENSION_MARK_HOVER
    property string _labelBackgroundColor : K3DRC.COLOR.TRANSPARENT

    onCountChanged:{
        if(count === 0) {
            currentIndex = -1;
        }
    }

    style: ComboBoxStyle {

        //textColor: "white"
        //Label text
        dropDownButtonWidth: thisComboBox.width
        label: Label {
          id: labeltext
          verticalAlignment: Qt.AlignVCenter
          horizontalAlignment: Qt.AlignLeft
          anchors.left: parent.left
//          anchors.leftMargin: 5
          text: control.currentText//_currentLabelText //dropDown.styleData.text === "Default"?  _defaultText: control.currentText
          color: _labelTextColor//!control.enabled ? "#DADAD9" : "#6F6E6E"
          anchors.fill: parent
          opacity: _labelOpacity
          font.pixelSize: thisComboBox._fontSize
          font.family: customFont.name
          font.bold: true
        }

        //Label rectangle
        background: K3DRectangle {
            implicitWidth : thisComboBox._implicitWidth
            implicitHeight: thisComboBox._implicitHeight
            border.color  : K3DRC.COLOR.GRAY
            border.width  : thisComboBox._boderWidth
            color: _labelBackgroundColor

            _backgroundSource: thisComboBox._background
            _baseMouseArea.hoverEnabled: true

            //down arrow
            K3DRectangle {
                anchors.verticalCenter: parent.verticalCenter
                anchors.right: parent.right
                anchors.rightMargin: thisComboBox._rightMarginButton
                _baseMouseArea.propagateComposedEvents: true
                _baseMouseArea.hoverEnabled: true
                _backgroundSource: _baseMouseArea.containsMouse? _downArrowHover : _downArrow
                _rotation : _isrotated? 0 : 180
                visible   : thisComboBox.model !== null && thisComboBox.model.length > 1
                height    : parent.height/5
                width     : height * 2

                onBaseClicked: {
                    thisComboBox.clicked(mouse);
                }
            }
        }

        //styleData.type : MenuItemType.ScrollIndicator

        // drop-down customization here
        property Component __dropDownStyle: MenuStyle {
            id:dropDown
            __maxPopupHeight: 600
            __menuItemType: "comboboxitem" //"menuitem"
            frame: K3DRectangle {
                color: K3DRC.COLOR.GRAY
                border.color: K3DRC.COLOR.BLACK
                border.width: 2
                //border.color : "#333130"
                //border.width: 0
                //radius: 5
            }
            submenuPopupDelay: 1000
            submenuOverlap:20
            itemDelegate.label:             // an item text
                Text {
                    id: dropdowntext
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignLeft
                    font.pixelSize: thisComboBox._fontSize
                    font.family: customFont.name
                    //color: styleData.selected ? "white" : "black"
                    color: "white"
                    text: styleData.text

                    font.bold: true
            }
            //_currentLabelText : dropdowntext.text === "Default" ? "12321":control.currentText
            itemDelegate.background: Rectangle {  // selection of an item
                radius: 1
                color: styleData.selected ? "#3C3C3C" : K3DRC.COLOR.TRANSPARENT
            }

            __scrollerStyle: ScrollViewStyle {
                transientScrollBars: true
                    handle: Item {
                        implicitWidth: thisComboBox.width/20
                        implicitHeight: thisComboBox.height
                        Rectangle {
                            color: "#3C3C3C"
                            //radius: 3
                            anchors.fill: parent
                            anchors.topMargin: 6
                            anchors.leftMargin: 4
                            anchors.rightMargin: 4
                            anchors.bottomMargin: 6
                        }
                    }
                    scrollBarBackground: Item {
                        implicitWidth: thisComboBox.width/10
                        implicitHeight: thisComboBox.height
                    }
            }
            //scrollIndicator
        }

//        property Component __popupStyle: Style {
//            property int __maxPopupHeight: 400
//            property int submenuOverlap: 0

//            property Component frame: Rectangle {
//                width: (parent ? parent.contentWidth : 0)
//                height: (parent ? parent.contentHeight : 0) + 2
//                border.color: "#1F1E1D"
//                border.width: 1
//                property real maxHeight: 500
//                property int margin: 1
//            }

//            property Component menuItemPanel: Text {
//                text: "NOT IMPLEMENTED"
//                color: "white"
//                font {
//                    pixelSize: 14
//                    bold: true
//                }
//            }

//            property Component __scrollerStyle: null
//        }
    }
}
