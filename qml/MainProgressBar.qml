import QtQuick 2.0

import QtQuick.Layouts  1.1  // Provide Qt layouts
import QtQuick.Dialogs  1.2
import MainGBSingletonObject 1.0
import "qrc:///javascript/qmlcommonresource.js" as GEORC
//import "qrc:///javascript/qmlcommonutil.js" as GEOUTIL
import "qrc:///qml/k3dBase"

K3DRectangle {
    id: thisProgressBar
    z: 0

    _CHIGHLIGHT_BORDER_WIDTH : MAINGB._CITEM_BORDER_WIDTH
    _CORIG_BORDER_WIDTH      : 0
    _CORIG_BORDER_COLOR      : "white"

    property real _originalWidth
    property real _CITEM_SPACING: MAINGB._SCREEN_HEIGHT / 108
    antialiasing: true

    property real   _progressValue      // [0,100]
    property int    _currentCircleIndex : -1
    property string _progressText       : "Progress Text!"
    property string _progressIconSource
    property string _progressIconText

//    property string _CCOLOR_CIRCLE_PROGRESSING  : "#0AB1D1"
//    property string _CCOLOR_CIRCLE_SETTLED      : "orange"
    // == Methods
    state                                       : GEORC.STATE.HIDDEN

    function show() {
        if(thisProgressBar.state === GEORC.STATE.PROGRESSING) {
            //_highLightAnimation.start();
            //gbMainAppWindow.playSound("qrc:///sound/chime_big_ben.wav");
        }
        else {
            thisProgressBar.state = GEORC.STATE.PROGRESSING;
            // !Note: Disable the whole main window(set as Busy) is already done in gbMainAppWindow.showProgressBar(isShown)!
        }
    }

    function hide() {
        if(thisProgressBar.width === thisProgressBar._originalWidth) {
            thisProgressBar.state = GEORC.STATE.HIDDEN;
        }
    }

    function updateProgress() {
        var i;

        for(i = 0; i <= thisProgressBar._currentCircleIndex; i++)
            progressCircleGroup.itemAt(i)._backgroundSource = progressCircleGroup.itemAt(i)._CSTYLE_ORANGE;

        for(i = thisProgressBar._currentCircleIndex + 1; i < progressCircleGroup.count; i++) {
            progressCircleGroup.itemAt(i)._backgroundSource = progressCircleGroup.itemAt(i)._CSTYLE_BLACK;
        }
    }

    function startProgressing() {
        for(var i = thisProgressBar._currentCircleIndex + 1; i < progressCircleGroup.count; i++) {
            progressCircleGroup.itemAt(i)._backgroundSource = progressCircleGroup.itemAt(i)._CSTYLE_BLACK;
        }
        if(thisProgressBar._currentCircleIndex <= progressCircleGroup.count-2)
            progressCircleGroup.itemAt(thisProgressBar._currentCircleIndex + 1)._progressingAnimation.start();
    }

    // ------------------------------------------------------------------------
    Row {
        id: mainRow
        //anchors.fill: thisNotificationBar
        //anchors.margins: thisNotificationBar._CHIGHLIGHT_BORDER_WIDTH
        //width:thisNotificationBar.width
        height:thisProgressBar.height

        anchors.left: thisProgressBar.left
        anchors.verticalCenter: thisProgressBar.verticalCenter

        spacing: thisProgressBar._CITEM_SPACING
        // layoutDirection: Qt.LeftToRight
        //clip: true

        property real _commonSubItemHeight: (thisProgressBar.height - 2*thisProgressBar._CHIGHLIGHT_BORDER_WIDTH)*9/10
        // MESSAGE TYPE ==============================================================================================================
        // [_messageType]
        //
        Rectangle {
            z :1
            id:msgType
            color : "#0AB1D1"
            anchors.verticalCenter: parent.verticalCenter

            height: mainRow.height
            width : thisProgressBar.width/54

            //border.width: 1
            //border.color: "white"
        }

        // MESSAGE ICON ==============================================================================================================
        // [_messageIconSource]
        //
        Rectangle {
            id: msgIcon
            color : GEORC.COLOR.TRANSPARENT

            anchors.verticalCenter: parent.verticalCenter
            height: mainRow._commonSubItemHeight
            width : GEOUTIL.TEXT.isEmpty(thisProgressBar._progressIconSource) ? 0: thisProgressBar.width/18

            //border.width: 1
            //border.color: "white"

            //clip:true

            K3DImage {
                id: msgIconImage
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                height: parent.height*3/5
                width : height
                source: thisProgressBar._progressIconSource
            }

            K3DText {
                id: msgIconText
                anchors.bottom: parent.bottom
                anchors.bottomMargin:  parent.height/6
                anchors.horizontalCenter: parent.horizontalCenter
                height: parent.height/5
                text  : thisProgressBar._progressIconText
                color : "white"
            }
        }

        // MESSAGE TEXT ==============================================================================================================
        // [_messageText]
        //
        Rectangle {
            id : msgTextProgressFrame
            z  : 1

            width : thisProgressBar.width - 3*thisProgressBar._CITEM_SPACING - msgType.width - msgIcon.width
            height: mainRow._commonSubItemHeight
            color : GEORC.COLOR.TRANSPARENT
            anchors.verticalCenter: parent.verticalCenter
//            border.width: 1
//            border.color: "white"

            property real _CPROGRESS_CIRCLE_SIZE: msgTextProgressFrame.height/5
            K3DText {
                id: msgText

                anchors {
                    top             : msgTextProgressFrame.top
                    topMargin       : msgTextProgressFrame.height/9
                    horizontalCenter: msgTextProgressFrame.horizontalCenter
                }

                text               : thisProgressBar._progressText
                width              : msgTextProgressFrame.width

                antialiasing       : true
                _fontSizeScale     : 1
                font.letterSpacing : 1
                color              : "white"
                style              : Text.Normal
                styleColor         : "white"
                wrapMode           : thisProgressBar.width === thisProgressBar._originalWidth? Text.WordWrap : Text.NoWrap
                verticalAlignment  : Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
            }

            Row {
                spacing            :  (msgTextProgressFrame.width - (2 + progressCircleGroup.model)* msgTextProgressFrame._CPROGRESS_CIRCLE_SIZE) /
                                      (progressCircleGroup.model-1)
                anchors {
                    bottom         : msgTextProgressFrame.bottom
                    left           : msgTextProgressFrame.left
                    right          : msgTextProgressFrame.right
                    margins        : msgTextProgressFrame._CPROGRESS_CIRCLE_SIZE
                }

                Repeater {
                    id: progressCircleGroup
                    model: 10

                    onItemAdded: {
                        item._index = index;
                    }

                    K3DRectangle {
                        id: progressCircle
                        z:2
                        width : msgTextProgressFrame._CPROGRESS_CIRCLE_SIZE
                        height: width
                        radius: width/2

                        property string _CSTYLE_BLACK  : "qrc:///res/progressbar/progressCircleBlack.svg"
                        property string _CSTYLE_BLUE   : "qrc:///res/progressbar/progressCircleBlue.svg"
                        property string _CSTYLE_ORANGE : "qrc:///res/progressbar/progressCircleBlue.svg"
                        _backgroundSource: _CSTYLE_BLACK
//                        color : GEORC.COLOR.TRANSPARENT
//                        border.width: 1
//                        border.color: color //"lightgray"

                        property int _index

                        // [PROGRESSING ANIM] --
                        property alias _progressingAnimation: progressingAnimation
                        PropertyAnimation {
                            id: progressingAnimation
                            loops  : 1
                            target : progressCircle; property :"_backgroundSource"
                            from   : progressCircle._CSTYLE_BLACK
                            to     : progressCircle._CSTYLE_BLUE; duration : 300
                            running: false
                            //easing.type: Easing.InOutElastic; easing.amplitude: 2.0; easing.period: 1.5

                            onStopped: {
                                if(progressCircle._index === progressCircleGroup.count-1) {
                                    for(var i = thisProgressBar._currentCircleIndex+1; i < progressCircleGroup.count; i++) {
                                        progressCircleGroup.itemAt(i)._backgroundSource = progressCircleGroup.itemAt(i)._CSTYLE_BLACK;
                                    }
                                    if(thisProgressBar._currentCircleIndex <= progressCircleGroup.count - 2)
                                        progressCircleGroup.itemAt(thisProgressBar._currentCircleIndex + 1)._progressingAnimation.start();
                                }
                                else {
                                    progressCircleGroup.itemAt(progressCircle._index+1)._progressingAnimation.start();
                                }
                            }
                        }


                        /*
                        Rectangle {
                            id: surroundingCircle
                            z       : -1
                            width   : msgTextFrame._CPROGRESS_CIRCLE_SIZE*2
                            height  : width
                            radius  : width/2
                            opacity : 1

                            anchors.centerIn: progressCircle

                            border.width: 1
                            border.color: "lightgray"

                            gradient: Gradient {
                                GradientStop { id: gradient1; position: 0.0 ; color: progressCircle.color }
                                GradientStop { id: gradient2; position: 1.0 ; color: "#92D1D1"            }
                            }

                            NumberAnimation on width {
                                duration : 300
                                from     : 0
                                to       : 15 // msgTextFrame._CPROGRESS_CIRCLE_SIZE*2
                                running  : true
                                loops    : Animation.Infinite
                            }
                        }
                        */
                    }
                } // End Repeater
            }
        }

    } // THE BIG ROW

    // ---------------------------------------------------------------------------------------------------------------------
    // -- STATES
    //http://qt-project.org/doc/qt-5/qtquick-statesanimations-animations.html
    //http://qt-project.org/doc/qt-5/qtquick-usecase-animations.html
    //http://qt-project.org/doc/qt-5/qml-qtquick-scriptaction.html
    //http://qt-project.org/doc/qt-4.8/qml-propertyanimation.html#details
    //http://qt-project.org/doc/qt-4.8/qdeclarativeanimation.html
    states: [
                State {
                    name: GEORC.STATE.HIDDEN
                    PropertyChanges {
                        target  : thisProgressBar
                        width   : 0
                        opacity : 0
                    }
                    StateChangeScript {
                        name: "'HiddenScript"
                        script: {
                            for(var i = 0; i < progressCircleGroup.count; i++) {
                                progressCircleGroup.itemAt(i)._backgroundSource = progressCircleGroup.itemAt(i)._CSTYLE_BLACK;
                            }
                        }
                    }
                },
                State {
                    name: GEORC.STATE.PROGRESSING

                    PropertyChanges {
                        target  : thisProgressBar
                        width   : _originalWidth
                        opacity : 1
                    }

                    StateChangeScript {
                        name: "ProgressingScript"
                        script: {
                        }
                    }
                }
            ]
    // End STATES

    // -- TRANSITIONS
    transitions:[    // NumberAnimation inherits PropertyAnimation
                     Transition {
                         from : GEORC.STATE.HIDDEN
                         to   : GEORC.STATE.PROGRESSING
                         //NumberAnimation   { properties: "width";   duration:700}
                         //NumberAnimation   { properties: "opacity"; duration:700}
                         ScriptAction      { scriptName: "ProgressingScript" }
                     },
                     Transition {
                         //from: GEORC.STATE.PROGRESSING
                         to   : GEORC.STATE.HIDDEN
                         NumberAnimation   { properties: "width";   duration:700}
                         NumberAnimation   { properties: "opacity"; duration:700}
                     }
                ]
    // End TRANSITIONS
}
