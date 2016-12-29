import QtQuick 2.5

import "qrc:///javascript/qmlcommonresource.js" as K3DRC
// http://qt-project.org/doc/qt-5/qml-qtquick-path.html

ParallelAnimation {
    id: thisAnimation

    property int _movingTargetX: 0
    property int _movingSourceX: 0
    property int _movingTargetY: 0
    property int _movingSourceY: 0

    property int _radius

    property Item _target : parent

    property real _duration : 1000

    running: false
    //loops: 1 (-1 mean Infinite)

    function startAnimating(isOnWard) {
        if(isOnWard) {
            animationPath.startX = thisAnimation._movingSourceX; animationPath.startY = thisAnimation._movingSourceY;
            animationPathArc.x   = thisAnimation._movingTargetX; animationPathArc.y   = thisAnimation._movingTargetY;
        }
        else {
            animationPath.startX = thisAnimation._movingTargetX; animationPath.startY = thisAnimation._movingTargetY;
            animationPathArc.x   = thisAnimation._movingSourceX; animationPathArc.y   = thisAnimation._movingSourceY;
        }

        thisAnimation.start();
    }

    //PauseAnimation { duration: 1000 }
    NumberAnimation   { target: thisAnimation._target; properties: "opacity"; from:1; to: 0; duration:thisAnimation._duration}

    PathAnimation {
        duration: thisAnimation._duration
        //easing.type: Easing.InQuad

        target: thisAnimation._target
        //orientation: PathAnimation.Fixed
        anchorPoint: Qt.point(thisAnimation._target.width/2, thisAnimation._target.height/2)
        path: Path {
            id: animationPath
            startX: thisAnimation._movingSourceX; startY: thisAnimation._movingSourceY

            PathArc {
                id:animationPathArc

                x: thisAnimation._movingTargetX; y: thisAnimation._movingTargetY
                radiusX: thisAnimation._radius; radiusY: thisAnimation._radius
                useLargeArc: true
                direction: PathArc.Counterclockwise
            }
            onChanged: {}
        }
    }


}
/*

ParallelAnimation {
    property int rotation: 360
    property int length: rotation * 2
    SequentialAnimation {
        RotateTransition {
            toAngleZ: rotation
            duration: 1000
        }
        RotateTransition {
            toAngleZ: 47
            duration: 1000
        }
    }
    SequentialAnimation {
        TranslateTransition {
            toX: length
            duration: 1000
        }
        TranslateTransition {
            toX: 0
            duration: 1000
        }
    }
}

PathAnimation {
    path: Path {
        //no startX, startY
        PathCurve { x: 100; y: 100}
        PathCurve {}    //last element is empty with no end point specified
    }
}

    PathAnimation {
        id: thisMarkMenuControlAnimation
        duration: 3000
        loops: Animation.Infinite
        easing.type: Easing.InQuad
        running: false // thisMarkMenuControl._isFreeDrifting

        target: thisMarkMenuControl
        orientation: PathAnimation.Fixed // Default
        anchorPoint: Qt.point(thisMarkMenuControl.width/2,thisMarkMenuControl.height/2)
        path: Path {
            id: animationPath
            startX: thisMarkMenuControl._cx; startY: thisMarkMenuControl._cy;

            PathArc {
                relativeX: thisMarkMenuControl.width; y: animationPath.startY
                radiusX: 25; radiusY: 15
            }

            PathArc {
                relativeX: 50; y: animationPath.startY
                radiusX: 25; radiusY: 25
            }
            PathArc {
                relativeX: 50; y: animationPath.startY
                radiusX: 25; radiusY: 50
            }
            PathArc {
                relativeX: 50; y: animationPath.startY
                radiusX: 50; radiusY: 100
            }
            PathArc {
                id:animationPathArc

                x: thisAnimation._movingTargetX; y: thisAnimation._movingTargetY
                radiusX: thisAnimation._radius; radiusY: thisAnimation._radius
                useLargeArc: true
                direction: PathArc.Counterclockwise
            }
            //onChanged: {}
        }
    }

*/
