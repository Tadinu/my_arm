/****************************************************************************
**
** Copyright (C) 2014 Klaralvdalens Datakonsult AB (KDAB).
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt3D module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 2.9
import Qt3D.Core 2.0
import Qt3D.Render 2.0

import com.rb.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

import "qrc:///javascript/qmlcommonresource.js" as RBRC


Entity {
    id: thisCar
    property Material material
    property vector3d _orientAngles : Qt.vector3d(0,0,0)

    property string _color: _state === _CSTATE_ERROR           ? RBRC.COLOR.RED   :
                            _state === _CSTATE_INITIALIZED     ? RBRC.COLOR.WHITE :
                            _state === _CSTATE_IDLE            ? RBRC.COLOR.GREEN :
                            _state === _CSTATE_OPERATING       ? RBRC.COLOR.BLUE  :
                            _state === _CSTATE_COLLIDING       ? RBRC.COLOR.BROWN : RBRC.COLOR.WHITE

    property string _name
    property int _state              : _CSTATE_IDLE
    property int _CSTATE_ERROR       : -1
    property int _CSTATE_INITIALIZED : 0
    property int _CSTATE_IDLE        : 1
    property int _CSTATE_OPERATING   : 2
    property int _CSTATE_COLLIDING   : 3

    property var _itemAgent // Point to RobotAgent*
    function regItemAgent(itemAgent) {
        _itemAgent = itemAgent;
    }

    function getState() {
        return _state;
    }

    function setState(state) {
        _state = state;
    }

    function setPos(pos) {
    }

    function setOrientation(orientAngles) {
        _orientAngles = orientAngles;
        //print("ORIENTATION: ", _orientAngles.z);
    }

    Mesh {
        id: carMesh
        source: "qrc:///res/obj/robot_car.obj"
    }

    Transform {
        id: carTransform

        property real rollAngle  : 0
        property real pitchAngle : 15
        property real yawAngle   : thisCar._orientAngles.z * 180 / Math.PI

        property real altitude   : 5
        property real scaleFactor: 1

        //Behavior on rollAngle { SpringAnimation { spring: 2; damping: 0.2} }
        Behavior on yawAngle  { SpringAnimation { spring: 2; damping: 0.2} }

        matrix: {
            var m = Qt.matrix4x4();
            m.translate(Qt.vector3d(Math.sin(yawAngle * Math.PI / 180) * scaleFactor,
                                    altitude,
                                    Math.cos(yawAngle * Math.PI / 180) * scaleFactor));
            m.rotate(yawAngle, Qt.vector3d(0, 1, 0));
            //m.rotate(pitchAngle, Qt.vector3d(0, 0, 1));
            //m.rotate(rollAngle, Qt.vector3d(1, 0, 0));
            m.scale(1.0 / carTransform.scaleFactor);
            return m;
        }
    }

    NumberAnimation {
        target: carTransform

        running: false
        loops: Animation.Infinite

        property: "yawAngle"
        duration: 10000
        from: 0
        to: 360
    }

    // Altitude / Pitch animation
    SequentialAnimation {
        running: false
        loops: Animation.Infinite
        ParallelAnimation {
            SequentialAnimation {
                NumberAnimation { target: carTransform; property: "pitchAngle"; from: 0; to: 30; duration: 2000; easing.type: Easing.OutQuad }
                NumberAnimation { target: carTransform; property: "pitchAngle"; from: 30; to: 0; duration: 2000; easing.type: Easing.OutSine }
            }
            NumberAnimation { target: carTransform; property: "altitude"; to: 5; duration: 4000; easing.type: Easing.InOutCubic }
        }
        PauseAnimation { duration: 1500 }
        ParallelAnimation {
            SequentialAnimation {
                NumberAnimation { target: carTransform; property: "pitchAngle"; from: 0; to: -30; duration: 1000; easing.type: Easing.OutQuad }
                NumberAnimation { target: carTransform; property: "pitchAngle"; from: -30; to: 0; duration: 5000; easing.type: Easing.OutSine }
            }
            NumberAnimation { target: carTransform; property: "altitude"; to: 0; duration: 6000; easing.type: Easing.InOutCubic}
        }
        PauseAnimation { duration: 1500 }
    }

    // Roll Animation
    SequentialAnimation {
        running: false
        loops: Animation.Infinite
        NumberAnimation { target: carTransform; property: "rollAngle"; to: 360; duration: 1500; easing.type: Easing.InOutQuad }
        PauseAnimation { duration: 1000 }
        NumberAnimation { target: carTransform; property: "rollAngle"; from: 0; to: 30; duration: 1000; easing.type: Easing.OutQuart }
        PauseAnimation { duration: 1500 }
        NumberAnimation { target: carTransform; property: "rollAngle"; from: 30; to: -30; duration: 1000; easing.type: Easing.OutQuart }
        PauseAnimation { duration: 1500 }
        NumberAnimation { target: carTransform; property: "rollAngle"; from: -30; to: 0; duration: 750; easing.type: Easing.OutQuart }
        PauseAnimation { duration: 2000 }
    }

    components: [
        carMesh,
        carTransform,
        material
    ]
}
