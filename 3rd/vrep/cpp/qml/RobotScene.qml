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
import QtQuick.Window 2.3
import QtQuick.Scene3D 2.0

import Qt3D.Core 2.0
import Qt3D.Render 2.0 // Camera QML 3D - https://doc.qt.io/qt-5/qml-qt3d-render-camera.html
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

import QtQuick.Controls 1.4        // Provide Qt control widgets
import QtQuick.Controls.Styles 1.4 // Provide Qt control widget styles
import QtQuick.Layouts  1.3        // Provide Qt layouts
import QtQuick.Dialogs  1.2        // StandardButton

import com.rb.qmladapter 1.0

//import "."  // Directory containing 'qmldir' file
import MainGBSingletonObject 1.0 // MAINGB

import "qrc:///javascript/qmlcommonresource.js" as RBRC


//! [0]
Scene3D {
    id: sceneRoot
    anchors.fill: parent
    aspects: ["render", "logic", "input"]

    property alias _autoCar: autoCar
    property alias _camera: camera

    function setArrowVisible(arrowId, visible) {
        if(arrow1._index === arrowId)       arrow1._visible  = visible;
        else if(arrow2._index === arrowId)  arrow2._visible  = visible;
        else if(arrow3._index === arrowId)  arrow3._visible  = visible;
        else if(arrow4._index === arrowId)  arrow4._visible  = visible;
        else if(arrow5._index === arrowId)  arrow5._visible  = visible;
        else if(arrow6._index === arrowId)  arrow6._visible  = visible;
        else if(arrow7._index === arrowId)  arrow7._visible  = visible;
        else if(arrow8._index === arrowId)  arrow8._visible  = visible;
        else if(arrow9._index === arrowId)  arrow9._visible  = visible;
        else if(arrow10._index === arrowId) arrow10._visible = visible;
        else if(arrow11._index === arrowId) arrow11._visible = visible;
        else if(arrow12._index === arrowId) arrow12._visible = visible;
        else if(arrow13._index === arrowId) arrow13._visible = visible;
        else if(arrow14._index === arrowId) arrow14._visible = visible;
        else if(arrow15._index === arrowId) arrow15._visible = visible;
        else if(arrow16._index === arrowId) arrow16._visible = visible;
    }

    Entity {
        id: sceneEntity

        Camera {
            id: camera
            projectionType: CameraLens.PerspectiveProjection
            fieldOfView: 70
            aspectRatio: Screen.width / Screen.height
            nearPlane : 0.1
            farPlane  : 1000.0
            position  : Qt.vector3d(15.0, 15.0, 0.0)
            viewCenter: RBRC.COOR_ORIGIN
            upVector  : RBRC.AXIS.Y
        }

        FirstPersonCameraController { camera: camera }

        ShadowMapLight {
            id: light
        }

        components: [
            ShadowMapFrameGraph {
                id: framegraph
                viewCamera: camera
                lightCamera: light.lightCamera
            },
            // Event Source will be set by the Qt3DQuickWindow
            InputSettings { }
        ]


        AdsEffect {
            id: shadowMapEffect

            shadowTexture: framegraph.shadowTexture
            light: light
        }

        AdsMaterial {
            id: arrowMaterial
            effect: shadowMapEffect
            diffuseColor: RBRC.COLOR.GREEN
            shininess: 75
        }

        // Toyplane entity
        RobotCar {
            id: autoCar
            material: AdsMaterial {
                effect: shadowMapEffect
                diffuseColor: _autoCar._color
                shininess: 75
            }
        }

        Arrow {
            id: arrow1
            _index: 0
            material: arrowMaterial
        }

        Arrow {
            id: arrow2
            _index: 1
            material: arrowMaterial
        }

        Arrow {
            id: arrow3
            _index: 2
            material: arrowMaterial
        }

        Arrow {
            id: arrow4
            _index: 3
            material: arrowMaterial
        }

        Arrow {
            id: arrow5
            _index: 4
            material: arrowMaterial
        }

        Arrow {
            id: arrow6
            _index: 5
            material: arrowMaterial
        }

        Arrow {
            id: arrow7
            _index: 6
            material: arrowMaterial
        }

        Arrow {
            id: arrow8
            _index: 7
            material: arrowMaterial
        }

        Arrow {
            id: arrow9
            _index: 8
            material: arrowMaterial
        }

        Arrow {
            id: arrow10
            _index: 9
            material: arrowMaterial
        }

        Arrow {
            id: arrow11
            _index: 10
            material: arrowMaterial
        }

        Arrow {
            id: arrow12
            _index: 11
            material: arrowMaterial
        }

        Arrow {
            id: arrow13
            _index: 12
            material: arrowMaterial
        }

        Arrow {
            id: arrow14
            _index: 13
            material: arrowMaterial
        }

        Arrow {
            id: arrow15
            _index: 14
            material: arrowMaterial
        }

        Arrow {
            id: arrow16
            _index: 15
            material: arrowMaterial
        }

        // Plane entity
        GroundPlane {
            material: AdsMaterial {
                effect: shadowMapEffect
                diffuseColor: RBRC.COLOR.GRAY
                specularColor: Qt.rgba(0, 0, 0, 1.0)
            }
        }
    }
}
