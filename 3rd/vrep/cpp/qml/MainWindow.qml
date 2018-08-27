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
import Qt3D.Render 2.0
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


Item {
    id: gbMainWindow
    width: 1280
    height: 768
    visible: true

    property alias _robotCar: rbScene._autoCar

    function setUltraSonicSensorArrowVisible(arrowId, visible) {
        rbScene.setArrowVisible(arrowId, visible);
    }

    // MAIN ROBOT SCENE --
    //
    RobotScene {
        id: rbScene
        anchors.fill: parent
    }

    // FRONT CAMERA IMAGE --
    //
    property string _frontCamImgSource: gbMainWindow.getFloorCamImageSource()
    function getFrontCamImage() {
        return "image://frontVisionSensorImage" + _rbMainWindowAgent.getFrontVisionSensorImageId()
    }

    VImage {
        id: frontCamImg
        anchors.right: parent.right
        anchors.top: parent.top
        height: parent.height/4
        width: height
        source: gbMainWindow._frontCamImgSource
    }

    // FLOOR CAMERA IMAGE --
    //
    property string _floorCamImgSource: gbMainWindow.getFloorCamImageSource()
    function getFloorCamImageSource() {
        return "image://floorVisionSensorImage" + _rbMainWindowAgent.getFloorVisionSensorImageId()
    }

    VImage {
        id: floorCamImg
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        height: parent.height/4
        width: height
        source: gbMainWindow._floorCamImgSource
    }

    Component.onCompleted: {
        //var rotateQuaternion = rbScene._camera.rotation(90, RBRC.AXIS.Y); // Qt.quaternion()
        //rbScene._camera.rotate(rotateQuaternion);
    }

    //MouseArea {
    //    anchors.fill: parent
    //    acceptedButtons: Qt.LeftButton
    //}
}
