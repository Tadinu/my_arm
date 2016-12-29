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

/*@!Note:
  - Use this class for SVG Image!
  - Set _sourceWidth & _sourceHeight directly! NOT width/height!
*/

Image {
    id: thisImage
    //z: 2

    // Do this to RASTERIZE THE .SVG IMAGE
    // Avoid changing this property dynamically; rendering an SVG
    // is slow compared to an image.
    property real _sourceWidth  : width//: thisImage.sourceSize.width
    property real _sourceHeight : height//: thisImage.sourceSize.height

    property real _imgOriginX      : 0
    property real _imgOriginY      : 0
    property real _imgRotationVectorX : 0
    property real _imgRotationVectorY : 0
    property real _imgRotationVectorZ : 0
    property real _imgRotationAngle   : 0

    property real _origSourceWidth  : 1
    property real _origSourceHeight : 1
    transform: Rotation {
        id : imageRotation
        origin.x : _imgOriginX
        origin.y : _imgOriginY
        axis {
            x : _imgRotationVectorX
            y : _imgRotationVectorY
            z : _imgRotationVectorZ
        }
        angle : _imgRotationAngle
    }
    onStatusChanged : {
        if (thisImage.status === Image.Ready) {
            _origSourceWidth  = sourceSize.width;
            _origSourceHeight = sourceSize.height;
        }
    }
    // !NOTE:
    // + DON'T SET sourceSize.width TO sourceSize.height or vice versa!
    sourceSize.width  : _sourceWidth
    sourceSize.height : _sourceHeight
    smooth            : true
    antialiasing      : true
    mipmap            : true
}
