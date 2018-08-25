// MAINGB
//
import QtQuick 2.0
import QtQuick.Window 2.2


import "qrc:///javascript/qmlcommonresource.js" as RBRC
//
import com.rb.qmladapter 1.0

pragma Singleton
// !NOTE:
// THE PROPERTY NAME OF THIS CLASS MUST BE STARTED WITH _ OR LOW CHARACTER.
// NAME LIKE "CABC" IS NOT ACCEPTABLE!
//

QtObject {
    id : thisMainGB
    property var  _QML_ADAPTER : null
    function setMainQMLAdapter(qmladapter) {
        _QML_ADAPTER = qmladapter;
    }
}
