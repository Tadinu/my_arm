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

/*!
    \qmltype ProgressBar
    \inqmlmodule UIComponents 1.0
    \brief A component that shows the progress of an event

    A ProgressBar shows the linear progress of an event as its \l value.
    The range is specified using the \l {minimumValue} and the \l{maximumValue} values.

    The ProgressBar component is part of the \l {UI Components} module.

    This documentation is part of the \l{componentset}{UIComponents} example.
*/
K3DRectangle {
    id: progressbar

    /*!
        The minimumValue value of the ProgressBar range.
        The \l value must not be less than this value.
    */
    property int minimumValue: 0

    /*!
        The maximumValue value of the ProgressBar range.
        The \l value must not be more than this value.
    */
    property int maximumValue: 100

    /*!
        The value of the progress.
    */
    property int value: 0

    /*!
       \qmlproperty color ProgressBar::color
       The color of the ProgressBar's gradient. Must bind to a color type.

       \omit
           The "\qmlproperty <type> <property name>" is needed because
           property alias need to have their types manually entered.

           QDoc will not publish the documentation within omit and endomit.
       \endomit

       \sa secondColor
    */
    property alias color: gradient1.color

    /*!
       \qmlproperty color ProgressBar::secondColor
       The second color of the ProgressBar's gradient.
       Must bind to a color type.

       \omit
           The "\qmlproperty <type> <property name>" is needed because
           property alias need to have their types manually entered.

           QDoc will not publish the documentation within omit and endomit.
       \endomit

        \sa color
    */
    property alias secondColor: gradient2.color

    width: 250; height: 23
    clip: true

    Rectangle {
        id: highlight
        antialiasing: true
        smooth : true
        /*!
            An internal documentation comment. The widthDest property is not
            a public API and therefore will not be exposed.
        */
        property int widthDest: ((progressbar.width * (value - minimumValue)) / (maximumValue - minimumValue) - 6)

        width: highlight.widthDest
        Behavior on width { SmoothedAnimation { velocity: 1200 } }

        anchors { left: parent.left; top: parent.top; bottom: parent.bottom; margins: 3 }
        radius: 1
        gradient: Gradient {
            GradientStop { id: gradient1; position: 0.0 }
            GradientStop { id: gradient2; position: 1.0 }
        }

    }
    K3DText {
        //anchors { right: highlight.right; rightMargin: 6; verticalCenter: parent.verticalCenter }
        anchors { horizontalCenter: highlight.horizontalCenter; verticalCenter: parent.verticalCenter }
        color: "white"
        font.bold: true
        text: Math.floor((value - minimumValue) / (maximumValue - minimumValue) * 100) + '%'
    }
}
