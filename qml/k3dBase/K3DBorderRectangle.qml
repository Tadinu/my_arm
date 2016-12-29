import QtQuick 2.3

K3DRectangle
{
    property bool _commonBorder : false

    property int _commonBorderWidth : 1

    property int _lBorderwidth : 1
    property int _rBorderwidth : 1
    property int _tBorderwidth : 1
    property int _bBorderwidth : 1

    //z : -1

    property string _borderColor : "white"
    border.color: _borderColor

    anchors
    {
        left: parent.left
        right: parent.right
        top: parent.top
        bottom: parent.bottom

        topMargin    : _commonBorder ? -_commonBorderWidth : -_tBorderwidth
        bottomMargin : _commonBorder ? -_commonBorderWidth : -_bBorderwidth
        leftMargin   : _commonBorder ? -_commonBorderWidth : -_lBorderwidth
        rightMargin  : _commonBorder ? -_commonBorderWidth : -_rBorderwidth
    }
}
