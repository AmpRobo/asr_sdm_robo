import QtQuick
import QtQuick.Controls

Rectangle {
    id: root
    property string label: ""
    property bool selected: false
    property var appPalette
    property int radiusValue: 10
    signal clicked

    radius: radiusValue
    color: selected ? appPalette.accent : appPalette.controlBackground
    border.color: selected ? appPalette.accentBorder : appPalette.controlBorder
    border.width: 1
    implicitHeight: 44
    implicitWidth: 140

    Text {
        anchors.centerIn: parent
        text: root.label
        font.pixelSize: 16
        font.bold: root.selected
        color: root.selected ? appPalette.accentText : appPalette.textPrimary
    }

    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: root.clicked()
    }
}
