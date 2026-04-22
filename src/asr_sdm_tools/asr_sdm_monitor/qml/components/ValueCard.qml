import QtQuick
import QtQuick.Layouts

Rectangle {
    id: root
    property string title: ""
    property string value: "--"
    property var appPalette

    radius: 12
    color: appPalette.cardBackground
    border.color: appPalette.border
    border.width: 1
    Layout.fillWidth: true
    implicitHeight: 90

    Column {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 8

        Text {
            text: root.title
            font.pixelSize: 16
            font.bold: true
            color: appPalette.textPrimary
        }

        Text {
            text: root.value
            font.pixelSize: 16
            color: appPalette.cardValue
            wrapMode: Text.WrapAnywhere
        }
    }
}
