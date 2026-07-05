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
    Layout.fillHeight: false
    implicitHeight: Math.max(90, contentColumn.implicitHeight + 28)

    Column {
        id: contentColumn
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 14
        spacing: 8

        Text {
            width: parent.width
            text: root.title
            font.pixelSize: 16
            font.bold: true
            color: appPalette.textPrimary
            wrapMode: Text.WrapAnywhere
        }

        Text {
            width: parent.width
            text: root.value
            font.pixelSize: 16
            color: appPalette.cardValue
            wrapMode: Text.WrapAnywhere
        }
    }
}
