import QtQuick
import QtQuick.Layouts

Item {
    id: root
    property string text: ""
    property var appPalette
    implicitHeight: 32
    Layout.fillWidth: true

    Text {
        anchors.verticalCenter: parent.verticalCenter
        text: root.text
        font.pixelSize: 18
        font.bold: true
        color: appPalette.textPrimary
    }
}
