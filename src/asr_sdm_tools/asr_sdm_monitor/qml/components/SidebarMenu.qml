import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../utils/I18n.js" as I18n

Rectangle {
    id: root
    property var appPalette
    property string language: "en"
    property int currentSection: 0
    property string rosStatus: "--"
    signal sectionSelected(int index)

    color: appPalette.sidebarBackground
    border.color: appPalette.border
    border.width: 1

    Column {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 14

        Text {
            text: I18n.t(root.language, "diagnostics")
            font.pixelSize: 18
            font.bold: true
            color: appPalette.textPrimary
        }

        Text {
            text: I18n.t(root.language, "topicPrefix") + " /diagnostics"
            font.pixelSize: 16
            color: appPalette.textSecondary
            wrapMode: Text.WrapAnywhere
        }

        Text {
            text: I18n.t(root.language, "statusPrefix") + " " + root.rosStatus
            font.pixelSize: 16
            color: appPalette.textSecondary
            wrapMode: Text.WrapAnywhere
        }

        Item { width: 1; height: 10 }

        SelectableButton {
            label: I18n.t(root.language, "hardware")
            selected: root.currentSection === 0
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: 56
            onClicked: root.sectionSelected(0)
        }

        SelectableButton {
            label: I18n.t(root.language, "video")
            selected: root.currentSection === 1
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: 56
            onClicked: root.sectionSelected(1)
        }
    }
}
