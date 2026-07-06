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
    property bool collapsed: false
    signal sectionSelected(int index)
    signal collapseToggled()

    color: appPalette.sidebarBackground
    border.color: appPalette.border
    border.width: 1
    clip: true

    Column {
        anchors.fill: parent
        anchors.margins: root.collapsed ? 6 : Math.max(8, Math.min(20, root.width * 0.08))
        spacing: root.collapsed ? 8 : (root.width < 150 ? 8 : 14)

        Rectangle {
            width: parent.width
            height: root.collapsed ? 38 : 42
            radius: 10
            color: collapseMouse.containsMouse ? root.appPalette.inputBackground : root.appPalette.controlBackground
            border.color: root.appPalette.controlBorder
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: root.collapsed ? "›" : "‹"
                font.pixelSize: 24
                font.bold: true
                color: root.appPalette.textPrimary
            }

            MouseArea {
                id: collapseMouse
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: root.collapseToggled()
            }
        }

        SelectableButton {
            label: root.collapsed ? I18n.t(root.language, "hardwareShort") : I18n.t(root.language, "hardware")
            selected: root.currentSection === 0
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: root.collapsed ? 42 : (root.width < 150 ? 46 : 56)
            labelPixelSize: root.collapsed ? 14 : (root.width < 150 ? 13 : 16)
            onClicked: root.sectionSelected(0)
        }

        SelectableButton {
            label: root.collapsed ? I18n.t(root.language, "videoShort") : I18n.t(root.language, "video")
            selected: root.currentSection === 1
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: root.collapsed ? 42 : (root.width < 150 ? 46 : 56)
            labelPixelSize: root.collapsed ? 14 : (root.width < 150 ? 13 : 16)
            onClicked: root.sectionSelected(1)
        }

        SelectableButton {
            label: root.collapsed ? I18n.t(root.language, "plotShort") : I18n.t(root.language, "plot")
            selected: root.currentSection === 2
            appPalette: root.appPalette
            implicitWidth: parent.width
            implicitHeight: root.collapsed ? 42 : (root.width < 150 ? 46 : 56)
            labelPixelSize: root.collapsed ? 14 : (root.width < 150 ? 13 : 16)
            onClicked: root.sectionSelected(2)
        }
    }
}
