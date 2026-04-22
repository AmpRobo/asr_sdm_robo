import QtQuick
import QtQuick.Layouts
import "../utils/UiHelpers.js" as UiHelpers

Rectangle {
    id: root
    property var appPalette
    property var columns: []
    property var rows: []

    radius: 12
    color: appPalette.surfaceBackground
    border.color: appPalette.border
    border.width: 1
    Layout.fillWidth: true
    implicitHeight: 320

    Flickable {
        anchors.fill: parent
        anchors.margins: 1
        clip: true
        contentWidth: width
        contentHeight: tableColumn.implicitHeight

        Column {
            id: tableColumn
            width: parent.width
            spacing: 0

            Rectangle {
                width: parent.width
                height: 48
                color: appPalette.headerBackground

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 12
                    anchors.rightMargin: 12
                    spacing: 8

                    Repeater {
                        model: root.columns
                        delegate: Text {
                            required property var modelData
                            Layout.preferredWidth: modelData.width
                            Layout.fillWidth: modelData.fill === true
                            text: modelData.label
                            font.pixelSize: 16
                            font.bold: true
                            color: appPalette.textPrimary
                            elide: Text.ElideRight
                        }
                    }
                }
            }

            Repeater {
                model: root.rows
                delegate: Rectangle {
                    id: rowDelegate
                    required property int index
                    required property var modelData
                    property var rowData: modelData
                    width: parent.width
                    height: 44
                    color: index % 2 === 0 ? appPalette.surfaceBackground : appPalette.rowAlternateBackground
                    border.color: appPalette.border
                    border.width: 1

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 12
                        anchors.rightMargin: 12
                        spacing: 8

                        Repeater {
                            model: root.columns
                            delegate: Text {
                                required property var modelData
                                Layout.preferredWidth: modelData.width
                                Layout.fillWidth: modelData.fill === true
                                text: UiHelpers.display(rowDelegate.rowData[modelData.key])
                                font.pixelSize: 15
                                color: appPalette.textSecondary
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
            }
        }
    }
}
