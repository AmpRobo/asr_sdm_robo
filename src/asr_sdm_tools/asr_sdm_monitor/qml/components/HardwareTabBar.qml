import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../utils/I18n.js" as I18n

Item {
    id: root
    property var appPalette
    property string language: "en"
    property int currentIndex: 0
    signal tabSelected(int index)

    implicitHeight: 56

    readonly property var tabKeys: ["cpu", "memory", "hdd", "net", "ntp"]

    RowLayout {
        anchors.fill: parent
        spacing: 12

        Repeater {
            model: root.tabKeys
            delegate: SelectableButton {
                label: I18n.t(root.language, modelData)
                selected: root.currentIndex === index
                appPalette: root.appPalette
                Layout.preferredWidth: 140
                Layout.fillWidth: false
                implicitHeight: 46
                onClicked: root.tabSelected(index)
            }
        }

        Item { Layout.fillWidth: true }
    }
}
