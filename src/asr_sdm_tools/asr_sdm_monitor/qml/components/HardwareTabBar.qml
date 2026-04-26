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
        id: tabLayout
        anchors.fill: parent
        spacing: root.width < 520 ? 6 : 12

        Repeater {
            model: root.tabKeys
            delegate: SelectableButton {
                label: I18n.t(root.language, modelData)
                selected: root.currentIndex === index
                appPalette: root.appPalette
                Layout.fillWidth: true
                Layout.minimumWidth: 64
                Layout.preferredWidth: 140
                implicitHeight: 46
                onClicked: root.tabSelected(index)
            }
        }
    }
}
