import QtQuick
import QtQuick.Layouts
import RosUi 1.0
import "../../components"
import "../../utils/I18n.js" as I18n
import "../../utils/UiHelpers.js" as UiHelpers

Flickable {
    id: root
    property var appPalette
    property string language: "en"

    clip: true
    contentWidth: width
    contentHeight: hddContent.implicitHeight

    ColumnLayout {
        id: hddContent
        width: root.width
        spacing: 16

        SectionTitle { text: I18n.t(root.language, "hdd"); appPalette: root.appPalette }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "diskCount"); value: UiHelpers.display(RosUi.hddSummary.diskCount); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "worstUse"); value: UiHelpers.display(RosUi.hddSummary.worstUse); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "stateLevel"); value: UiHelpers.display(RosUi.hddSummary.level); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "stateDescription"); value: UiHelpers.display(RosUi.hddSummary.state); appPalette: root.appPalette }
        }

        SectionTitle { text: I18n.t(root.language, "diskDetails"); appPalette: root.appPalette }

        SimpleTable {
            appPalette: root.appPalette
            columns: [
                { label: I18n.t(root.language, "disk"), key: "disk", width: 260 },
                { label: I18n.t(root.language, "mount"), key: "mount", width: 140 },
                { label: I18n.t(root.language, "size"), key: "size", width: 120 },
                { label: I18n.t(root.language, "available"), key: "available", width: 140 },
                { label: I18n.t(root.language, "use"), key: "use", width: 100 },
                { label: I18n.t(root.language, "status"), key: "status", fill: true }
            ]
            rows: RosUi.hddRows
        }
    }
}
