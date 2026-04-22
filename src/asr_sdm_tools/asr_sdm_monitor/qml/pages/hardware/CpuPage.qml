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
    contentHeight: cpuContent.implicitHeight

    ColumnLayout {
        id: cpuContent
        width: root.width
        spacing: 16

        SectionTitle { text: I18n.t(root.language, "cpu"); appPalette: root.appPalette }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "avgUsage"); value: UiHelpers.display(RosUi.cpuSummary.avgUsage); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "maxCoreUsage"); value: UiHelpers.display(RosUi.cpuSummary.maxUsage); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "avgClock"); value: UiHelpers.display(RosUi.cpuSummary.avgClock); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "load1"); value: UiHelpers.display(RosUi.cpuSummary.load1); appPalette: root.appPalette }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "load5"); value: UiHelpers.display(RosUi.cpuSummary.load5); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "load15"); value: UiHelpers.display(RosUi.cpuSummary.load15); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "coreCount"); value: UiHelpers.display(RosUi.cpuSummary.coreCount); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "state"); value: UiHelpers.display(RosUi.cpuSummary.level) + " / " + UiHelpers.display(RosUi.cpuSummary.state); appPalette: root.appPalette }
        }

        AreaLineChart {
            appPalette: root.appPalette
            primaryLabel: I18n.t(root.language, "cpuHistory")
            primaryData: RosUi.cpuHistory
            maxY: 1.0
            scaleText: I18n.t(root.language, "percentScale")
        }

        SectionTitle { text: I18n.t(root.language, "coreDetails"); appPalette: root.appPalette }

        SimpleTable {
            appPalette: root.appPalette
            columns: [
                { label: I18n.t(root.language, "core"), key: "core", width: 90 },
                { label: I18n.t(root.language, "usage"), key: "usage", width: 120 },
                { label: I18n.t(root.language, "clock"), key: "clock", width: 140 },
                { label: I18n.t(root.language, "user"), key: "user", width: 120 },
                { label: I18n.t(root.language, "system"), key: "system", width: 120 },
                { label: I18n.t(root.language, "idle"), key: "idle", width: 120 },
                { label: I18n.t(root.language, "status"), key: "status", fill: true }
            ]
            rows: RosUi.cpuCoreRows
        }
    }
}
