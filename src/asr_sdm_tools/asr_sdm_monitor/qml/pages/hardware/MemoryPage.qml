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
    contentHeight: memoryContent.implicitHeight

    ColumnLayout {
        id: memoryContent
        width: root.width
        spacing: 16

        SectionTitle { text: I18n.t(root.language, "memory"); appPalette: root.appPalette }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "usedPhysical"); value: UiHelpers.display(RosUi.memorySummary.usedPhysical); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "totalPhysical"); value: UiHelpers.display(RosUi.memorySummary.totalPhysical); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "freePhysical"); value: UiHelpers.display(RosUi.memorySummary.freePhysical); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "usagePercent"); value: UiHelpers.display(RosUi.memorySummary.usagePercent); appPalette: root.appPalette }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "usedSwap"); value: UiHelpers.display(RosUi.memorySummary.usedSwap); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "totalSwap"); value: UiHelpers.display(RosUi.memorySummary.totalSwap); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "updateStatus"); value: UiHelpers.display(RosUi.memorySummary.updateStatus); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "state"); value: UiHelpers.display(RosUi.memorySummary.level) + " / " + UiHelpers.display(RosUi.memorySummary.state); appPalette: root.appPalette }
        }

        AreaLineChart {
            appPalette: root.appPalette
            primaryLabel: I18n.t(root.language, "memoryHistory")
            primaryData: RosUi.memoryHistory
            maxY: 1.0
            scaleText: I18n.t(root.language, "percentScale")
        }

        SectionTitle { text: I18n.t(root.language, "memoryDetails"); appPalette: root.appPalette }

        SimpleTable {
            appPalette: root.appPalette
            columns: [
                { label: I18n.t(root.language, "type"), key: "item", width: 150 },
                { label: I18n.t(root.language, "total"), key: "total", width: 180 },
                { label: I18n.t(root.language, "used"), key: "used", width: 180 },
                { label: I18n.t(root.language, "free"), key: "free", fill: true }
            ]
            rows: RosUi.memoryRows
        }
    }
}
