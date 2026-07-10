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
    readonly property real netMaxY: UiHelpers.netAdaptiveMax(RosUi.netInHistory, RosUi.netOutHistory)

    clip: true
    contentWidth: width
    contentHeight: netContent.implicitHeight

    ColumnLayout {
        id: netContent
        width: root.width
        spacing: 16

        SectionTitle { text: I18n.t(root.language, "net"); appPalette: root.appPalette }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "totalInput"); value: UiHelpers.display(RosUi.netSummary.input); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "totalOutput"); value: UiHelpers.display(RosUi.netSummary.output); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "interfaceCount"); value: UiHelpers.display(RosUi.netSummary.interfaceCount); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "errorCount"); value: UiHelpers.display(RosUi.netSummary.errors); appPalette: root.appPalette }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { Layout.fillHeight: true; title: I18n.t(root.language, "interfaceList"); value: UiHelpers.display(RosUi.netSummary.interfaces); appPalette: root.appPalette }
            ValueCard { Layout.fillHeight: true; title: I18n.t(root.language, "ipAddressList"); value: UiHelpers.display(RosUi.netSummary.ipAddresses); appPalette: root.appPalette }
            ValueCard { Layout.fillHeight: true; title: I18n.t(root.language, "stateLevel"); value: UiHelpers.display(RosUi.netSummary.level); appPalette: root.appPalette }
            ValueCard { Layout.fillHeight: true; title: I18n.t(root.language, "stateDescription"); value: UiHelpers.display(RosUi.netSummary.state); appPalette: root.appPalette }
        }

        AreaLineChart {
            appPalette: root.appPalette
            primaryLabel: I18n.t(root.language, "inputHistory")
            secondaryLabel: I18n.t(root.language, "outputHistory")
            primaryData: RosUi.netInHistory
            secondaryData: RosUi.netOutHistory
            maxY: root.netMaxY
            scaleText: I18n.formatNetScale(root.language, root.netMaxY)
        }

        SectionTitle { text: I18n.t(root.language, "interfaceDetails"); appPalette: root.appPalette }

        SimpleTable {
            appPalette: root.appPalette
            columns: [
                { label: I18n.t(root.language, "interface"), key: "interface", width: 130 },
                { label: I18n.t(root.language, "ipAddress"), key: "ip", width: 150 },
                { label: I18n.t(root.language, "state"), key: "state", width: 90 },
                { label: I18n.t(root.language, "input"), key: "input", width: 150 },
                { label: I18n.t(root.language, "output"), key: "output", width: 150 },
                { label: I18n.t(root.language, "rxErr"), key: "rxErrors", width: 90 },
                { label: I18n.t(root.language, "txErr"), key: "txErrors", width: 90 },
                { label: I18n.t(root.language, "totalRx"), key: "totalRx", width: 130 },
                { label: I18n.t(root.language, "totalTx"), key: "totalTx", fill: true }
            ]
            rows: RosUi.netInterfaceRows
        }
    }
}
