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
    contentHeight: ntpContent.implicitHeight

    ColumnLayout {
        id: ntpContent
        width: root.width
        spacing: 16

        SectionTitle { text: I18n.t(root.language, "ntp"); appPalette: root.appPalette }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            ValueCard { title: I18n.t(root.language, "offset"); value: UiHelpers.display(RosUi.ntpSummary.offset); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "tolerance"); value: UiHelpers.display(RosUi.ntpSummary.tolerance); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "errorTolerance"); value: UiHelpers.display(RosUi.ntpSummary.errorTolerance); appPalette: root.appPalette }
            ValueCard { title: I18n.t(root.language, "state"); value: UiHelpers.display(RosUi.ntpSummary.level) + " / " + UiHelpers.display(RosUi.ntpSummary.state); appPalette: root.appPalette }
        }

        SectionTitle { text: I18n.t(root.language, "ntpDetails"); appPalette: root.appPalette }

        SimpleTable {
            appPalette: root.appPalette
            columns: [
                { label: I18n.t(root.language, "name"), key: "name", width: 260 },
                { label: I18n.t(root.language, "value"), key: "value", fill: true }
            ]
            rows: RosUi.ntpRows
        }
    }
}
