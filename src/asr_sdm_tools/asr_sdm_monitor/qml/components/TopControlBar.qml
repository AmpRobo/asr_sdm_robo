import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../utils/I18n.js" as I18n

Rectangle {
    id: root
    property var appPalette
    property string language: "en"
    property string themeMode: "dark"
    signal themeModeChangedByUser(string mode)
    signal languageChangedByUser(string languageCode)

    color: appPalette.sidebarBackground
    border.color: appPalette.border
    border.width: 1
    implicitHeight: 72

    function comboTextRole(items, index) {
        return items[index].label
    }

    readonly property var themeOptions: [
        { value: "dark", label: I18n.t(language, "darkTheme") },
        { value: "light", label: I18n.t(language, "lightTheme") }
    ]

    readonly property var languageOptions: [
        { value: "en", label: I18n.t(language, "english") },
        { value: "zh", label: I18n.t(language, "chinese") }
    ]

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 20
        anchors.rightMargin: 20
        spacing: 18

        Text {
            text: I18n.t(root.language, "appTitle")
            font.pixelSize: 22
            font.bold: true
            color: appPalette.textPrimary
        }

        Item { Layout.fillWidth: true }

        Text {
            text: I18n.t(root.language, "theme")
            font.pixelSize: 16
            color: appPalette.textPrimary
        }

        ComboBox {
            id: themeCombo
            Layout.preferredWidth: 180
            model: root.themeOptions
            textRole: "label"
            currentIndex: root.themeMode === "light" ? 1 : 0
            onActivated: root.themeModeChangedByUser(root.themeOptions[currentIndex].value)

            palette.window: appPalette.inputBackground
            palette.button: appPalette.inputBackground
            palette.base: appPalette.inputBackground
            palette.text: appPalette.textPrimary
            palette.buttonText: appPalette.textPrimary
            palette.highlight: appPalette.accent
            palette.highlightedText: appPalette.accentText

            background: Rectangle {
                radius: 8
                color: appPalette.inputBackground
                border.color: appPalette.controlBorder
            }
            contentItem: Text {
                leftPadding: 12
                rightPadding: 36
                verticalAlignment: Text.AlignVCenter
                text: themeCombo.displayText
                font.pixelSize: 15
                color: appPalette.textPrimary
            }
        }

        Text {
            text: I18n.t(root.language, "language")
            font.pixelSize: 16
            color: appPalette.textPrimary
        }

        ComboBox {
            id: languageCombo
            Layout.preferredWidth: 160
            model: root.languageOptions
            textRole: "label"
            currentIndex: root.language === "zh" ? 1 : 0
            onActivated: root.languageChangedByUser(root.languageOptions[currentIndex].value)

            palette.window: appPalette.inputBackground
            palette.button: appPalette.inputBackground
            palette.base: appPalette.inputBackground
            palette.text: appPalette.textPrimary
            palette.buttonText: appPalette.textPrimary
            palette.highlight: appPalette.accent
            palette.highlightedText: appPalette.accentText

            background: Rectangle {
                radius: 8
                color: appPalette.inputBackground
                border.color: appPalette.controlBorder
            }
            contentItem: Text {
                leftPadding: 12
                rightPadding: 36
                verticalAlignment: Text.AlignVCenter
                text: languageCombo.displayText
                font.pixelSize: 15
                color: appPalette.textPrimary
            }
        }
    }
}
