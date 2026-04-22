import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import RosUi 1.0
import "components"
import "pages"
import "utils/Theme.js" as Theme
import "utils/I18n.js" as I18n

ApplicationWindow {
    id: window
    visible: true
    width: 1440
    height: 900
    title: I18n.t(currentLanguage, "appTitle")

    property string currentThemeMode: "dark"
    property string currentLanguage: "en"
    property int currentSection: 0
    property int currentHardwareTab: 0
    readonly property var appPalette: Theme.palette(currentThemeMode)

    color: appPalette.windowBackground
    font.pixelSize: 16

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TopControlBar {
            Layout.fillWidth: true
            appPalette: window.appPalette
            language: window.currentLanguage
            themeMode: window.currentThemeMode
            onThemeModeChangedByUser: window.currentThemeMode = mode
            onLanguageChangedByUser: window.currentLanguage = languageCode
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            SidebarMenu {
                Layout.preferredWidth: 240
                Layout.fillHeight: true
                appPalette: window.appPalette
                language: window.currentLanguage
                currentSection: window.currentSection
                rosStatus: RosUi.rosStatus
                onSectionSelected: window.currentSection = index
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: window.appPalette.contentBackground
                border.color: window.appPalette.border
                border.width: 1

                StackLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    currentIndex: window.currentSection

                    HardwarePage {
                        appPalette: window.appPalette
                        language: window.currentLanguage
                        currentTab: window.currentHardwareTab
                        onTabChanged: window.currentHardwareTab = index
                    }

                    VideoPage {
                        appPalette: window.appPalette
                    }
                }
            }
        }
    }
}
