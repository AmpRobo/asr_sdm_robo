import QtQuick
import QtQuick.Controls
import QtQuick.Window
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
    flags: Qt.Window | Qt.FramelessWindowHint

    readonly property int resizeHandleSize: 6
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

        Rectangle {
            id: windowTitleBar
            Layout.fillWidth: true
            implicitHeight: 36
            color: window.appPalette.headerBackground
            border.color: window.appPalette.border
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: I18n.t(window.currentLanguage, "appTitle")
                font.pixelSize: 14
                font.bold: true
                color: window.appPalette.textSecondary
            }

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton
                onPressed: function(mouse) {
                    if (mouse.button === Qt.LeftButton) {
                        window.startSystemMove()
                    }
                }
                onDoubleClicked: {
                    if (window.visibility === Window.Maximized) {
                        window.showNormal()
                    } else {
                        window.showMaximized()
                    }
                }
            }
        }

        TopControlBar {
            Layout.fillWidth: true
            appPalette: window.appPalette
            language: window.currentLanguage
            themeMode: window.currentThemeMode
            onThemeModeChangedByUser: function(mode) {
                window.currentThemeMode = mode
            }
            onLanguageChangedByUser: function(languageCode) {
                window.currentLanguage = languageCode
            }
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
                onSectionSelected: function(index) {
		    window.currentSection = index
		}
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
                        onTabChanged: function(index) {
			    window.currentHardwareTab = index
			}
                    }

                    VideoPage {
                        appPalette: window.appPalette
                        language: window.currentLanguage
                    }
                }
            }
        }
    }

    MouseArea {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeVerCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.TopEdge)
    }

    MouseArea {
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeVerCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.BottomEdge)
    }

    MouseArea {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeHorCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.LeftEdge)
    }

    MouseArea {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeHorCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.RightEdge)
    }

    MouseArea {
        anchors.left: parent.left
        anchors.top: parent.top
        width: window.resizeHandleSize
        height: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeFDiagCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.LeftEdge | Qt.TopEdge)
    }

    MouseArea {
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        width: window.resizeHandleSize
        height: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeFDiagCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.RightEdge | Qt.BottomEdge)
    }

    MouseArea {
        anchors.right: parent.right
        anchors.top: parent.top
        width: window.resizeHandleSize
        height: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeBDiagCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.RightEdge | Qt.TopEdge)
    }

    MouseArea {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        width: window.resizeHandleSize
        height: window.resizeHandleSize
        visible: window.visibility !== Window.Maximized
        cursorShape: Qt.SizeBDiagCursor
        acceptedButtons: Qt.LeftButton
        onPressed: window.startSystemResize(Qt.LeftEdge | Qt.BottomEdge)
    }
}
