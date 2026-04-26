import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import "../components"
import "hardware"

Item {
    id: root
    property var appPalette
    property string language: "en"
    property int currentTab: 0
    signal tabChanged(int index)

    ColumnLayout {
        anchors.fill: parent
        spacing: 16

        HardwareTabBar {
            Layout.fillWidth: true
            appPalette: root.appPalette
            language: root.language
            currentIndex: root.currentTab
            onTabSelected: function(index) {
		root.tabChanged(index)
	    }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: root.currentTab

            CpuPage { appPalette: root.appPalette; language: root.language }
            MemoryPage { appPalette: root.appPalette; language: root.language }
            HddPage { appPalette: root.appPalette; language: root.language }
            NetPage { appPalette: root.appPalette; language: root.language }
            NtpPage { appPalette: root.appPalette; language: root.language }
        }
    }
}
