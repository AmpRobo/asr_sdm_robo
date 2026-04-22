import QtQuick
import QtQuick.Layouts

Rectangle {
    id: root
    property var appPalette
    property var primaryData: []
    property var secondaryData: []
    property string primaryLabel: ""
    property string secondaryLabel: ""
    property string scaleText: ""
    property real maxY: 1.0
    property int horizontalGridLines: 5
    property int verticalGridLines: 7

    radius: 12
    color: appPalette.surfaceBackground
    border.color: appPalette.border
    border.width: 1
    Layout.fillWidth: true
    implicitHeight: 320

    Column {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 10

        RowLayout {
            width: parent.width
            spacing: 18

            Text {
                text: root.primaryLabel
                font.pixelSize: 16
                color: appPalette.primaryChart
            }

            Text {
                visible: root.secondaryData && root.secondaryData.length > 0
                text: root.secondaryLabel
                font.pixelSize: 16
                color: appPalette.secondaryChart
            }

            Item { Layout.fillWidth: true }

            Text {
                visible: root.scaleText !== ""
                text: root.scaleText
                font.pixelSize: 14
                color: appPalette.textSecondary
            }
        }

        Canvas {
            id: chartCanvas
            anchors.left: parent.left
            anchors.right: parent.right
            height: 250

            onPaint: {
                const ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)

                const w = width
                const h = height
                const left = 18
                const right = w - 18
                const top = 12
                const bottom = h - 18
                const chartW = right - left
                const chartH = bottom - top
                const yMax = Math.max(0.000001, root.maxY)

                ctx.strokeStyle = appPalette.chartGrid
                ctx.lineWidth = 1

                for (let gx = 0; gx < root.verticalGridLines; ++gx) {
                    const x = left + chartW * gx / Math.max(1, root.verticalGridLines - 1)
                    ctx.beginPath()
                    ctx.moveTo(x, top)
                    ctx.lineTo(x, bottom)
                    ctx.stroke()
                }

                for (let gy = 0; gy < root.horizontalGridLines; ++gy) {
                    const y = top + chartH * gy / Math.max(1, root.horizontalGridLines - 1)
                    ctx.beginPath()
                    ctx.moveTo(left, y)
                    ctx.lineTo(right, y)
                    ctx.stroke()
                }

                function drawSeries(data, lineColor, fillColor) {
                    if (!data || data.length < 2)
                        return

                    ctx.beginPath()
                    for (let i = 0; i < data.length; ++i) {
                        const value = Math.max(0, Math.min(yMax, Number(data[i])))
                        const x = left + chartW * i / Math.max(1, data.length - 1)
                        const y = bottom - chartH * (value / yMax)
                        if (i === 0)
                            ctx.moveTo(x, y)
                        else
                            ctx.lineTo(x, y)
                    }
                    ctx.strokeStyle = lineColor
                    ctx.lineWidth = 2.5
                    ctx.stroke()

                    ctx.beginPath()
                    for (let j = 0; j < data.length; ++j) {
                        const value2 = Math.max(0, Math.min(yMax, Number(data[j])))
                        const x2 = left + chartW * j / Math.max(1, data.length - 1)
                        const y2 = bottom - chartH * (value2 / yMax)
                        if (j === 0)
                            ctx.moveTo(x2, y2)
                        else
                            ctx.lineTo(x2, y2)
                    }
                    ctx.lineTo(right, bottom)
                    ctx.lineTo(left, bottom)
                    ctx.closePath()
                    ctx.fillStyle = fillColor
                    ctx.fill()
                }

                drawSeries(root.primaryData, appPalette.primaryChart, appPalette.primaryFill)
                drawSeries(root.secondaryData, appPalette.secondaryChart, appPalette.secondaryFill)
            }

            onWidthChanged: requestPaint()
            onHeightChanged: requestPaint()
        }
    }

    onPrimaryDataChanged: chartCanvas.requestPaint()
    onSecondaryDataChanged: chartCanvas.requestPaint()
    onMaxYChanged: chartCanvas.requestPaint()
    onScaleTextChanged: chartCanvas.requestPaint()
    onHorizontalGridLinesChanged: chartCanvas.requestPaint()
    onVerticalGridLinesChanged: chartCanvas.requestPaint()
    onAppPaletteChanged: chartCanvas.requestPaint()
}
