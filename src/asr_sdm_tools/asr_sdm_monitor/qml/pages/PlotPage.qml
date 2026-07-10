import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import RosUi 1.0
import "../components"
import "../utils/I18n.js" as I18n

Item {
    id: root
    property var appPalette
    property string language: "en"
    property string xAxisMode: "time"
    property string timestampMode: "relative"
    property string axisScaleMode: "independent"
    property string xField: ""
    property string recordingPath: ""
    property string recordedPath: ""
    property int ySeriesNumber: 1
    property var ySeries: []
    property double timeWindowSeconds: 4.0
    property bool xShowTickLabels: true
    property bool yShowTickLabels: true
    property string settingsSource: "live"
    property string plotSection: "live"
    property string topicSortMode: "nameAsc"
    property bool suppressDataSourceSync: false
    property var livePlotSettings: ({
        xAxisMode: "time",
        timestampMode: "relative",
        axisScaleMode: "independent",
        xField: "",
        ySeriesNumber: 1,
        ySeries: [],
        timeWindowSeconds: 4.0,
        xShowTickLabels: true,
        yShowTickLabels: true
    })
    property var recordedPlotSettings: ({
        xAxisMode: "time",
        timestampMode: "relative",
        axisScaleMode: "independent",
        xField: "",
        ySeriesNumber: 1,
        ySeries: [],
        timeWindowSeconds: 4.0,
        xShowTickLabels: true,
        yShowTickLabels: true
    })
    property var displayedPlotSamples: []

    readonly property int baseFontSize: 16
    readonly property double defaultSeriesLineWidth: 1.0
    readonly property string dataSource: RosUi.plotDataSource
    readonly property var fieldOptions: root.dataSource === "recorded" ? RosUi.recordedPlotFieldOptions : RosUi.plotFieldOptions
    readonly property var sourceSamples: root.dataSource === "recorded" ? RosUi.recordedPlotSamples : RosUi.imuPlotSamples
    readonly property var plotSectionOptions: [
        { value: "topics", label: I18n.t(root.language, "topics") },
        { value: "live", label: I18n.t(root.language, "live") },
        { value: "recorded", label: I18n.t(root.language, "recorded") }
    ]
    readonly property var topicSortOptions: [
        { value: "nameAsc", label: I18n.t(root.language, "topicSortNameAscending") },
        { value: "nameDesc", label: I18n.t(root.language, "topicSortNameDescending") },
        { value: "source", label: I18n.t(root.language, "topicSortSource") },
        { value: "plottableFirst", label: I18n.t(root.language, "topicSortPlottableFirst") }
    ]
    readonly property var xAxisOptions: [
        { value: "time", label: I18n.t(root.language, "time") },
        { value: "topic", label: I18n.t(root.language, "topicMessage") }
    ]
    readonly property var timestampOptions: [
        { value: "relative", label: I18n.t(root.language, "relativeTime") },
        { value: "absolute", label: I18n.t(root.language, "absoluteTime") }
    ]
    readonly property var axisScaleOptions: [
        { value: "independent", label: I18n.t(root.language, "independent") },
        { value: "square", label: I18n.t(root.language, "square") }
    ]
    readonly property var playbackSpeedOptions: [
        { value: 0.25, label: "0.25x" },
        { value: 0.5, label: "0.5x" },
        { value: 1.0, label: "1.0x" },
        { value: 2.0, label: "2.0x" },
        { value: 4.0, label: "4.0x" }
    ]
    readonly property string plotMode: root.xAxisMode === "time" ? "time" : "xy"
    readonly property string longestTopicText: longestOptionText()
    readonly property string longestCapabilityText: longestTopicCapabilityText()
    readonly property string longestSettingLabelText: longestSettingLabel()
    readonly property string longestSeriesLabelText: longestSeriesLabel()
    readonly property int settingsLabelColumnWidth: Math.ceil(Math.max(128, settingsLabelProbe.implicitWidth + 10))
    readonly property int seriesLabelColumnWidth: Math.ceil(Math.max(92, seriesLabelProbe.implicitWidth + 10))
    readonly property int sidebarTargetWidth: Math.ceil(Math.max(420, Math.min(760, Math.max(longestTopicProbe.implicitWidth + settingsLabelColumnWidth + 120, settingsLabelColumnWidth + 300))))
    readonly property var selectedYFields: selectedYFieldList()
    readonly property var activeSeriesOptions: activeSeriesList()

    Text {
        id: longestTopicProbe
        visible: false
        text: root.longestTopicText
        font.pixelSize: root.baseFontSize
    }

    Text {
        id: capabilityHeaderProbe
        visible: false
        text: I18n.t(root.language, "topicCapability")
        font.pixelSize: root.baseFontSize
        font.bold: true
    }

    Text {
        id: capabilityValueProbe
        visible: false
        text: root.longestCapabilityText
        font.pixelSize: root.baseFontSize - 1
    }

    Text {
        id: capabilityRequiredProbe
        visible: false
        text: I18n.t(root.language, "plottable") + " · "
              + I18n.t(root.language, "recordableOnly")
        font.pixelSize: root.baseFontSize - 1
    }

    Text {
        id: settingsLabelProbe
        visible: false
        text: root.longestSettingLabelText
        font.pixelSize: root.baseFontSize
    }

    Text {
        id: seriesLabelProbe
        visible: false
        text: root.longestSeriesLabelText
        font.pixelSize: root.baseFontSize
    }

    function defaultPlotSettings() {
        return {
            xAxisMode: "time",
            timestampMode: "relative",
            axisScaleMode: "independent",
            xField: "",
            ySeriesNumber: 1,
            ySeries: [],
            timeWindowSeconds: 4.0,
            xShowTickLabels: true,
            yShowTickLabels: true
        }
    }

    function cloneSeries(series) {
        const result = []
        const src = series || []
        for (let i = 0; i < src.length; ++i) {
            const item = src[i] || {}
            result.push({
                field: String(item.field || ""),
                color: item.color || defaultSeriesColor(i),
                lineWidth: Number(item.lineWidth) > 0 ? Number(item.lineWidth) : root.defaultSeriesLineWidth,
                customColor: !!item.customColor
            })
        }
        return result
    }

    function currentPlotSettings() {
        return {
            xAxisMode: root.xAxisMode,
            timestampMode: root.timestampMode,
            axisScaleMode: root.axisScaleMode,
            xField: root.xField,
            ySeriesNumber: root.ySeriesNumber,
            ySeries: cloneSeries(root.ySeries),
            timeWindowSeconds: root.timeWindowSeconds,
            xShowTickLabels: root.xShowTickLabels,
            yShowTickLabels: root.yShowTickLabels
        }
    }

    function saveCurrentSettings() {
        const saved = currentPlotSettings()
        if (root.settingsSource === "recorded")
            root.recordedPlotSettings = saved
        else
            root.livePlotSettings = saved
    }

    function loadSettingsForSource(source) {
        const settings = source === "recorded" ? root.recordedPlotSettings : root.livePlotSettings
        root.settingsSource = source
        root.xAxisMode = settings.xAxisMode || "time"
        root.timestampMode = settings.timestampMode || "relative"
        root.axisScaleMode = settings.axisScaleMode || "independent"
        root.xField = settings.xField || ""
        root.timeWindowSeconds = Number(settings.timeWindowSeconds) > 0 ? Number(settings.timeWindowSeconds) : 4.0
        root.xShowTickLabels = settings.xShowTickLabels !== false
        root.yShowTickLabels = settings.yShowTickLabels !== false
        root.ySeries = cloneSeries(settings.ySeries)
        root.ySeriesNumber = Math.max(1, Math.min(16, Math.round(Number(settings.ySeriesNumber) || 1)))
        root.ensureSeriesCount()
        if (root.xAxisMode === "topic" && !root.containsField(root.xField))
            root.xField = root.firstFieldPath()
        root.refreshDisplayedSamples()
    }

    function switchPlotSource(source) {
        if (source === root.dataSource)
            return
        root.saveCurrentSettings()
        root.suppressDataSourceSync = true
        RosUi.setPlotDataSource(source)
        root.loadSettingsForSource(source)
        root.suppressDataSourceSync = false
    }

    function switchPlotSection(section) {
        if (section === root.plotSection)
            return
        if (section === "topics") {
            root.saveCurrentSettings()
            root.plotSection = "topics"
            RosUi.refreshPlotTopics()
            return
        }
        root.plotSection = section
        root.switchPlotSource(section)
    }

    function indexOfValue(options, value, role) {
        const key = role || "value"
        if (!options)
            return 0
        for (let i = 0; i < options.length; ++i) {
            if (String(options[i][key]) === String(value))
                return i
        }
        return 0
    }

    function longestOptionText() {
        const labels = [
            I18n.t(root.language, "plotSettings"),
            I18n.t(root.language, "timestampFormat"),
            I18n.t(root.language, "showTickLabels"),
            I18n.t(root.language, "seriesNumber"),
            I18n.t(root.language, "timeWindow"),
            I18n.t(root.language, "labelType"),
            I18n.t(root.language, "label"),
            I18n.t(root.language, "speed")
        ]
        let longest = labels[0]
        for (let l = 0; l < labels.length; ++l) {
            if (labels[l].length > longest.length)
                longest = labels[l]
        }
        for (let i = 0; i < root.fieldOptions.length; ++i) {
            const path = String(root.fieldOptions[i].path || "")
            if (path.length > longest.length)
                longest = path
        }
        return longest
    }

    function longestSettingLabel() {
        const labels = [
            I18n.t(root.language, "labelType"),
            I18n.t(root.language, "label"),
            I18n.t(root.language, "axisScale"),
            I18n.t(root.language, "showTickLabels"),
            I18n.t(root.language, "timestampFormat"),
            I18n.t(root.language, "currentTime"),
            I18n.t(root.language, "timeWindow"),
            I18n.t(root.language, "seriesNumber"),
            I18n.t(root.language, "startTime"),
            I18n.t(root.language, "endTime"),
            I18n.t(root.language, "speed")
        ]
        let longest = labels[0]
        for (let i = 0; i < labels.length; ++i) {
            if (labels[i].length > longest.length)
                longest = labels[i]
        }
        return longest
    }

    function longestSeriesLabel() {
        const labels = [
            I18n.t(root.language, "label"),
            I18n.t(root.language, "color"),
            I18n.t(root.language, "lineWidth")
        ]
        let longest = labels[0]
        for (let i = 0; i < labels.length; ++i) {
            if (labels[i].length > longest.length)
                longest = labels[i]
        }
        return longest
    }

    function containsField(path) {
        if (!path || path.length === 0)
            return false
        for (let i = 0; i < root.fieldOptions.length; ++i) {
            if (root.fieldOptions[i].path === path)
                return true
        }
        return false
    }

    function firstFieldPath() {
        return root.fieldOptions.length > 0 ? root.fieldOptions[0].path : ""
    }

    function defaultSeriesColor(index) {
        if (!root.appPalette)
            return "transparent"
        const colors = root.appPalette.chartSeriesColors || []
        if (colors.length > 0)
            return colors[index % colors.length]
        return root.appPalette.primaryChart || root.appPalette.accent || root.appPalette.textPrimary
    }

    function defaultSeries(index) {
        return { field: "", color: defaultSeriesColor(index), lineWidth: root.defaultSeriesLineWidth, customColor: false }
    }

    function normalizeSeriesFields() {
        const next = []
        const used = {}
        for (let i = 0; i < root.ySeries.length; ++i) {
            const src = root.ySeries[i] || {}
            let field = String(src.field || "")
            if (!root.containsField(field) || used[field])
                field = ""
            if (field.length > 0)
                used[field] = true
            next.push({
                field: field,
                color: src.customColor ? (src.color || defaultSeriesColor(i)) : defaultSeriesColor(i),
                lineWidth: Number(src.lineWidth) > 0 ? Number(src.lineWidth) : root.defaultSeriesLineWidth,
                customColor: !!src.customColor
            })
        }
        root.ySeries = next
    }

    function ensureSeriesCount() {
        let count = Math.round(Number(root.ySeriesNumber))
        if (!isFinite(count) || count < 1)
            count = 1
        count = Math.min(16, count)
        if (root.ySeriesNumber !== count)
            root.ySeriesNumber = count

        const next = []
        for (let i = 0; i < count; ++i) {
            const src = i < root.ySeries.length ? root.ySeries[i] : defaultSeries(i)
            next.push({
                field: src.field || "",
                color: src.customColor ? (src.color || defaultSeriesColor(i)) : defaultSeriesColor(i),
                lineWidth: Number(src.lineWidth) > 0 ? Number(src.lineWidth) : root.defaultSeriesLineWidth,
                customColor: !!src.customColor
            })
        }
        root.ySeries = next
        normalizeSeriesFields()
    }

    function availableFieldsForSeries(seriesIndex) {
        const selectedElsewhere = {}
        for (let i = 0; i < root.ySeries.length; ++i) {
            if (i === seriesIndex)
                continue
            const field = String(root.ySeries[i].field || "")
            if (field.length > 0)
                selectedElsewhere[field] = true
        }

        const result = [{ path: "", display: I18n.t(root.language, "none") }]
        for (let j = 0; j < root.fieldOptions.length; ++j) {
            const path = root.fieldOptions[j].path
            if (!selectedElsewhere[path])
                result.push({ path: path, display: root.fieldOptions[j].label || path })
        }
        return result
    }

    function setSeriesField(seriesIndex, path) {
        const next = root.ySeries.slice()
        if (seriesIndex < 0 || seriesIndex >= next.length)
            return
        const src = next[seriesIndex] || defaultSeries(seriesIndex)
        next[seriesIndex] = {
            field: path,
            color: src.color || defaultSeriesColor(seriesIndex),
            lineWidth: Number(src.lineWidth) > 0 ? Number(src.lineWidth) : root.defaultSeriesLineWidth,
            customColor: !!src.customColor
        }
        root.ySeries = next
        normalizeSeriesFields()
    }

    function setSeriesColor(seriesIndex, colorText) {
        const next = root.ySeries.slice()
        if (seriesIndex < 0 || seriesIndex >= next.length)
            return
        const src = next[seriesIndex] || defaultSeries(seriesIndex)
        const nextColor = String(colorText || "").trim().length > 0 ? String(colorText).trim() : defaultSeriesColor(seriesIndex)
        next[seriesIndex] = { field: src.field || "", color: nextColor, lineWidth: Number(src.lineWidth) > 0 ? Number(src.lineWidth) : root.defaultSeriesLineWidth, customColor: true }
        root.ySeries = next
    }

    function setSeriesLineWidth(seriesIndex, widthText) {
        const next = root.ySeries.slice()
        if (seriesIndex < 0 || seriesIndex >= next.length)
            return
        const src = next[seriesIndex] || defaultSeries(seriesIndex)
        const parsed = Number(widthText)
        const nextWidth = isFinite(parsed) && parsed > 0 ? parsed : root.defaultSeriesLineWidth
        next[seriesIndex] = { field: src.field || "", color: src.color || defaultSeriesColor(seriesIndex), lineWidth: nextWidth, customColor: !!src.customColor }
        root.ySeries = next
    }

    function updateDefaultSeriesColors() {
        const next = []
        for (let i = 0; i < root.ySeries.length; ++i) {
            const src = root.ySeries[i] || defaultSeries(i)
            next.push({
                field: src.field || "",
                color: src.customColor ? (src.color || defaultSeriesColor(i)) : defaultSeriesColor(i),
                lineWidth: Number(src.lineWidth) > 0 ? Number(src.lineWidth) : root.defaultSeriesLineWidth,
                customColor: !!src.customColor
            })
        }
        root.ySeries = next
    }

    function selectedYFieldList() {
        const result = []
        for (let i = 0; i < root.ySeries.length; ++i) {
            const field = String(root.ySeries[i].field || "")
            if (field.length > 0 && root.containsField(field))
                result.push(field)
        }
        return result
    }

    function activeSeriesList() {
        const result = []
        for (let i = 0; i < root.ySeries.length; ++i) {
            const item = root.ySeries[i] || {}
            const field = String(item.field || "")
            if (field.length > 0 && root.containsField(field)) {
                result.push({
                    field: field,
                    color: item.color || defaultSeriesColor(result.length),
                    lineWidth: Number(item.lineWidth) > 0 ? Number(item.lineWidth) : root.defaultSeriesLineWidth
                })
            }
        }
        return result
    }

    function timeAxisLabel() {
        return root.timestampMode === "absolute"
               ? I18n.t(root.language, "absoluteTime")
               : I18n.t(root.language, "relativeTime") + " (s)"
    }

    function fieldLabel(path) {
        for (let i = 0; i < root.fieldOptions.length; ++i) {
            if (root.fieldOptions[i].path === path)
                return root.fieldOptions[i].label
        }
        return path
    }

    function zeroPad(value, width) {
        let text = String(value)
        while (text.length < width)
            text = "0" + text
        return text
    }

    function formatAbsoluteTime(ms) {
        if (!isFinite(ms) || ms <= 0)
            return "--"
        const date = new Date(Number(ms))
        return zeroPad(date.getHours(), 2) + ":" + zeroPad(date.getMinutes(), 2) + ":" + zeroPad(date.getSeconds(), 2) + "." + zeroPad(date.getMilliseconds(), 3)
    }

    function parsePlaybackTimeInput(text, fallbackMs) {
        const raw = String(text || "").trim()
        if (raw.length === 0)
            return fallbackMs

        const numeric = Number(raw)
        if (isFinite(numeric)) {
            if (numeric > 100000000000.0)
                return numeric
            return RosUi.playbackStartTimeMs + numeric * 1000.0
        }

        const parts = raw.match(/^(\d{1,2}):(\d{1,2}):(\d{1,2})(?:\.(\d{1,3}))?$/)
        if (parts) {
            const base = new Date(Number(fallbackMs))
            const msText = parts[4] || "0"
            const ms = Number((msText + "000").slice(0, 3))
            base.setHours(Number(parts[1]), Number(parts[2]), Number(parts[3]), ms)
            return base.getTime()
        }

        return fallbackMs
    }


    function topicSummaryText() {
        const topics = RosUi.plotTopics || []
        let selected = 0
        let fields = 0
        for (let i = 0; i < topics.length; ++i) {
            if (topics[i].selected) {
                selected += 1
                fields += Number(topics[i].fieldCount || 0)
            }
        }
        return I18n.t(root.language, "topicSummaryTopics") + ": " + topics.length
               + "   " + I18n.t(root.language, "topicSummarySelected") + ": " + selected
               + "   " + I18n.t(root.language, "topicSummaryFields") + ": " + fields
    }

    function topicsHeadingText() {
        return I18n.t(root.language, "allTopicSources")
    }

    function topicsHintText() {
        return I18n.t(root.language, "unifiedTopicCandidateHint")
    }

    function topicSourceRank(topicItem) {
        return topicItem && topicItem.sourceKind === "ros2_bag_play" ? 1 : 0
    }

    function sortedPlotTopics() {
        const topics = (RosUi.plotTopics || []).slice()
        const mode = root.topicSortMode
        topics.sort(function(left, right) {
            const leftName = String(left.name || "")
            const rightName = String(right.name || "")
            const nameOrder = leftName.localeCompare(rightName)

            if (mode === "nameDesc") {
                if (nameOrder !== 0)
                    return -nameOrder
            } else if (mode === "source") {
                const sourceOrder = root.topicSourceRank(left) - root.topicSourceRank(right)
                if (sourceOrder !== 0)
                    return sourceOrder
                const leftSource = String(left.sourceName || "")
                const rightSource = String(right.sourceName || "")
                const sourceNameOrder = leftSource.localeCompare(rightSource)
                if (sourceNameOrder !== 0)
                    return sourceNameOrder
                if (nameOrder !== 0)
                    return nameOrder
            } else if (mode === "plottableFirst") {
                const capabilityOrder = Number(!!right.plottable) - Number(!!left.plottable)
                if (capabilityOrder !== 0)
                    return capabilityOrder
                if (nameOrder !== 0)
                    return nameOrder
            } else if (nameOrder !== 0) {
                return nameOrder
            }

            const sourceOrder = root.topicSourceRank(left) - root.topicSourceRank(right)
            if (sourceOrder !== 0)
                return sourceOrder
            return String(left.key || "").localeCompare(String(right.key || ""))
        })
        return topics
    }

    function topicSourceText(topicItem) {
        if (!topicItem)
            return "--"

        let sourceLabel = I18n.t(root.language, "ros2LiveSource")
        if (topicItem.sourceKind === "ros2_bag_play")
            sourceLabel = I18n.t(root.language, "ros2BagPlaySource")

        const sourceName = String(topicItem.sourceName || "")
        return sourceName.length > 0 ? sourceLabel + " · " + sourceName : sourceLabel
    }

    function topicCapabilityText(topicItem) {
        if (topicItem && topicItem.plottable) {
            let capability = I18n.t(root.language, "plottable") + " · " + Number(topicItem.fieldCount || 0)
            if (topicItem.recordable)
                capability += " · " + I18n.t(root.language, "recordableOnly")
            return capability
        }
        if (topicItem && topicItem.recordable)
            return I18n.t(root.language, "recordableOnly")
        return I18n.t(root.language, "unsupportedPlotTopic")
    }

    function longestTopicCapabilityText() {
        const topics = RosUi.plotTopics || []
        let longest = I18n.t(root.language, "unsupportedPlotTopic")
        const recordable = I18n.t(root.language, "recordableOnly")
        if (recordable.length > longest.length)
            longest = recordable

        for (let i = 0; i < topics.length; ++i) {
            const capability = root.topicCapabilityText(topics[i])
            if (capability.length > longest.length)
                longest = capability
        }
        return longest
    }

    function currentLiveTimeText() {
        const samples = RosUi.imuPlotSamples || []
        if (samples.length === 0)
            return "--"
        const sample = samples[samples.length - 1]
        return root.timestampMode === "absolute"
               ? root.formatAbsoluteTime(sample.absoluteTimeMs)
               : Number(sample.relativeTime || 0).toFixed(3) + " s"
    }

    function applyTimeWindow(text) {
        const parsed = Number(text)
        if (isFinite(parsed) && parsed > 0)
            root.timeWindowSeconds = parsed
        else
            root.timeWindowSeconds = 4.0
    }

    function lowerBoundByTime(src, targetMs) {
        let lo = 0
        let hi = src.length
        while (lo < hi) {
            const mid = Math.floor((lo + hi) / 2)
            const t = Number(src[mid].absoluteTimeMs)
            if (!isFinite(t) || t < targetMs)
                lo = mid + 1
            else
                hi = mid
        }
        return lo
    }

    function filteredByTimeWindow(src, currentAbsoluteMs) {
        if (root.xAxisMode !== "time")
            return src
        const windowMs = Math.max(0.05, Number(root.timeWindowSeconds)) * 1000.0
        const halfWindowMs = windowMs / 2.0
        const startMs = currentAbsoluteMs - halfWindowMs
        const endMs = currentAbsoluteMs + halfWindowMs
        const first = lowerBoundByTime(src, startMs)
        const last = lowerBoundByTime(src, endMs)
        const filtered = []
        for (let i = first; i < src.length && i < last; ++i) {
            const sample = src[i]
            const t = Number(sample.absoluteTimeMs)
            if (isFinite(t) && t >= startMs && t <= endMs)
                filtered.push(sample)
            if (isFinite(t) && t > endMs)
                break
        }
        return filtered
    }

    function refreshDisplayedSamples() {
        const src = root.sourceSamples || []
        if (src.length === 0) {
            root.displayedPlotSamples = []
            return
        }

        if (root.dataSource === "recorded") {
            root.displayedPlotSamples = filteredByTimeWindow(src, RosUi.playbackCurrentTimeMs)
            return
        }

        if (root.xAxisMode === "time") {
            const latest = Number(src[src.length - 1].absoluteTimeMs)
            root.displayedPlotSamples = isFinite(latest) ? filteredByTimeWindow(src, latest) : src
        } else {
            root.displayedPlotSamples = src
        }
    }

    function openAndPlayRecorded() {
        if (root.recordedPath.length > 0 && root.recordedPath !== RosUi.recordedFilePath)
            RosUi.openRecordedPlotFile(root.recordedPath)
        RosUi.setPlaybackPlaying(!RosUi.playbackPlaying)
    }

    onFieldOptionsChanged: {
        ensureSeriesCount()
        if (root.xAxisMode === "topic" && !root.containsField(root.xField))
            root.xField = root.firstFieldPath()
        refreshDisplayedSamples()
    }
    onDataSourceChanged: {
        if (!root.suppressDataSourceSync) {
            if (root.plotSection !== "topics")
                root.plotSection = root.dataSource
            if (root.settingsSource !== root.dataSource) {
                root.saveCurrentSettings()
                root.loadSettingsForSource(root.dataSource)
            } else {
                root.ensureSeriesCount()
                if (root.xAxisMode === "topic" && !root.containsField(root.xField))
                    root.xField = root.firstFieldPath()
                root.refreshDisplayedSamples()
            }
        }
    }
    onXAxisModeChanged: {
        if (root.xAxisMode === "topic" && !root.containsField(root.xField))
            root.xField = root.firstFieldPath()
        refreshDisplayedSamples()
    }
    onYSeriesNumberChanged: ensureSeriesCount()
    onYSeriesChanged: refreshDisplayedSamples()
    onTimeWindowSecondsChanged: refreshDisplayedSamples()
    onAppPaletteChanged: updateDefaultSeriesColors()

    Connections {
        target: RosUi
        function onImuPlotSamplesChanged() { root.refreshDisplayedSamples() }
        function onRecordedPlotSamplesChanged() { root.refreshDisplayedSamples() }
        function onPlaybackCurrentTimeMsChanged() { root.refreshDisplayedSamples() }
        function onPlotDataSourceChanged() { root.refreshDisplayedSamples(); root.ensureSeriesCount() }
        function onRecordedPlaybackChanged() {
            if (RosUi.recordedFilePath.length > 0)
                root.recordedPath = RosUi.recordedFilePath
            root.refreshDisplayedSamples()
        }
    }

    Component.onCompleted: {
        root.recordingPath = RosUi.defaultPlotRecordingPath()
        root.plotSection = "topics"
        root.settingsSource = root.dataSource
        root.loadSettingsForSource(root.dataSource)
        root.refreshDisplayedSamples()
        RosUi.refreshPlotTopics()
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 46
            spacing: root.width < 520 ? 6 : 12

            Repeater {
                model: root.plotSectionOptions
                delegate: SelectableButton {
                    label: modelData.label
                    selected: root.plotSection === modelData.value
                    appPalette: root.appPalette
                    Layout.minimumWidth: 64
                    Layout.preferredWidth: 140
                    implicitHeight: 46
                    onClicked: root.switchPlotSection(modelData.value)
                }
            }

            Item { Layout.fillWidth: true }
        }

        RowLayout {
            visible: root.plotSection !== "topics"
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            Rectangle {
                id: plotSidebar
                Layout.preferredWidth: Math.min(root.sidebarTargetWidth, Math.max(420, root.width * 0.48))
                Layout.minimumWidth: Math.min(400, Math.max(300, root.width * 0.34))
                Layout.maximumWidth: Math.max(500, Math.min(760, root.width * 0.58))
                Layout.fillHeight: true
                color: root.appPalette.sidebarBackground
                radius: 12
                border.color: root.appPalette.border
                border.width: 1
                clip: true

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 14
                    clip: true
                    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                    ScrollBar.vertical.policy: ScrollBar.AsNeeded

                    ColumnLayout {
                        width: Math.max(0, plotSidebar.width - 36)
                        spacing: 12

                        Text {
                            Layout.fillWidth: true
                            text: I18n.t(root.language, "plotSettings")
                            font.pixelSize: root.baseFontSize
                            font.bold: true
                            color: root.appPalette.textPrimary
                            elide: Text.ElideRight
                        }

                        ColumnLayout {
                            visible: root.dataSource === "live"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.fillWidth: true
                                text: I18n.t(root.language, "recordingFile")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: root.recordingPath
                                onTextChanged: root.recordingPath = text
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textPrimary
                                selectByMouse: true
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                            }

                            SelectableButton {
                                Layout.fillWidth: true
                                implicitHeight: 42
                                label: RosUi.plotRecording ? I18n.t(root.language, "stopRecording") : I18n.t(root.language, "startRecording")
                                selected: RosUi.plotRecording
                                appPalette: root.appPalette
                                onClicked: {
                                    if (RosUi.plotRecording) {
                                        RosUi.stopPlotRecording()
                                        root.recordingPath = RosUi.defaultPlotRecordingPath()
                                    } else {
                                        if (root.recordingPath.length === 0)
                                            root.recordingPath = RosUi.defaultPlotRecordingPath()
                                        RosUi.startPlotRecording(root.recordingPath)
                                    }
                                }
                            }
                        }

                        ColumnLayout {
                            visible: root.dataSource === "recorded"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.fillWidth: true
                                text: I18n.t(root.language, "recordedFile")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: root.recordedPath
                                onTextChanged: root.recordedPath = text
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textPrimary
                                selectByMouse: true
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 8

                                SelectableButton {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    implicitHeight: 42
                                    label: I18n.t(root.language, "open")
                                    selected: false
                                    appPalette: root.appPalette
                                    onClicked: {
                                        const folder = RosUi.pickExistingDirectory(
                                            I18n.t(root.language, "selectRecordedBag"),
                                            root.recordedPath)
                                        if (folder.length > 0) {
                                            root.recordedPath = folder
                                            RosUi.openRecordedPlotFile(root.recordedPath)
                                        }
                                    }
                                }

                                SelectableButton {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    implicitHeight: 42
                                    label: RosUi.playbackPlaying ? I18n.t(root.language, "pause") : I18n.t(root.language, "play")
                                    selected: RosUi.playbackPlaying
                                    appPalette: root.appPalette
                                    onClicked: root.openAndPlayRecorded()
                                }
                            }

                            ColumnLayout {
                                Layout.fillWidth: true
                                spacing: 6

                                RowLayout {
                                    Layout.fillWidth: true
                                    spacing: 8
                                    Text { Layout.preferredWidth: root.settingsLabelColumnWidth; text: I18n.t(root.language, "startTime"); font.pixelSize: root.baseFontSize; color: root.appPalette.textSecondary; elide: Text.ElideRight }
                                    TextField {
                                        Layout.fillWidth: true
                                        text: root.formatAbsoluteTime(RosUi.playbackStartTimeMs)
                                        onEditingFinished: RosUi.setPlaybackStartTimeMs(root.parsePlaybackTimeInput(text, RosUi.playbackStartTimeMs))
                                        font.pixelSize: root.baseFontSize
                                        color: root.appPalette.textPrimary
                                        selectByMouse: true
                                        background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                    }
                                }

                                RowLayout {
                                    Layout.fillWidth: true
                                    spacing: 8
                                    Text { Layout.preferredWidth: root.settingsLabelColumnWidth; text: I18n.t(root.language, "endTime"); font.pixelSize: root.baseFontSize; color: root.appPalette.textSecondary; elide: Text.ElideRight }
                                    TextField {
                                        Layout.fillWidth: true
                                        text: root.formatAbsoluteTime(RosUi.playbackEndTimeMs)
                                        onEditingFinished: RosUi.setPlaybackEndTimeMs(root.parsePlaybackTimeInput(text, RosUi.playbackEndTimeMs))
                                        font.pixelSize: root.baseFontSize
                                        color: root.appPalette.textPrimary
                                        selectByMouse: true
                                        background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                    }
                                }

                                RowLayout {
                                    Layout.fillWidth: true
                                    spacing: 8
                                    Text { Layout.preferredWidth: root.settingsLabelColumnWidth; text: I18n.t(root.language, "currentTime"); font.pixelSize: root.baseFontSize; color: root.appPalette.textSecondary; elide: Text.ElideRight }
                                    TextField {
                                        Layout.fillWidth: true
                                        text: root.formatAbsoluteTime(RosUi.playbackCurrentTimeMs)
                                        onEditingFinished: RosUi.setPlaybackCurrentTimeMs(root.parsePlaybackTimeInput(text, RosUi.playbackCurrentTimeMs))
                                        font.pixelSize: root.baseFontSize
                                        color: root.appPalette.textPrimary
                                        selectByMouse: true
                                        background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                    }
                                }

                                RowLayout {
                                    Layout.fillWidth: true
                                    spacing: 8
                                    Text { Layout.preferredWidth: root.settingsLabelColumnWidth; text: I18n.t(root.language, "speed"); font.pixelSize: root.baseFontSize; color: root.appPalette.textSecondary; elide: Text.ElideRight }
                                    ComboBox {
                                        id: playbackSpeedCombo
                                        Layout.fillWidth: true
                                        model: root.playbackSpeedOptions
                                        textRole: "label"
                                        currentIndex: root.indexOfValue(root.playbackSpeedOptions, RosUi.playbackSpeed)
                                        onActivated: RosUi.setPlaybackSpeed(root.playbackSpeedOptions[currentIndex].value)
                                        palette.window: root.appPalette.inputBackground
                                        palette.button: root.appPalette.inputBackground
                                        palette.base: root.appPalette.inputBackground
                                        palette.text: root.appPalette.textPrimary
                                        palette.buttonText: root.appPalette.textPrimary
                                        palette.highlight: root.appPalette.accent
                                        palette.highlightedText: root.appPalette.accentText
                                        background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                        contentItem: Text { leftPadding: 12; rightPadding: 34; text: playbackSpeedCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                                    }
                                }
                            }
                        }

                        Text {
                            Layout.fillWidth: true
                            text: I18n.t(root.language, "xAxis")
                            font.pixelSize: root.baseFontSize
                            font.bold: true
                            color: root.appPalette.textPrimary
                            elide: Text.ElideRight
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "labelType")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            ComboBox {
                                id: xAxisModeCombo
                                Layout.fillWidth: true
                                model: root.xAxisOptions
                                textRole: "label"
                                currentIndex: root.indexOfValue(root.xAxisOptions, root.xAxisMode)
                                onActivated: root.xAxisMode = root.xAxisOptions[currentIndex].value
                                palette.window: root.appPalette.inputBackground
                                palette.button: root.appPalette.inputBackground
                                palette.base: root.appPalette.inputBackground
                                palette.text: root.appPalette.textPrimary
                                palette.buttonText: root.appPalette.textPrimary
                                palette.highlight: root.appPalette.accent
                                palette.highlightedText: root.appPalette.accentText
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                contentItem: Text { leftPadding: 12; rightPadding: 34; text: xAxisModeCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                            }
                        }

                        RowLayout {
                            visible: root.xAxisMode === "topic"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "label")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            ComboBox {
                                id: xTopicCombo
                                Layout.fillWidth: true
                                model: root.fieldOptions
                                textRole: "label"
                                currentIndex: root.indexOfValue(root.fieldOptions, root.xField, "path")
                                onActivated: {
                                    if (root.fieldOptions.length > 0)
                                        root.xField = root.fieldOptions[currentIndex].path
                                }
                                palette.window: root.appPalette.inputBackground
                                palette.button: root.appPalette.inputBackground
                                palette.base: root.appPalette.inputBackground
                                palette.text: root.appPalette.textPrimary
                                palette.buttonText: root.appPalette.textPrimary
                                palette.highlight: root.appPalette.accent
                                palette.highlightedText: root.appPalette.accentText
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                contentItem: Text { leftPadding: 12; rightPadding: 34; text: xTopicCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                            }
                        }

                        RowLayout {
                            visible: root.xAxisMode === "topic"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "axisScale")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            ComboBox {
                                id: axisScaleCombo
                                Layout.fillWidth: true
                                model: root.axisScaleOptions
                                textRole: "label"
                                currentIndex: root.indexOfValue(root.axisScaleOptions, root.axisScaleMode)
                                onActivated: root.axisScaleMode = root.axisScaleOptions[currentIndex].value
                                palette.window: root.appPalette.inputBackground
                                palette.button: root.appPalette.inputBackground
                                palette.base: root.appPalette.inputBackground
                                palette.text: root.appPalette.textPrimary
                                palette.buttonText: root.appPalette.textPrimary
                                palette.highlight: root.appPalette.accent
                                palette.highlightedText: root.appPalette.accentText
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                contentItem: Text { leftPadding: 12; rightPadding: 34; text: axisScaleCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "showTickLabels")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 8

                                SelectableButton {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    implicitHeight: 34
                                    label: I18n.t(root.language, "on")
                                    selected: root.xShowTickLabels
                                    appPalette: root.appPalette
                                    labelPixelSize: root.baseFontSize - 1
                                    onClicked: root.xShowTickLabels = true
                                }

                                SelectableButton {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    implicitHeight: 34
                                    label: I18n.t(root.language, "off")
                                    selected: !root.xShowTickLabels
                                    appPalette: root.appPalette
                                    labelPixelSize: root.baseFontSize - 1
                                    onClicked: root.xShowTickLabels = false
                                }
                            }
                        }

                        RowLayout {
                            visible: root.xAxisMode === "time"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "timestampFormat")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            ComboBox {
                                id: timestampCombo
                                Layout.fillWidth: true
                                model: root.timestampOptions
                                textRole: "label"
                                currentIndex: root.indexOfValue(root.timestampOptions, root.timestampMode)
                                onActivated: root.timestampMode = root.timestampOptions[currentIndex].value
                                palette.window: root.appPalette.inputBackground
                                palette.button: root.appPalette.inputBackground
                                palette.base: root.appPalette.inputBackground
                                palette.text: root.appPalette.textPrimary
                                palette.buttonText: root.appPalette.textPrimary
                                palette.highlight: root.appPalette.accent
                                palette.highlightedText: root.appPalette.accentText
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                contentItem: Text { leftPadding: 12; rightPadding: 34; text: timestampCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                            }
                        }

                        RowLayout {
                            visible: root.xAxisMode === "time"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "currentTime")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            Text {
                                Layout.fillWidth: true
                                text: root.dataSource === "recorded" ? root.formatAbsoluteTime(RosUi.playbackCurrentTimeMs) : root.currentLiveTimeText()
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textPrimary
                                horizontalAlignment: Text.AlignRight
                                elide: Text.ElideRight
                            }
                        }

                        RowLayout {
                            visible: root.xAxisMode === "time"
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "timeWindow")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: Number(root.timeWindowSeconds).toFixed(2)
                                onEditingFinished: root.applyTimeWindow(text)
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textPrimary
                                horizontalAlignment: Text.AlignRight
                                selectByMouse: true
                                validator: DoubleValidator { bottom: 0.05; decimals: 3 }
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                            }
                        }

                        Text {
                            Layout.fillWidth: true
                            text: I18n.t(root.language, "yAxis")
                            font.pixelSize: root.baseFontSize
                            font.bold: true
                            color: root.appPalette.textPrimary
                            elide: Text.ElideRight
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "seriesNumber")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: String(root.ySeriesNumber)
                                onEditingFinished: {
                                    const parsed = Number(text)
                                    root.ySeriesNumber = isFinite(parsed) && parsed >= 1 ? Math.round(parsed) : 1
                                }
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textPrimary
                                horizontalAlignment: Text.AlignRight
                                selectByMouse: true
                                validator: IntValidator { bottom: 1; top: 16 }
                                background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Text {
                                Layout.preferredWidth: root.settingsLabelColumnWidth
                                text: I18n.t(root.language, "showTickLabels")
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textSecondary
                                elide: Text.ElideRight
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 8

                                SelectableButton {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    implicitHeight: 34
                                    label: I18n.t(root.language, "on")
                                    selected: root.yShowTickLabels
                                    appPalette: root.appPalette
                                    labelPixelSize: root.baseFontSize - 1
                                    onClicked: root.yShowTickLabels = true
                                }

                                SelectableButton {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    implicitHeight: 34
                                    label: I18n.t(root.language, "off")
                                    selected: !root.yShowTickLabels
                                    appPalette: root.appPalette
                                    labelPixelSize: root.baseFontSize - 1
                                    onClicked: root.yShowTickLabels = false
                                }
                            }
                        }

                        Repeater {
                            model: root.ySeries

                            delegate: Rectangle {
                                id: seriesCard
                                readonly property int seriesIndex: index
                                readonly property var seriesData: modelData
                                readonly property var availableFieldOptions: root.availableFieldsForSeries(seriesIndex)

                                Layout.fillWidth: true
                                implicitHeight: 186
                                radius: 10
                                color: root.appPalette.elevatedBackground
                                border.color: root.appPalette.border
                                border.width: 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 10
                                    spacing: 8

                                    Text {
                                        Layout.fillWidth: true
                                        text: I18n.t(root.language, "series") + " " + (seriesCard.seriesIndex + 1)
                                        font.pixelSize: root.baseFontSize
                                        font.bold: true
                                        color: root.appPalette.textPrimary
                                        elide: Text.ElideRight
                                    }

                                    RowLayout {
                                        Layout.fillWidth: true
                                        spacing: 8

                                        Text {
                                            Layout.preferredWidth: root.seriesLabelColumnWidth
                                            text: I18n.t(root.language, "label")
                                            font.pixelSize: root.baseFontSize
                                            color: root.appPalette.textSecondary
                                            elide: Text.ElideRight
                                        }

                                        ComboBox {
                                            id: seriesFieldCombo
                                            Layout.fillWidth: true
                                            model: seriesCard.availableFieldOptions
                                            textRole: "display"
                                            currentIndex: root.indexOfValue(seriesCard.availableFieldOptions, seriesCard.seriesData.field, "path")
                                            onActivated: {
                                                const options = seriesCard.availableFieldOptions
                                                if (options.length > currentIndex)
                                                    root.setSeriesField(seriesCard.seriesIndex, options[currentIndex].path)
                                            }
                                            palette.window: root.appPalette.inputBackground
                                            palette.button: root.appPalette.inputBackground
                                            palette.base: root.appPalette.inputBackground
                                            palette.text: root.appPalette.textPrimary
                                            palette.buttonText: root.appPalette.textPrimary
                                            palette.highlight: root.appPalette.accent
                                            palette.highlightedText: root.appPalette.accentText
                                            background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                            contentItem: Text { leftPadding: 12; rightPadding: 34; text: seriesFieldCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                                        }
                                    }

                                    RowLayout {
                                        Layout.fillWidth: true
                                        spacing: 8

                                        Text {
                                            Layout.preferredWidth: root.seriesLabelColumnWidth
                                            text: I18n.t(root.language, "color")
                                            font.pixelSize: root.baseFontSize
                                            color: root.appPalette.textSecondary
                                            elide: Text.ElideRight
                                        }

                                        Rectangle {
                                            Layout.preferredWidth: 34
                                            Layout.preferredHeight: 28
                                            radius: 6
                                            color: seriesCard.seriesData.color
                                            border.color: root.appPalette.controlBorder
                                            border.width: 1

                                            MouseArea {
                                                anchors.fill: parent
                                                cursorShape: Qt.PointingHandCursor
                                                onClicked: {
                                                    const color = RosUi.pickColor(
                                                        I18n.t(root.language, "color"),
                                                        seriesCard.seriesData.color)
                                                    if (color.length > 0)
                                                        root.setSeriesColor(seriesCard.seriesIndex, color)
                                                }
                                            }
                                        }

                                        TextField {
                                            Layout.fillWidth: true
                                            text: seriesCard.seriesData.color
                                            onEditingFinished: root.setSeriesColor(seriesCard.seriesIndex, text)
                                            font.pixelSize: root.baseFontSize
                                            color: root.appPalette.textPrimary
                                            selectByMouse: true
                                            background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                        }
                                    }

                                    RowLayout {
                                        Layout.fillWidth: true
                                        spacing: 8

                                        Text {
                                            Layout.preferredWidth: root.seriesLabelColumnWidth
                                            text: I18n.t(root.language, "lineWidth")
                                            font.pixelSize: root.baseFontSize
                                            color: root.appPalette.textSecondary
                                            elide: Text.ElideRight
                                        }

                                        TextField {
                                            Layout.fillWidth: true
                                            text: Number(seriesCard.seriesData.lineWidth).toString()
                                            onEditingFinished: root.setSeriesLineWidth(seriesCard.seriesIndex, text)
                                            font.pixelSize: root.baseFontSize
                                            color: root.appPalette.textPrimary
                                            horizontalAlignment: Text.AlignRight
                                            selectByMouse: true
                                            validator: DoubleValidator { bottom: 0.1; decimals: 2 }
                                            background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                                        }
                                    }
                                }
                            }
                        }

                        Item { Layout.fillHeight: true }
                    }
                }
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 8

                PlotCanvas {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumWidth: 0
                    Layout.minimumHeight: 0
                    appPalette: root.appPalette
                    language: root.language
                    samples: root.displayedPlotSamples
                    plotMode: root.plotMode
                    timestampMode: root.timestampMode
                    xField: root.xField
                    yFields: root.selectedYFields
                    seriesOptions: root.activeSeriesOptions
                    fieldOptions: root.fieldOptions
                    xLabel: root.plotMode === "xy" ? root.fieldLabel(root.xField) : root.timeAxisLabel()
                    axisScaleMode: root.axisScaleMode
                    showXAxisTickLabels: root.xShowTickLabels
                    showYAxisTickLabels: root.yShowTickLabels
                    playbackCurrentTimeMs: RosUi.playbackCurrentTimeMs
                    playbackStartTimeMs: RosUi.playbackStartTimeMs
                    showPlaybackMarker: root.dataSource === "recorded"
                    viewKey: root.dataSource
                }

                Rectangle {
                    visible: root.dataSource === "recorded"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 58
                    radius: 12
                    color: root.appPalette.sidebarBackground
                    border.color: root.appPalette.border
                    border.width: 1

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 10

                        SelectableButton {
                            Layout.preferredWidth: 82
                            implicitHeight: 38
                            label: RosUi.playbackPlaying ? I18n.t(root.language, "pause") : I18n.t(root.language, "play")
                            selected: RosUi.playbackPlaying
                            appPalette: root.appPalette
                            labelPixelSize: root.baseFontSize
                            onClicked: root.openAndPlayRecorded()
                        }

                        Text {
                            Layout.preferredWidth: 120
                            text: root.formatAbsoluteTime(RosUi.playbackCurrentTimeMs)
                            font.pixelSize: root.baseFontSize
                            color: root.appPalette.textPrimary
                            elide: Text.ElideRight
                        }

                        Slider {
                            id: playbackSlider
                            Layout.fillWidth: true
                            from: RosUi.playbackStartTimeMs
                            to: Math.max(RosUi.playbackEndTimeMs, RosUi.playbackStartTimeMs + 1)
                            value: RosUi.playbackCurrentTimeMs
                            onMoved: RosUi.setPlaybackCurrentTimeMs(value)
                        }

                        Text {
                            Layout.preferredWidth: 120
                            text: root.formatAbsoluteTime(RosUi.playbackEndTimeMs)
                            font.pixelSize: root.baseFontSize
                            color: root.appPalette.textSecondary
                            horizontalAlignment: Text.AlignRight
                            elide: Text.ElideRight
                        }
                    }
                }
            }
        }

        Rectangle {
            id: topicsPanel
            visible: root.plotSection === "topics"
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 12
            color: root.appPalette.sidebarBackground
            border.color: root.appPalette.border
            border.width: 1
            clip: true

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 12

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Text {
                        Layout.fillWidth: true
                        text: root.topicsHeadingText()
                        font.pixelSize: root.baseFontSize + 4
                        font.bold: true
                        color: root.appPalette.textPrimary
                        elide: Text.ElideRight
                    }

                    SelectableButton {
                        Layout.preferredWidth: 120
                        implicitHeight: 38
                        label: I18n.t(root.language, "refresh")
                        selected: false
                        appPalette: root.appPalette
                        onClicked: RosUi.refreshPlotTopics()
                    }
                }

                Text {
                    Layout.fillWidth: true
                    text: root.topicSummaryText()
                    font.pixelSize: root.baseFontSize
                    color: root.appPalette.textSecondary
                    elide: Text.ElideRight
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Text {
                        text: I18n.t(root.language, "topicSort")
                        font.pixelSize: root.baseFontSize
                        color: root.appPalette.textSecondary
                        elide: Text.ElideRight
                    }

                    ComboBox {
                        id: topicSortCombo
                        Layout.preferredWidth: Math.max(240, Math.min(360, topicsPanel.width * 0.30))
                        model: root.topicSortOptions
                        textRole: "label"
                        currentIndex: root.indexOfValue(root.topicSortOptions, root.topicSortMode)
                        onActivated: root.topicSortMode = root.topicSortOptions[currentIndex].value
                        palette.window: root.appPalette.inputBackground
                        palette.button: root.appPalette.inputBackground
                        palette.base: root.appPalette.inputBackground
                        palette.text: root.appPalette.textPrimary
                        palette.buttonText: root.appPalette.textPrimary
                        palette.highlight: root.appPalette.accent
                        palette.highlightedText: root.appPalette.accentText
                        background: Rectangle { radius: 8; color: root.appPalette.inputBackground; border.color: root.appPalette.controlBorder; border.width: 1 }
                        contentItem: Text { leftPadding: 12; rightPadding: 34; text: topicSortCombo.displayText; color: root.appPalette.textPrimary; font.pixelSize: root.baseFontSize; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                    }

                    Item { Layout.fillWidth: true }
                }

                Text {
                    Layout.fillWidth: true
                    text: root.topicsHintText()
                    wrapMode: Text.WordWrap
                    font.pixelSize: root.baseFontSize - 1
                    color: root.appPalette.textSecondary
                }

                Item {
                    id: topicColumns
                    Layout.fillWidth: true
                    Layout.preferredHeight: 0
                    visible: false
                    readonly property int rowSpacing: topicsPanel.width < 720 ? 6 : (topicsPanel.width < 1000 ? 8 : 12)
                    readonly property int checkWidth: topicsPanel.width < 720 ? 32 : 44
                    readonly property int rowHorizontalMargins: 20
                    readonly property int panelHorizontalMargins: 32
                    readonly property int dataColumnCount: 3
                    readonly property int availableDataWidth: Math.max(0,
                        topicsPanel.width - panelHorizontalMargins - rowHorizontalMargins
                        - checkWidth - rowSpacing * 4)
                    readonly property int capabilityTargetWidth: Math.ceil(Math.max(
                        capabilityHeaderProbe.implicitWidth,
                        capabilityValueProbe.implicitWidth,
                        capabilityRequiredProbe.implicitWidth) + 16)
                    readonly property int reservedCapabilityWidth: Math.min(
                        capabilityTargetWidth, availableDataWidth)
                    readonly property int dataColumnWidth: Math.max(0, Math.floor(
                        (availableDataWidth - reservedCapabilityWidth) / dataColumnCount))
                    readonly property int capabilityWidth: Math.max(0,
                        availableDataWidth - dataColumnWidth * dataColumnCount)
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    radius: 8
                    color: root.appPalette.elevatedBackground
                    border.color: root.appPalette.border
                    border.width: 1

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 12
                        spacing: topicColumns.rowSpacing

                        Text { Layout.preferredWidth: topicColumns.checkWidth; text: "" }
                        Text { Layout.preferredWidth: topicColumns.dataColumnWidth; text: I18n.t(root.language, "topicName"); font.pixelSize: root.baseFontSize; font.bold: true; color: root.appPalette.textPrimary; elide: Text.ElideRight }
                        Text { Layout.preferredWidth: topicColumns.dataColumnWidth; text: I18n.t(root.language, "topicType"); font.pixelSize: root.baseFontSize; font.bold: true; color: root.appPalette.textPrimary; elide: Text.ElideRight }
                        Text { Layout.preferredWidth: topicColumns.dataColumnWidth; text: I18n.t(root.language, "topicSource"); font.pixelSize: root.baseFontSize; font.bold: true; color: root.appPalette.textPrimary; elide: Text.ElideRight }
                        Text { Layout.preferredWidth: topicColumns.capabilityWidth; text: I18n.t(root.language, "topicCapability"); font.pixelSize: root.baseFontSize; font.bold: true; color: root.appPalette.textPrimary; horizontalAlignment: Text.AlignRight; elide: Text.ElideRight }
                    }
                }

                ListView {
                    id: topicListView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 6
                    clip: true
                    model: root.sortedPlotTopics()
                    boundsBehavior: Flickable.StopAtBounds
                    ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

                    function restoreContentY(savedContentY) {
                        const maxContentY = Math.max(0, contentHeight - height)
                        contentY = Math.max(0, Math.min(savedContentY, maxContentY))
                    }

                    function setTopicSelectedWithoutJump(topicKey, selected) {
                        const savedContentY = contentY
                        RosUi.setPlotTopicSelected(topicKey, selected)
                        restoreContentY(savedContentY)
                        Qt.callLater(function() { topicListView.restoreContentY(savedContentY) })
                    }

                    delegate: Rectangle {
                        width: topicListView.width
                        height: 58
                        radius: 8
                        color: root.appPalette.elevatedBackground
                        border.color: modelData.selected ? root.appPalette.accent : root.appPalette.border
                        border.width: modelData.selected ? 2 : 1

                        RowLayout {
                            anchors.fill: parent
                            anchors.leftMargin: 8
                            anchors.rightMargin: 12
                            spacing: topicColumns.rowSpacing

                            CheckBox {
                                id: topicCheckBox
                                Layout.preferredWidth: topicColumns.checkWidth
                                checked: !!modelData.selected
                                onClicked: topicListView.setTopicSelectedWithoutJump(modelData.key, checked)

                                indicator: Rectangle {
                                    implicitWidth: 22
                                    implicitHeight: 22
                                    x: 4
                                    y: (topicCheckBox.height - height) / 2
                                    radius: 4
                                    color: topicCheckBox.checked ? root.appPalette.accent : root.appPalette.inputBackground
                                    border.color: topicCheckBox.checked ? root.appPalette.accentBorder : root.appPalette.controlBorder
                                    border.width: 1

                                    Text {
                                        anchors.centerIn: parent
                                        text: topicCheckBox.checked ? "✓" : ""
                                        font.pixelSize: 15
                                        font.bold: true
                                        color: root.appPalette.accentText
                                    }
                                }
                            }

                            Text {
                                Layout.preferredWidth: topicColumns.dataColumnWidth
                                text: modelData.name
                                font.pixelSize: root.baseFontSize
                                color: root.appPalette.textPrimary
                                elide: Text.ElideMiddle
                            }

                            Text {
                                Layout.preferredWidth: topicColumns.dataColumnWidth
                                text: modelData.type && modelData.type.length > 0 ? modelData.type : "--"
                                font.pixelSize: root.baseFontSize - 1
                                color: root.appPalette.textSecondary
                                elide: Text.ElideMiddle
                            }

                            Text {
                                Layout.preferredWidth: topicColumns.dataColumnWidth
                                text: root.topicSourceText(modelData)
                                font.pixelSize: root.baseFontSize - 1
                                color: root.appPalette.textSecondary
                                elide: Text.ElideMiddle
                            }

                            Text {
                                Layout.preferredWidth: topicColumns.capabilityWidth
                                text: root.topicCapabilityText(modelData)
                                font.pixelSize: root.baseFontSize - 1
                                color: modelData.plottable ? root.appPalette.textPrimary : root.appPalette.textSecondary
                                horizontalAlignment: Text.AlignRight
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
            }
        }
    }
}
