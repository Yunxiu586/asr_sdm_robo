import QtQuick
import QtQuick.Layouts
import "../utils/I18n.js" as I18n

Rectangle {
    id: root
    property var appPalette
    property string language: "en"
    property var samples: []
    property string plotMode: "time"
    property string timestampMode: "relative"
    property string xField: ""
    property var yFields: []
    property var seriesOptions: []
    property var fieldOptions: []
    property string xLabel: ""
    property string axisScaleMode: "independent"
    property bool showXAxisTickLabels: true
    property bool showYAxisTickLabels: true
    property double playbackCurrentTimeMs: 0
    property double playbackStartTimeMs: 0
    property bool showPlaybackMarker: false

    radius: 12
    color: appPalette.surfaceBackground
    border.color: appPalette.border
    border.width: 1
    clip: true

    function isFiniteNumber(value) {
        const n = Number(value)
        return isFinite(n)
    }

    function paddedRange(minValue, maxValue) {
        if (!isFinite(minValue) || !isFinite(maxValue))
            return { min: -1, max: 1 }
        if (Math.abs(maxValue - minValue) < 1e-9) {
            const padSame = Math.max(1.0, Math.abs(maxValue) * 0.1)
            return { min: minValue - padSame, max: maxValue + padSame }
        }
        const pad = (maxValue - minValue) * 0.08
        return { min: minValue - pad, max: maxValue + pad }
    }

    function zeroPad(value, width) {
        let text = String(value)
        while (text.length < width)
            text = "0" + text
        return text
    }

    function timeLabel(value) {
        if (root.plotMode === "time" && root.timestampMode === "absolute") {
            const date = new Date(Number(value))
            const hh = zeroPad(date.getHours(), 2)
            const mm = zeroPad(date.getMinutes(), 2)
            const ss = zeroPad(date.getSeconds(), 2)
            const ms = zeroPad(date.getMilliseconds(), 3)
            return hh + ":" + mm + ":" + ss + "." + ms
        }
        return Number(value).toFixed(2)
    }

    function fieldLabel(path) {
        for (let i = 0; i < root.fieldOptions.length; ++i) {
            if (root.fieldOptions[i].path === path)
                return root.fieldOptions[i].label || path
        }
        return path
    }

    function fieldUnit(path) {
        for (let i = 0; i < root.fieldOptions.length; ++i) {
            if (root.fieldOptions[i].path === path)
                return root.fieldOptions[i].unit || ""
        }
        return ""
    }

    function paletteSeriesColor(index) {
        const colors = root.appPalette.chartSeriesColors || [
            root.appPalette.primaryChart,
            root.appPalette.secondaryChart,
            root.appPalette.accentBorder,
            "#f2cc60",
            "#c586c0",
            "#ce9178",
            "#b5cea8",
            "#d7ba7d",
            "#ff8c00",
            "#f44747",
            "#569cd6",
            "#86c232"
        ]
        return colors[index % colors.length]
    }

    function normalizedSeries() {
        const result = []
        const configured = root.seriesOptions || []
        if (configured.length > 0) {
            for (let i = 0; i < configured.length; ++i) {
                const item = configured[i] || {}
                const field = String(item.field || "")
                if (field.length === 0)
                    continue
                result.push({
                    field: field,
                    color: item.color || paletteSeriesColor(result.length),
                    lineWidth: Number(item.lineWidth) > 0 ? Number(item.lineWidth) : 1.0
                })
            }
            return result
        }

        const fields = root.yFields || []
        for (let j = 0; j < fields.length; ++j) {
            result.push({ field: fields[j], color: paletteSeriesColor(j), lineWidth: 1.0 })
        }
        return result
    }

    function seriesColor(seriesItem, index) {
        return (seriesItem && seriesItem.color) ? seriesItem.color : paletteSeriesColor(index)
    }

    function seriesLineWidth(seriesItem) {
        return (seriesItem && Number(seriesItem.lineWidth) > 0) ? Number(seriesItem.lineWidth) : 1.0
    }

    Canvas {
        id: canvas
        anchors.fill: parent
        anchors.margins: Math.max(10, Math.min(22, Math.min(root.width, root.height) * 0.04))

        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)

            const selectedSeries = normalizedSeries()
            const source = root.samples || []
            if (selectedSeries.length === 0 || source.length === 0)
                return

            const series = []
            let minX = Number.POSITIVE_INFINITY
            let maxX = Number.NEGATIVE_INFINITY
            let minY = Number.POSITIVE_INFINITY
            let maxY = Number.NEGATIVE_INFINITY

            for (let s = 0; s < selectedSeries.length; ++s) {
                const seriesConfig = selectedSeries[s]
                const fieldPath = seriesConfig.field
                const points = []
                for (let i = 0; i < source.length; ++i) {
                    const sample = source[i]
                    const x = root.plotMode === "xy"
                              ? Number(sample[root.xField])
                              : (root.timestampMode === "absolute" ? Number(sample.absoluteTimeMs) : Number(sample.relativeTime))
                    const y = Number(sample[fieldPath])
                    if (isFinite(x) && isFinite(y)) {
                        points.push({ x: x, y: y })
                        minX = Math.min(minX, x)
                        maxX = Math.max(maxX, x)
                        minY = Math.min(minY, y)
                        maxY = Math.max(maxY, y)
                    }
                }
                if (points.length > 0)
                    series.push({ field: fieldPath, points: points, color: seriesConfig.color, lineWidth: seriesConfig.lineWidth })
            }

            if (series.length === 0 || !isFinite(minX) || !isFinite(maxX) || !isFinite(minY) || !isFinite(maxY))
                return

            const w = width
            const h = height
            const left = root.showYAxisTickLabels ? Math.max(62, Math.min(96, w * 0.12)) : Math.max(30, Math.min(44, w * 0.06))
            const right = w - Math.max(18, Math.min(36, w * 0.04))
            const legendStartX = left
            const legendStartY = 8
            const legendGap = 18
            const legendRowHeight = 20
            const legendMarkerWidth = 18
            const legendTextOffset = 26
            const availableLegendWidth = Math.max(1, right - left)
            const legendItems = []

            ctx.font = "13px sans-serif"
            for (let itemIndex = 0; itemIndex < series.length; ++itemIndex) {
                const legendField = series[itemIndex].field
                const legendText = fieldLabel(legendField) + (fieldUnit(legendField) ? " (" + fieldUnit(legendField) + ")" : "")
                const measured = ctx.measureText(legendText).width + legendTextOffset + 8
                legendItems.push({ text: legendText, width: measured })
            }

            let maxLegendItemWidth = 120
            for (let m = 0; m < legendItems.length; ++m)
                maxLegendItemWidth = Math.max(maxLegendItemWidth, legendItems[m].width)

            maxLegendItemWidth = Math.min(maxLegendItemWidth, Math.max(120, availableLegendWidth))
            let legendColumns = Math.max(1, Math.floor((availableLegendWidth + legendGap) / (maxLegendItemWidth + legendGap)))
            legendColumns = Math.max(1, Math.min(series.length, legendColumns))
            const legendRows = Math.ceil(series.length / legendColumns)
            const legendCellWidth = Math.max(120, (availableLegendWidth - legendGap * (legendColumns - 1)) / legendColumns)
            const legendHeight = legendRows * legendRowHeight

            const bottom = h - (root.showXAxisTickLabels ? Math.max(46, Math.min(70, h * 0.12)) : Math.max(30, Math.min(42, h * 0.08)))
            let top = Math.max(42, legendStartY + legendHeight + 14)
            if (bottom - top < 80)
                top = Math.max(42, bottom - 80)
            const chartW = Math.max(1, right - left)
            const chartH = Math.max(1, bottom - top)

            const xr = paddedRange(minX, maxX)
            const yr = paddedRange(minY, maxY)
            minX = xr.min
            maxX = xr.max
            minY = yr.min
            maxY = yr.max

            if (root.plotMode === "xy" && root.axisScaleMode === "square") {
                const xCenter = (minX + maxX) / 2
                const yCenter = (minY + maxY) / 2
                const halfRange = Math.max(maxX - minX, maxY - minY) / 2
                minX = xCenter - halfRange
                maxX = xCenter + halfRange
                minY = yCenter - halfRange
                maxY = yCenter + halfRange
            }

            function px(x) {
                return left + chartW * (x - minX) / Math.max(1e-12, maxX - minX)
            }

            function py(y) {
                return bottom - chartH * (y - minY) / Math.max(1e-12, maxY - minY)
            }

            function fitText(text, maxWidth) {
                if (ctx.measureText(text).width <= maxWidth)
                    return text
                let result = text
                while (result.length > 1 && ctx.measureText(result + "…").width > maxWidth)
                    result = result.slice(0, -1)
                return result + "…"
            }

            ctx.strokeStyle = root.appPalette.chartGrid
            ctx.lineWidth = 1
            ctx.fillStyle = root.appPalette.textSecondary
            ctx.font = "12px sans-serif"

            const verticalTicks = 5
            const horizontalTicks = 5

            for (let gx = 0; gx < verticalTicks; ++gx) {
                const t = gx / Math.max(1, verticalTicks - 1)
                const xPos = left + chartW * t
                const value = minX + (maxX - minX) * t
                ctx.beginPath()
                ctx.moveTo(xPos, top)
                ctx.lineTo(xPos, bottom)
                ctx.stroke()
                if (root.showXAxisTickLabels) {
                    ctx.textAlign = "center"
                    ctx.textBaseline = "top"
                    ctx.fillText(timeLabel(value), xPos, bottom + 10)
                }
            }

            for (let gy = 0; gy < horizontalTicks; ++gy) {
                const t2 = gy / Math.max(1, horizontalTicks - 1)
                const yPos = top + chartH * t2
                const value2 = maxY - (maxY - minY) * t2
                ctx.beginPath()
                ctx.moveTo(left, yPos)
                ctx.lineTo(right, yPos)
                ctx.stroke()
                if (root.showYAxisTickLabels) {
                    ctx.textAlign = "right"
                    ctx.textBaseline = "middle"
                    ctx.fillText(Number(value2).toFixed(3), left - 10, yPos)
                }
            }

            ctx.strokeStyle = root.appPalette.textSecondary
            ctx.lineWidth = 1.2
            ctx.beginPath()
            ctx.moveTo(left, top)
            ctx.lineTo(left, bottom)
            ctx.lineTo(right, bottom)
            ctx.stroke()

            if (root.showPlaybackMarker && root.plotMode === "time" && isFinite(root.playbackCurrentTimeMs)) {
                const markerXValue = root.timestampMode === "absolute"
                                   ? root.playbackCurrentTimeMs
                                   : (root.playbackCurrentTimeMs - root.playbackStartTimeMs) / 1000.0
                if (markerXValue >= minX && markerXValue <= maxX) {
                    const markerX = px(markerXValue)
                    ctx.strokeStyle = root.appPalette.accent
                    ctx.lineWidth = 1.4
                    ctx.beginPath()
                    ctx.moveTo(markerX, top)
                    ctx.lineTo(markerX, bottom)
                    ctx.stroke()
                }
            }

            for (let seriesIndex = 0; seriesIndex < series.length; ++seriesIndex) {
                const line = series[seriesIndex]
                ctx.strokeStyle = seriesColor(line, seriesIndex)
                ctx.lineWidth = seriesLineWidth(line)
                ctx.beginPath()
                for (let k = 0; k < line.points.length; ++k) {
                    const xPix = px(line.points[k].x)
                    const yPix = py(line.points[k].y)
                    if (k === 0)
                        ctx.moveTo(xPix, yPix)
                    else
                        ctx.lineTo(xPix, yPix)
                }
                ctx.stroke()

                if (root.plotMode === "xy") {
                    ctx.fillStyle = seriesColor(line, seriesIndex)
                    const step = Math.max(1, Math.floor(line.points.length / 80))
                    for (let p = 0; p < line.points.length; p += step) {
                        ctx.beginPath()
                        ctx.arc(px(line.points[p].x), py(line.points[p].y), 2.3, 0, Math.PI * 2)
                        ctx.fill()
                    }
                }
            }

            ctx.fillStyle = root.appPalette.textSecondary
            ctx.font = "13px sans-serif"
            ctx.textAlign = "center"
            ctx.textBaseline = "bottom"
            ctx.fillText(root.xLabel, left + chartW / 2, h - 2)

            ctx.textAlign = "left"
            ctx.textBaseline = "top"
            ctx.font = "13px sans-serif"
            for (let legendIndex = 0; legendIndex < series.length; ++legendIndex) {
                const legendRow = Math.floor(legendIndex / legendColumns)
                const legendColumn = legendIndex % legendColumns
                const legendX = legendStartX + legendColumn * (legendCellWidth + legendGap)
                const legendY = legendStartY + legendRow * legendRowHeight
                const legendText = fitText(legendItems[legendIndex].text, Math.max(20, legendCellWidth - legendTextOffset))
                ctx.fillStyle = seriesColor(series[legendIndex], legendIndex)
                ctx.fillRect(legendX, legendY + 7, legendMarkerWidth, Math.max(2, Math.min(6, seriesLineWidth(series[legendIndex]))))
                ctx.fillStyle = root.appPalette.textPrimary
                ctx.fillText(legendText, legendX + legendTextOffset, legendY)
            }
        }

        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
    }

    onSamplesChanged: canvas.requestPaint()
    onPlotModeChanged: canvas.requestPaint()
    onTimestampModeChanged: canvas.requestPaint()
    onXFieldChanged: canvas.requestPaint()
    onYFieldsChanged: canvas.requestPaint()
    onSeriesOptionsChanged: canvas.requestPaint()
    onFieldOptionsChanged: canvas.requestPaint()
    onXLabelChanged: canvas.requestPaint()
    onAxisScaleModeChanged: canvas.requestPaint()
    onShowXAxisTickLabelsChanged: canvas.requestPaint()
    onShowYAxisTickLabelsChanged: canvas.requestPaint()
    onPlaybackCurrentTimeMsChanged: canvas.requestPaint()
    onPlaybackStartTimeMsChanged: canvas.requestPaint()
    onShowPlaybackMarkerChanged: canvas.requestPaint()
    onAppPaletteChanged: canvas.requestPaint()
}
