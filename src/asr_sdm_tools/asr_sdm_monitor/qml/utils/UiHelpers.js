function display(v, fallback) {
    if (v === undefined || v === null || v === "")
        return fallback === undefined ? "--" : fallback
    return v
}

function maxValueOf(listA, listB) {
    let maxValue = 0
    if (listA) {
        for (let i = 0; i < listA.length; ++i)
            maxValue = Math.max(maxValue, Number(listA[i]))
    }
    if (listB) {
        for (let j = 0; j < listB.length; ++j)
            maxValue = Math.max(maxValue, Number(listB[j]))
    }
    return maxValue
}

function netAdaptiveMax(listA, listB) {
    const peak = maxValueOf(listA, listB)
    if (peak <= 1.0)
        return 1.0
    if (peak <= 10.0)
        return 10.0
    if (peak <= 100.0)
        return 100.0
    return Math.ceil(peak / 100.0) * 100.0
}
