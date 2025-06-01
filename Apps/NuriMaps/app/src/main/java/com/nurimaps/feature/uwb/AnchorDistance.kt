package com.nurimaps.feature.uwb

data class AnchorDistance(
    val anchor: Anchor,
    val distance: Double // meter 단위
)

object BleDataParser {

    /**
     * BLE로부터 받은 문자열을 파싱하고, Anchor + 거리로 변환
     * 예: "060102090130040210" → [Anchor(id=6)..., distance=1.02], ...
     */
    fun parse(rawData: String): List<AnchorDistance> {
        val result = mutableListOf<AnchorDistance>()
        var i = 0
        while (i + 5 < rawData.length) {
            try {
                val id = rawData.substring(i, i + 2).toInt()
                val distStr = rawData.substring(i + 2, i + 6)
                val distance = "${distStr.substring(0, 2)}.${distStr.substring(2, 4)}".toDouble()

                val anchor = AnchorRepository.getAnchorById(id)
                if (anchor != null) {
                    result.add(AnchorDistance(anchor, distance))
                }
            } catch (e: Exception) {
                e.printStackTrace() // 오류 시 무시
            }
            i += 6
        }
        return result
    }
}
