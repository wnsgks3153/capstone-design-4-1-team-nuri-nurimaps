package com.nurimaps.feature.uwb

class AnchorDistanceCache(private val maxSize: Int = 3) {

    private val anchorMap = linkedMapOf<Int, AnchorDistance>()

    fun update(anchorDistance: AnchorDistance) {
        val id = anchorDistance.anchor.id

        // 기존에 존재하면 삭제 (나중에 다시 넣어서 최근으로)
        anchorMap.remove(id)
        anchorMap[id] = anchorDistance

        // 초과하면 가장 오래된 것 제거
        if (anchorMap.size > maxSize) {
            val oldestKey = anchorMap.entries.first().key
            anchorMap.remove(oldestKey)
        }
    }

    fun getAnchors(): List<AnchorDistance> {
        return anchorMap.values.toList()
    }

    fun clear() {
        anchorMap.clear()
    }
}
