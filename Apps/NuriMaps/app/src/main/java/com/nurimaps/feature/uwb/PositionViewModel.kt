package com.nurimaps.feature.uwb

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

class PositionViewModel(
    private val baseLat: Double = 36.16786,
    private val baseLng: Double = 128.46734
) : ViewModel() {

    private val _currentPosition = MutableStateFlow<Position?>(null)
    val currentPosition: StateFlow<Position?> = _currentPosition

    private val _currentLatLng = MutableStateFlow<Pair<Double, Double>?>(null)
    val currentLatLng: StateFlow<Pair<Double, Double>?> = _currentLatLng

    fun onBleDataReceived(rawData: String) {
        val parsedList = BleDataParser.parse(rawData)
        if (parsedList.size >= 3) {
            val position = PositionCalculator.calculatePosition(parsedList)
            _currentPosition.value = position

            position?.let {
                val latLng = convertLocalToLatLng(baseLat, baseLng, it.x, it.y)
                _currentLatLng.value = latLng
            }
        }
    }

    private fun convertLocalToLatLng(baseLat: Double, baseLng: Double, x: Double, y: Double): Pair<Double, Double> {
        val earthRadius = 6378137.0
        val deltaLat = y / earthRadius * (180 / Math.PI)
        val deltaLng = x / (earthRadius * Math.cos(baseLat * Math.PI / 180)) * (180 / Math.PI)

        val lat = baseLat + deltaLat
        val lng = baseLng + deltaLng
        return Pair(lat, lng)
    }
}