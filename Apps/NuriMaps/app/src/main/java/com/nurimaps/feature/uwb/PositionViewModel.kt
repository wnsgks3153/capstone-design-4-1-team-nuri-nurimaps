package com.nurimaps.feature.uwb

import android.util.Log
import androidx.lifecycle.ViewModel
import com.nurimaps.feature.ble.domain.BLEController
import dagger.hilt.android.lifecycle.HiltViewModel
import jakarta.inject.Inject
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

data class CustomLocationState(
    val isCustomLocationAvailable: Boolean,
    val latLng: Pair<Double, Double>? = null
)

@HiltViewModel
class PositionViewModel @Inject constructor(
    private val bleController: BLEController,
) : ViewModel() {

    private val baseLat: Double = 36.16786
    private val baseLng: Double = 128.46734

    private val _customLocationState = MutableStateFlow(CustomLocationState(false, null))
    val customLocationState: StateFlow<CustomLocationState> = _customLocationState

    companion object {
        private const val TAG = "PositionViewModel"
    }

    init {
        bleController.setOnDataReceivedListener { rawData ->
            Log.d(TAG, "BLE data received: $rawData")
            onBleDataReceived(rawData)
        }
    }

    fun onBleDataReceived(rawData: String) {
        Log.d(TAG, "Parsing raw data...")
        val parsedList = BleDataParser.parse(rawData)
        Log.d(TAG, "Parsed list size: ${parsedList.size}")

        if (parsedList.size >= 3) {
            val position = PositionCalculator.calculatePosition(parsedList)
            if (position != null) {
                Log.d(TAG, "Position calculated: x=${position.x}, y=${position.y}")
                val latLng = convertLocalToLatLng(baseLat, baseLng, position.x, position.y)
                Log.d(TAG, "Converted latLng: lat=${latLng.first}, lng=${latLng.second}")
                _customLocationState.value = CustomLocationState(true, latLng)
            } else {
                Log.w(TAG, "Position calculation failed.")
                _customLocationState.value = CustomLocationState(false, null)
            }
        } else {
            Log.w(TAG, "Insufficient parsed data. Size: ${parsedList.size}")
            _customLocationState.value = CustomLocationState(false, null)
        }
    }

    private fun convertLocalToLatLng(baseLat: Double, baseLng: Double, x: Double, y: Double): Pair<Double, Double> {
        val earthRadius = 6378137.0
        val deltaLat = y / earthRadius * (180 / Math.PI)
        val deltaLng = x / (earthRadius * Math.cos(baseLat * Math.PI / 180)) * (180 / Math.PI)

        val lat = baseLat + deltaLat
        val lng = baseLng + deltaLng

        Log.d(TAG, "convertLocalToLatLng: x=$x, y=$y -> lat=$lat, lng=$lng")
        return Pair(lat, lng)
    }
}