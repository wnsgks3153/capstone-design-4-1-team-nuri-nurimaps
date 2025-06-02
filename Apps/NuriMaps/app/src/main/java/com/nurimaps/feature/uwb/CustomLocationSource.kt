package com.nurimaps.feature.uwb

import android.location.Location
import android.util.Log
import com.naver.maps.map.LocationSource
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch

class CustomLocationSource(
    private val gpsLocationSource: LocationSource,
    private val positionViewModel: PositionViewModel
) : LocationSource {

    private var onLocationChangedListener: LocationSource.OnLocationChangedListener? = null
    private var coroutineScope: CoroutineScope? = null
    private var collectJob: Job? = null

    companion object {
        private const val TAG = "CustomLocationSource"
    }

    override fun activate(listener: LocationSource.OnLocationChangedListener) {
        Log.d(TAG, "activate() called")
        onLocationChangedListener = listener

        gpsLocationSource.activate { location ->
            Log.d(TAG, "GPS location update received: lat=${location?.latitude}, lng=${location?.longitude}")
            val customState = positionViewModel.customLocationState.value
            if (customState.isCustomLocationAvailable && customState.latLng != null) {
                Log.d(TAG, "Using custom location: lat=${customState.latLng.first}, lng=${customState.latLng.second}")
                val customLocation = Location("custom").apply {
                    latitude = customState.latLng.first
                    longitude = customState.latLng.second
                    location?.let {
                        accuracy = it.accuracy
                        time = System.currentTimeMillis()
                        if (it.hasBearing()) bearing = it.bearing
                        if (it.hasSpeed()) speed = it.speed
                        if (it.hasAltitude()) altitude = it.altitude
                    }
                }
                listener.onLocationChanged(customLocation)
            } else {
                Log.d(TAG, "Using GPS location as fallback")
                location?.let {
                    listener.onLocationChanged(it)
                } ?: run {
                    Log.w(TAG, "GPS location is null, cannot update location")
                }
            }
        }

        coroutineScope = CoroutineScope(Dispatchers.Main)
        collectJob = coroutineScope?.launch {
            positionViewModel.customLocationState.collect { customState ->
                if (customState.isCustomLocationAvailable && customState.latLng != null) {
                    Log.d(TAG, "Collect update - new custom location: lat=${customState.latLng.first}, lng=${customState.latLng.second}")
                    val customLocation = Location("custom").apply {
                        latitude = customState.latLng.first
                        longitude = customState.latLng.second
                        accuracy = 10f
                        time = System.currentTimeMillis()
                        customState.altitude?.let { altitude = it }
                    }
                    onLocationChangedListener?.onLocationChanged(customLocation)
                } else {
                    Log.d(TAG, "Collect update - no custom location available")
                }
            }
        }
    }

    override fun deactivate() {
        Log.d(TAG, "deactivate() called")
        gpsLocationSource.deactivate()
        collectJob?.cancel()
        coroutineScope = null
        onLocationChangedListener = null
    }
}