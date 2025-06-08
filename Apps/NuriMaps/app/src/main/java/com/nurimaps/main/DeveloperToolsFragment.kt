package com.nurimaps.main

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.CheckBox
import android.widget.TextView
import androidx.core.app.ActivityCompat
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.lifecycleScope
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.nurimaps.feature.uwb.PositionViewModel
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

class DeveloperToolsFragment : Fragment() {

    private lateinit var fusedLocationClient: FusedLocationProviderClient
    private lateinit var gpsTextView: TextView
    private lateinit var locationCallback: LocationCallback
    private lateinit var locationRequest: LocationRequest

    private lateinit var calculatedTextView: TextView
    private lateinit var checkBoxShowDistances: CheckBox
    private lateinit var checkBoxShowAnchors: CheckBox

    private val positionViewModel: PositionViewModel by activityViewModels()

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.developer_tools, container, false)
        super.onViewCreated(view, savedInstanceState)

        gpsTextView = view.findViewById(R.id.text_gps_position)
        calculatedTextView = view.findViewById(R.id.text_calculated_position)
        checkBoxShowDistances = view.findViewById(R.id.checkbox_show_distances)
        checkBoxShowAnchors = view.findViewById(R.id.checkbox_show_anchor_markers)

        fusedLocationClient = LocationServices.getFusedLocationProviderClient(requireActivity())

        locationRequest = LocationRequest.Builder(
            Priority.PRIORITY_HIGH_ACCURACY, 3000L
        ).apply {
            setMinUpdateIntervalMillis(1000L)
        }.build()

        locationCallback = object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                val location = result.lastLocation
                if (location != null) {
                    val lat = location.latitude
                    val lng = location.longitude
                    val alt = location.altitude
                    gpsTextView.text = "위치: $lat, $lng\n고도: $alt"
                }
            }
        }

        requestLocation()
        observeCalculatedPosition()
        setupCheckboxes()

        return view
    }

    private fun requestLocation() {
        if (ActivityCompat.checkSelfPermission(requireContext(), Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            ActivityCompat.checkSelfPermission(requireContext(), Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            gpsTextView.text = "위치 권한이 없습니다."
            return
        }
    }

    private fun observeCalculatedPosition() {
        lifecycleScope.launch {
            positionViewModel.customLocationState.collectLatest { state ->
                if (state.isCustomLocationAvailable && state.latLng != null && state.altitude != null) {
                    calculatedTextView.text = "위치: ${state.latLng.first}, ${state.latLng.second}\n고도: ${state.altitude}"
                } else {
                    calculatedTextView.text = "위치 계산 불가"
                }
            }
        }
    }

    private fun setupCheckboxes() {
        checkBoxShowDistances.setOnCheckedChangeListener { _, isChecked ->
            // 체크 상태 ViewModel이나 MapFragment에 전달 필요
        }

        checkBoxShowAnchors.setOnCheckedChangeListener { _, isChecked ->
            // 체크 상태 ViewModel이나 MapFragment에 전달 필요
        }
    }

    override fun onResume() {
        super.onResume()
        if (ActivityCompat.checkSelfPermission(requireContext(), Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            fusedLocationClient.requestLocationUpdates(locationRequest, locationCallback, null)
        }
    }

    override fun onPause() {
        super.onPause()
        fusedLocationClient.removeLocationUpdates(locationCallback)
    }

}