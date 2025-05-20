package com.nurimaps.feature.presentation.components

import com.nurimaps.feature.ble.domain.model.BluetoothDeviceModel

data class DeviceConnectState(
    val scannedDevices: List<BluetoothDeviceModel> = emptyList(),
    val pairedDevices: List<BluetoothDeviceModel> = emptyList(),
    val receivedValues: List<String> = emptyList(),
    val message: String? = null,
    val isConnected: Boolean = false,
    val isPairing: Boolean = false,
    val connectedDeviceName: String = ""
)