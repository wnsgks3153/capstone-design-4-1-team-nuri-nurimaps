package com.nurimaps.feature.ble.data.mappers

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import com.nurimaps.feature.ble.domain.model.BluetoothDeviceModel

@SuppressLint("MissingPermission")
fun BluetoothDevice.toBluetoothDeviceModel() = BluetoothDeviceModel(
    name = name,
    address = address
)