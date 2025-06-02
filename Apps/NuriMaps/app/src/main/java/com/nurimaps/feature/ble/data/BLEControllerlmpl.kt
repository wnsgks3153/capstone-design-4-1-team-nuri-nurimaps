package com.nurimaps.feature.ble.data

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothDevice.TRANSPORT_LE
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import com.nurimaps.feature.ble.data.mappers.toBluetoothDeviceModel
import com.nurimaps.feature.ble.data.receivers.PairDeviceReceiver
import com.nurimaps.feature.ble.domain.BLEController
import com.nurimaps.feature.ble.domain.model.BluetoothDeviceModel
import com.nurimaps.feature.uwb.BleDataParser
import com.nurimaps.feature.uwb.PositionCalculator
import com.nurimaps.feature.uwb.PositionViewModel
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import java.util.UUID

@SuppressLint("MissingPermission")
class BLEControllerImpl(
    private val context: Context,
    //private val viewModel: PositionViewModel
) : BLEController {

    private var onDataReceived: ((String) -> Unit)? = null

    override fun setOnDataReceivedListener(listener: (String) -> Unit) {
        onDataReceived = listener
    }

    private val bluetoothManager by lazy {
        context.getSystemService(BluetoothManager::class.java)
    }

    private val bluetoothAdapter by lazy {
        bluetoothManager?.adapter
    }

    private var isScanning = false
    private var isPairDeviceReceiverRegistered = false
    private var bluetoothGatt: BluetoothGatt? = null

    private val _scannedDevices = MutableStateFlow<List<BluetoothDeviceModel>>(emptyList())
    override val scannedDevices: StateFlow<List<BluetoothDeviceModel>>
        get() = _scannedDevices.asStateFlow()

    private val _pairedDevices = MutableStateFlow<List<BluetoothDeviceModel>>(emptyList())
    override val pairedDevices: StateFlow<List<BluetoothDeviceModel>>
        get() = _pairedDevices.asStateFlow()

    private val _receivedValues = MutableStateFlow<List<String>>(emptyList())
    override val receivedValues: StateFlow<List<String>>
        get() = _receivedValues.asStateFlow()

    private val _message = MutableSharedFlow<String>()
    override val message: SharedFlow<String>
        get() = _message.asSharedFlow()

    private val _isConnected = MutableStateFlow(false)
    override val isConnected: StateFlow<Boolean>
        get() = _isConnected.asStateFlow()

    private val _isPairing = MutableStateFlow(false)
    override val isPairing: StateFlow<Boolean>
        get() = _isPairing.asStateFlow()

    private val _connectedDeviceName = MutableStateFlow("")
    override val connectedDeviceName: StateFlow<String>
        get() = _connectedDeviceName.asStateFlow()

    private val leScanCallback: ScanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            super.onScanResult(callbackType, result)
            _scannedDevices.update { devices ->
                val newDevice = result.device.toBluetoothDeviceModel()
                if (newDevice in devices) devices else devices + newDevice
            }
        }
    }

    private var notifyCharacteristic: BluetoothGattCharacteristic? = null

    private val pairDeviceReceiver = PairDeviceReceiver(
        onPairRequest = {
            _isPairing.update { true }
            Log.i("PairDeviceReceiver", "Pairing request sent")
        },
        onPairedSuccessfully = { device ->
            setMessage("Connected to ${device?.address}")

            stopScan()
            getPairedDevices()

            _scannedDevices.update { emptyList() }
            _isPairing.update { false }
        },
        onPairingError = { device ->
            setMessage("Cannot connected to device ${device?.address}")

            stopScan()

            _scannedDevices.update { emptyList() }
            _isPairing.update { false }
        },
        onPairing = {
            _isPairing.update { true }
        }
    )

    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    setMessage("Successfully connected to ${gatt.device.address}")
                    _isConnected.update { true }
                    _connectedDeviceName.update { gatt.device.name }
                    gatt.discoverServices()

                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    setMessage("Successfully disconnected from ${gatt.device.address}")
                    _isConnected.update { false }
                    _connectedDeviceName.update { "" }
                    gatt.close()
                }

            } else {
                setMessage("Error $status encountered for ${gatt.device.address}")
                _isConnected.update { false }
                _connectedDeviceName.update { "" }
                gatt.close()
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                var found = false

                for (service in gatt.services) {
                    for (characteristic in service.characteristics) {
                        val props = characteristic.properties

                        val supportsNotify =
                            props and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0
                        val supportsWrite =
                            props and BluetoothGattCharacteristic.PROPERTY_WRITE != 0 ||
                                    props and BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE != 0

                        if (supportsNotify && supportsWrite) {
                            notifyCharacteristic = characteristic

                            gatt.setCharacteristicNotification(characteristic, true)

                            val descriptor = characteristic.getDescriptor(
                                UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
                            )
                            descriptor?.let {
                                it.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                                gatt.writeDescriptor(it)
                            }

                            setMessage("✅ Found characteristic: ${characteristic.uuid}")
                            found = true
                            break
                        }
                    }
                    if (found) break
                }

                if (!found) setMessage("⚠️ 알림 가능한 characteristic을 찾을 수 없습니다.")
            } else {
                setMessage("Service discovery failed -> status: $status")
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            super.onCharacteristicChanged(gatt, characteristic, value)

            if (characteristic.uuid == notifyCharacteristic?.uuid) {
                val receivedString = value.toString(Charsets.UTF_8)
                _receivedValues.update { values -> values + receivedString }
                onDataReceived?.invoke(receivedString) // ✅ 여기서 ViewModel로 전달됨
            }
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt?,
            characteristic: BluetoothGattCharacteristic?,
            status: Int
        ) {
            super.onCharacteristicWrite(gatt, characteristic, status)
            if (status == BluetoothGatt.GATT_SUCCESS) {
                setMessage("Value sent successfully to server: ${gatt?.device?.address}")
            } else {
                setMessage("Failed to send value -> status: $status")
            }
        }
    }

    init {
        getPairedDevices()
    }

    override fun startScan() {
        if (!hasBluetoothScanPermission()) {
            setMessage("No bluetooth permission")
            return
        }

        if (!isBluetoothEnabled()) {
            setMessage("Bluetooth not enabled")
            return
        }

        if (isScanning) return

        if (!isPairDeviceReceiverRegistered) {
            registerPairDeviceReceiver()
            isPairDeviceReceiverRegistered = true
        }

        getPairedDevices()

        bluetoothAdapter?.bluetoothLeScanner?.startScan(leScanCallback)
        isScanning = true
    }

    @SuppressLint("MissingPermission")
    override fun stopScan() {
        if (!hasBluetoothScanPermission()) {
            setMessage("No bluetooth permission")
            return
        }

        if (!isBluetoothEnabled()) {
            setMessage("Bluetooth not enabled")
            return
        }

        if (!isScanning) return

        bluetoothAdapter?.bluetoothLeScanner?.stopScan(leScanCallback)
        _scannedDevices.update { emptyList() }
        isScanning = false
    }

    override fun release() {
        disConnect()

        if (isPairDeviceReceiverRegistered) context.unregisterReceiver(pairDeviceReceiver)
    }

    override fun pair(address: String) {
        if (!hasBluetoothConnectPermission()) return
        val device = bluetoothAdapter?.getRemoteDevice(address)

        device?.let {
            try {
                device.createBond()
            } catch (e: Exception) {
                setMessage(("Failed to pair with device: ${e.message}"))
            }
        } ?: run {
            setMessage("Device not found with address: $address")
        }
    }

    override fun connect(address: String) {
        val device = bluetoothAdapter?.getRemoteDevice(address)
        bluetoothGatt = device?.connectGatt(context, false, gattCallback, TRANSPORT_LE)

        stopScan()
    }

    override fun disConnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()

        _isConnected.update { false }
        _connectedDeviceName.update { "" }
        _receivedValues.update { emptyList() }

        setMessage("Successfully disconnected from ${bluetoothGatt?.device?.address}")
    }

    override fun sendValue(text: String) {
        val characteristic = notifyCharacteristic

        if (characteristic != null) {
            val data = text.toByteArray(Charsets.UTF_8)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                bluetoothGatt?.writeCharacteristic(
                    characteristic,
                    data,
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                )
            } else {
                @Suppress("DEPRECATION")
                characteristic.value = data
                @Suppress("DEPRECATION")
                bluetoothGatt?.writeCharacteristic(characteristic)
            }
        } else {
            setMessage("알림 가능한 characteristic이 설정되지 않았습니다.")
        }
    }

    private fun getPairedDevices() {
        if (!hasBluetoothConnectPermission()) {
            setMessage("No BLUETOOTH_CONNECT permission")
            return
        }

        bluetoothAdapter
            ?.bondedDevices
            ?.map { it.toBluetoothDeviceModel() }
            ?.also { devices -> _pairedDevices.update { devices } }
    }

    private fun registerPairDeviceReceiver() {
        context.registerReceiver(
            pairDeviceReceiver,
            IntentFilter().apply {
                addAction(BluetoothDevice.ACTION_PAIRING_REQUEST)
                addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED)
            }
        )
    }

    private fun hasBluetoothScanPermission(): Boolean {
        var hasPermission = true
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (!hasPermission(Manifest.permission.BLUETOOTH_SCAN)) {
                hasPermission = false
            }
        } else {
            if (!hasPermission(Manifest.permission.BLUETOOTH)) {
                hasPermission = false
            }
        }
        return hasPermission
    }

    private fun hasBluetoothConnectPermission(): Boolean {
        var hasPermission = true
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (!hasPermission(Manifest.permission.BLUETOOTH_CONNECT)) {
                hasPermission = false
            }
        } else {
            if (!hasPermission(Manifest.permission.BLUETOOTH)) {
                hasPermission = false
            }
        }
        return hasPermission
    }

    private fun isBluetoothEnabled() = bluetoothAdapter?.isEnabled == true

    private fun setMessage(text: String) {
        CoroutineScope(Dispatchers.IO).launch {
            _message.emit(text)
        }
    }

    private fun hasPermission(permission: String) =
        context.checkSelfPermission(permission) == PackageManager.PERMISSION_GRANTED
}