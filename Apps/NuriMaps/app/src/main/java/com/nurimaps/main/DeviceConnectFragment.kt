package com.nurimaps.main

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.FrameLayout
import android.widget.Toast
import androidx.activity.addCallback
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.Toolbar
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentContainerView
import androidx.fragment.app.activityViewModels
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.google.android.material.snackbar.Snackbar
import com.nurimaps.feature.presentation.components.DeviceAdapter
import com.nurimaps.feature.presentation.components.DeviceConnectState
import com.nurimaps.feature.presentation.components.DeviceConnectViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch

@AndroidEntryPoint
class DeviceConnectFragment : Fragment(){

    private val viewModel: DeviceConnectViewModel by activityViewModels()
    private var hasNavigatedToCommunication = false
    private val _hasNavigatedToChat = MutableStateFlow(false)
    val hasNavigatedToChat: StateFlow<Boolean> = _hasNavigatedToChat

    fun markNavigatedToChat() {
        _hasNavigatedToChat.value = true
    }

    fun resetNavigationFlag() {
        _hasNavigatedToChat.value = false
    }

    private lateinit var pairedRecyclerView: RecyclerView
    private lateinit var foundRecyclerView: RecyclerView
    private lateinit var startScanBtn: Button
    private lateinit var stopScanBtn: Button
    private lateinit var toolbar: Toolbar

    private lateinit var pairedAdapter: DeviceAdapter
    private lateinit var scannedAdapter: DeviceAdapter

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(R.layout.device_connect_layout, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {

        toolbar = view.findViewById(R.id.device_toolbar)
        pairedRecyclerView = view.findViewById(R.id.pairedDeviceRecyclerView)
        foundRecyclerView = view.findViewById(R.id.foundDeviceRecyclerView)
        startScanBtn = view.findViewById(R.id.btn_start_scan)
        stopScanBtn = view.findViewById(R.id.btn_stop_scan)

        (activity as? AppCompatActivity)?.setSupportActionBar(toolbar)
        (activity as? AppCompatActivity)?.supportActionBar?.setDisplayHomeAsUpEnabled(false)
        (activity as? AppCompatActivity)?.supportActionBar?.setHomeButtonEnabled(false)

        toolbar.setOnMenuItemClickListener { item ->
            when (item.itemId) {
                R.id.action_scan_devices -> {
                    viewModel.startScan()
                    true
                }
                else -> false
            }
        }

        pairedAdapter = DeviceAdapter { device ->
            viewModel.connect(device.address)
        }
        scannedAdapter = DeviceAdapter { device ->
            viewModel.pair(device.address)
        }

        pairedRecyclerView.apply {
            layoutManager = LinearLayoutManager(requireContext())
            adapter = pairedAdapter
        }

        foundRecyclerView.apply {
            layoutManager = LinearLayoutManager(requireContext())
            adapter = scannedAdapter
        }

        startScanBtn.setOnClickListener { viewModel.startScan() }
        stopScanBtn.setOnClickListener { viewModel.stopScan() }


        observeState()
    }

    override fun onResume() {
        super.onResume()

        // 연결 상태가 이전에 남아있었다면 초기화
        if (viewModel.state.value.isConnected) {
            viewModel.disconnect() // 혹은 viewModel.resetState() 같은 함수 구현
        }
    }

    private fun observeState() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.lifecycle.repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.state.collect { state ->
                    pairedAdapter.submitList(state.pairedDevices)
                    scannedAdapter.submitList(state.scannedDevices)

                    val currentFragment = parentFragmentManager.findFragmentById(R.id.content_frame)
                    val isCommunicationScreenVisible = currentFragment is DeviceCommunicationFragment

                    if (state.isConnected && !isCommunicationScreenVisible && !hasNavigatedToCommunication) {
                        hasNavigatedToCommunication = true
                        goToDeviceCommunicationFragment()
                    }

                    if (!state.isConnected) {
                        hasNavigatedToCommunication = false
                    }
                }
            }
        }
    }

    private fun goToDeviceCommunicationFragment() {
        parentFragmentManager.beginTransaction()
            .replace(R.id.content_frame, DeviceCommunicationFragment()) // 이 ID는 적절히 변경
            .addToBackStack("DeviceCommunication")
            .commit()
    }

}