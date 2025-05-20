package com.nurimaps.main

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.FrameLayout
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.Toolbar
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentContainerView
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
import kotlinx.coroutines.launch

@AndroidEntryPoint
class DeviceConnectFragment : Fragment(){

    private val viewModel: DeviceConnectViewModel by viewModels()

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

        // 툴바 설정
        setupToolbar()

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

    private fun setupToolbar() {
        (activity as? AppCompatActivity)?.setSupportActionBar(toolbar)
        toolbar.setNavigationOnClickListener {
            parentFragmentManager.popBackStack()
            requireActivity().findViewById<FragmentContainerView>(R.id.map_fragment).visibility = View.VISIBLE
            requireActivity().findViewById<FrameLayout>(R.id.content_frame).visibility = View.GONE
            (activity as? MainActivity)?.showTopLeftButton(true)
        }

        toolbar.setOnMenuItemClickListener { item ->
            when (item.itemId) {
                R.id.action_scan_devices -> {
                    viewModel.startScan()
                    true
                }
                else -> false
            }
        }
    }

    private fun setupRecyclerViews() {
        pairedRecyclerView.layoutManager = LinearLayoutManager(requireContext())
        foundRecyclerView.layoutManager = LinearLayoutManager(requireContext())
        // TODO: adapter 설정 필요
    }

    private fun observeState() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.lifecycle.repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.state.collect { state ->
                    pairedAdapter.submitList(state.pairedDevices)
                    scannedAdapter.submitList(state.scannedDevices)
                }
            }
        }
    }


//    private fun showConnectingDialog() {
//        if (!::connectingDialog.isInitialized || !connectingDialog.isShowing) {
//            connectingDialog = AlertDialog.Builder(requireContext())
//                .setTitle("Connecting...")
//                .setMessage("Please wait while connecting to device")
//                .setCancelable(false)
//                .create()
//            connectingDialog.show()
//        }
//    }
//
//    private fun dismissConnectingDialog() {
//        if (::connectingDialog.isInitialized && connectingDialog.isShowing) {
//            connectingDialog.dismiss()
//        }
//    }
//
//    private fun showConnectedView(state: DeviceConnectState) {
//        connectedLayout.visibility = View.VISIBLE
//        disconnectedLayout.visibility = View.GONE
//
//        // TODO: 연결된 상태에서 받은 값 표시 및 보내기 UI 구성
//    }
//
//    private fun showDisconnectedView(state: DeviceConnectState) {
//        connectedLayout.visibility = View.GONE
//        disconnectedLayout.visibility = View.VISIBLE
//
//        // TODO: state.pairedDevices / state.scannedDevices로 RecyclerView 업데이트
//    }

}