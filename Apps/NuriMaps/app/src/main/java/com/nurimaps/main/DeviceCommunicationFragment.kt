package com.nurimaps.main

import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.InputMethodManager
import android.widget.EditText
import android.widget.ImageButton
import android.widget.TextView
import androidx.activity.addCallback
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.nurimaps.feature.presentation.components.DeviceConnectViewModel
import com.nurimaps.feature.presentation.components.ReceivedValuesAdapter
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class DeviceCommunicationFragment : Fragment() {

    private lateinit var deviceNameTextView: TextView
    private lateinit var receivedValuesRecyclerView: RecyclerView
    private lateinit var disconnectButton: ImageButton
    private lateinit var sendEditText: EditText
    private lateinit var sendButton: ImageButton

    private lateinit var adapter: ReceivedValuesAdapter

    // viewModel은 Fragment마다 달라질 수 있으니 생성자 주입 혹은 activityViewModels() 등 상황에 맞게 수정
    private val viewModel: DeviceConnectViewModel by activityViewModels()

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(R.layout.device_communication, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        deviceNameTextView = view.findViewById(R.id.deviceNameTextView)
        receivedValuesRecyclerView = view.findViewById(R.id.receivedValuesRecyclerView)
        disconnectButton = view.findViewById(R.id.disconnectButton)
        sendEditText = view.findViewById(R.id.sendEditText)
        sendButton = view.findViewById(R.id.sendButton)

        adapter = ReceivedValuesAdapter()
        receivedValuesRecyclerView.adapter = adapter
        receivedValuesRecyclerView.layoutManager = LinearLayoutManager(requireContext())

        // ViewModel state 구독
        lifecycleScope.launchWhenStarted {
            viewModel.state.collect { state ->
                deviceNameTextView.text = state.connectedDeviceName
                adapter.submitList(state.receivedValues.toList())  // toList()로 새 리스트 생성하여 DiffUtil 작동 유도

                // 연결 끊기 가능하게 버튼 활성화/비활성화 등 UI 상태 변경 가능
                disconnectButton.isEnabled = state.isConnected
            }
        }

        disconnectButton.setOnClickListener {
            viewModel.disconnect()
            // 필요하면 뒤로 가기 등 화면 전환 처리
            parentFragmentManager.popBackStack()
        }

        sendButton.setOnClickListener {
            val textToSend = sendEditText.text.toString()
            if (textToSend.isNotBlank()) {
                viewModel.sendValue(textToSend)
                sendEditText.text.clear()
                hideKeyboard()
            }
        }

        requireActivity().onBackPressedDispatcher.addCallback(viewLifecycleOwner) {
            viewModel.disconnect()  // 연결 해제
            parentFragmentManager.popBackStack()  // 뒤로가기
        }
    }

    private fun hideKeyboard() {
        val imm = requireContext().getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager
        imm.hideSoftInputFromWindow(sendEditText.windowToken, 0)
    }
}
