package com.nurimaps.main

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.FrameLayout
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.Toolbar
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentContainerView
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView

class DeviceConnectFragment : Fragment(){

    private lateinit var pairedRecyclerView: RecyclerView
    private lateinit var foundRecyclerView: RecyclerView
    private lateinit var toolbar: Toolbar

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


        // 툴바 설정
        (activity as? AppCompatActivity)?.setSupportActionBar(toolbar)
        toolbar.setNavigationOnClickListener {
            // 장치연결 화면 감추고 맵 다시 보이기
            parentFragmentManager.popBackStack()
            requireActivity().findViewById<FragmentContainerView>(R.id.map_fragment).visibility = View.VISIBLE
            requireActivity().findViewById<FrameLayout>(R.id.content_frame).visibility = View.GONE
            (activity as? MainActivity)?.showTopLeftButton(true)
        }

        toolbar.setOnMenuItemClickListener { item ->
            when (item.itemId) {
//                R.id.action_scan_devices -> {
//                    // TODO: 장치 검색 기능 구현 예정
//                    Toast.makeText(requireContext(), "장치 검색 기능 예정", Toast.LENGTH_SHORT).show()
//                    true
//                }
                else -> false
            }
        }

        // RecyclerView 초기화 (샘플 어댑터 사용)
        pairedRecyclerView.layoutManager = LinearLayoutManager(requireContext())
        foundRecyclerView.layoutManager = LinearLayoutManager(requireContext())

        // 예시 데이터
        val pairedDevices = listOf("기기 A", "기기 B")
        val foundDevices = listOf("기기 C", "기기 D")

//        pairedRecyclerView.adapter = DeviceListAdapter(pairedDevices)
//        foundRecyclerView.adapter = DeviceListAdapter(foundDevices)
    }

}