package com.nurimaps.feature.presentation.components

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.nurimaps.feature.ble.domain.model.BluetoothDeviceModel

class DeviceAdapter(
    private val onClick: (BluetoothDeviceModel) -> Unit
) : ListAdapter<BluetoothDeviceModel, DeviceAdapter.DeviceViewHolder>(DiffCallback) {

    object DiffCallback : DiffUtil.ItemCallback<BluetoothDeviceModel>() {
        override fun areItemsTheSame(oldItem: BluetoothDeviceModel, newItem: BluetoothDeviceModel): Boolean {
            return oldItem.address == newItem.address
        }

        override fun areContentsTheSame(oldItem: BluetoothDeviceModel, newItem: BluetoothDeviceModel): Boolean {
            return oldItem == newItem
        }
    }

    inner class DeviceViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        val text = view.findViewById<TextView>(android.R.id.text1)
        fun bind(device: BluetoothDeviceModel) {
            text.text = device.name ?: "(No name)"
            itemView.setOnClickListener { onClick(device) }
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DeviceViewHolder {
        val view = LayoutInflater.from(parent.context)
            .inflate(android.R.layout.simple_list_item_1, parent, false)
        return DeviceViewHolder(view)
    }

    override fun onBindViewHolder(holder: DeviceViewHolder, position: Int) {
        holder.bind(getItem(position))
    }
}
