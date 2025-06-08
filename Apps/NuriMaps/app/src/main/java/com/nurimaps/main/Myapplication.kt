package com.nurimaps.main

import android.app.Application
import com.nurimaps.feature.ble.domain.BLEController
import dagger.hilt.android.HiltAndroidApp
import javax.inject.Inject

@HiltAndroidApp
class MyApplication : Application(){
    @Inject
    lateinit var bleController: BLEController

    override fun onTerminate() {
        super.onTerminate()
        bleController.release()
    }
}