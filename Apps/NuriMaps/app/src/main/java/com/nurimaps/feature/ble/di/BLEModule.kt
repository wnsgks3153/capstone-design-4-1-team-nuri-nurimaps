package com.nurimaps.feature.ble.di

import android.content.Context
import com.nurimaps.feature.ble.data.BLEControllerImpl
import com.nurimaps.feature.ble.domain.BLEController

import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object BLEModule {

    @Provides
    @Singleton
    fun provideBTController(
        @ApplicationContext context: Context
    ): BLEController = BLEControllerImpl(context)

}