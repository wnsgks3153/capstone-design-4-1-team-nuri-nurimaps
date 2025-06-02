package com.nurimaps.main

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.Intent
import android.os.Bundle
import android.view.View
import android.graphics.Typeface
import android.location.Location
import android.widget.TextView
import android.widget.ScrollView
import android.widget.LinearLayout
import android.content.pm.PackageManager
import android.os.Build
import android.widget.FrameLayout
import android.widget.Toast
import androidx.activity.addCallback
import androidx.cardview.widget.CardView
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.core.view.ViewCompat
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.WindowInsetsCompat
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatImageButton
import androidx.drawerlayout.widget.DrawerLayout
import androidx.fragment.app.FragmentContainerView
import com.google.android.gms.location.LocationServices
import com.google.android.material.navigation.NavigationView
import com.google.android.gms.location.FusedLocationProviderClient
import com.naver.maps.map.NaverMap
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.MapFragment
import com.naver.maps.map.CameraUpdate
import com.naver.maps.map.CameraAnimation
import com.naver.maps.map.NaverMapOptions
import com.naver.maps.geometry.LatLngBounds
import com.naver.maps.map.OnMapReadyCallback
import com.naver.maps.map.widget.CompassView
import com.naver.maps.map.LocationTrackingMode
import com.naver.maps.map.overlay.OverlayImage
import com.naver.maps.map.overlay.GroundOverlay
import com.naver.maps.map.util.FusedLocationSource
import com.naver.maps.map.widget.ZoomControlView
import com.nurimaps.feature.uwb.CustomLocationSource
import com.nurimaps.feature.uwb.PositionViewModel
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : AppCompatActivity(), OnMapReadyCallback {

    // 상수 정의
    companion object {
        private const val DEFAULT_FLOOR = "3F"
        private const val MAP_ZOOM_THRESHOLD = 17.0
        private const val MAX_DISTANCE_TO_TARGET = 500.0
        private val TARGET_LOCATION = LatLng(36.1678660, 128.4676331)
        private const val LOCATION_PERMISSION_REQUEST_CODE = 100
    }

    // 상태 변수
    private var currentLocation: Location? = null // 현재 위치 값 받아와서 다른데 쓸일 있으면 다시 사용
    private var groundOverlay: GroundOverlay? = null
    private var currentFloor = DEFAULT_FLOOR // 기본 층을 3층으로 설정

    // UI 컴포넌트
    private lateinit var floorScrollView: ScrollView
    private lateinit var floorButtonLayout: LinearLayout
    private lateinit var floorsCardContainer: CardView
    private lateinit var drawerLayout: DrawerLayout
    private lateinit var topLeftButton: AppCompatImageButton
    private lateinit var navView: NavigationView

    // 층수 버튼
    private val floorButtons = mutableMapOf<String, TextView>()

    // 위치를 받아오기 위한
    //private lateinit var locationSource: FusedLocationSource //
    private lateinit var naverMap: NaverMap

    private val positionViewModel: PositionViewModel by viewModels()
    private lateinit var customLocationSource: CustomLocationSource
    private lateinit var gpsLocationSource: FusedLocationSource // GPS LocationSource 참조 유지
    private lateinit var fusedLocationClient: FusedLocationProviderClient

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val locationGranted = permissions[Manifest.permission.ACCESS_FINE_LOCATION] == true ||
                permissions[Manifest.permission.ACCESS_COARSE_LOCATION] == true

        val bluetoothGranted = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions[Manifest.permission.BLUETOOTH_CONNECT] == true &&
                    permissions[Manifest.permission.BLUETOOTH_SCAN] == true
        } else {
            permissions[Manifest.permission.BLUETOOTH] == true &&
                    permissions[Manifest.permission.BLUETOOTH_ADMIN] == true
        }

        if (locationGranted) {
            initMap()
        }

        if (bluetoothGranted && !isBluetoothEnabled) {
            enableBluetoothLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
        }

    }

    private val enableBluetoothLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) {
        // Bluetooth Enable 결과 처리 (필요시)
    }

    private val bluetoothManager by lazy {
        applicationContext.getSystemService(BluetoothManager::class.java)
    }
    private val bluetoothAdapter by lazy {
        bluetoothManager?.adapter
    }

    private val isBluetoothEnabled: Boolean
        get() = bluetoothAdapter?.isEnabled == true

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_main)

        // 시스템 바 인셋 처리
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }

        drawerLayout = findViewById(R.id.drawer_layout)
        topLeftButton = findViewById(R.id.top_left_button)
        navView = findViewById(R.id.navigation_view)

        // 위치 & BLE 권한 한번에 요청
        requestAllPermissions()

        // UI 초기화
        initUI()

        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)

        gpsLocationSource = FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE).apply {}

        // CustomLocationSource 생성
        customLocationSource = CustomLocationSource(gpsLocationSource, positionViewModel)

        onBackPressedDispatcher.addCallback(this) {
            if (drawerLayout.isDrawerOpen(navView)) {
                drawerLayout.closeDrawer(navView)
            } else {
                val backStackCount = supportFragmentManager.backStackEntryCount
                if (backStackCount > 0) {
                    supportFragmentManager.popBackStack()
                } else {
                    finish()
                }
            }
        }

        topLeftButton.setOnClickListener {
            if (!drawerLayout.isDrawerOpen(navView)) {
                drawerLayout.openDrawer(navView)
            }
        }

        navView.setNavigationItemSelectedListener { menuItem ->
            when (menuItem.itemId) {
                R.id.device_connect_item -> {
                    supportFragmentManager.beginTransaction()
                        .replace(R.id.content_frame, DeviceConnectFragment())
                        .addToBackStack(null)
                        .commit()

                    findViewById<FragmentContainerView>(R.id.map_fragment).visibility = View.GONE
                    findViewById<FrameLayout>(R.id.content_frame).visibility = View.VISIBLE
                    showTopLeftButton(false)
                    drawerLayout.setDrawerLockMode(DrawerLayout.LOCK_MODE_LOCKED_CLOSED)
                }
                R.id.developer_tools_item -> {
                    supportFragmentManager.beginTransaction()
                        .replace(R.id.content_frame, DeveloperToolsFragment())
                        .addToBackStack(null)
                        .commit()

                    findViewById<FragmentContainerView>(R.id.map_fragment).visibility = View.GONE
                    findViewById<FrameLayout>(R.id.content_frame).visibility = View.VISIBLE
                    showTopLeftButton(false)
                    drawerLayout.setDrawerLockMode(DrawerLayout.LOCK_MODE_LOCKED_CLOSED)
                }
            }
            drawerLayout.closeDrawers()
            true
        }

        supportFragmentManager.addOnBackStackChangedListener {
            val isContentVisible = supportFragmentManager.backStackEntryCount > 0

            findViewById<FragmentContainerView>(R.id.map_fragment).visibility =
                if (isContentVisible) View.GONE else View.VISIBLE

            findViewById<FrameLayout>(R.id.content_frame).visibility =
                if (!isContentVisible) View.GONE else View.VISIBLE

            findViewById<CompassView>(R.id.compass).visibility =
                if (isContentVisible) View.GONE else View.VISIBLE

            findViewById<ZoomControlView>(R.id.zoom).visibility =
                if (isContentVisible) View.GONE else View.VISIBLE

            findViewById<CardView>(R.id.floors_card_container).visibility =
                if (isContentVisible) View.GONE else View.VISIBLE

            showTopLeftButton(!isContentVisible) // content_frame이 보이면 true, 아니면 false

            drawerLayout.setDrawerLockMode(
                if (isContentVisible) DrawerLayout.LOCK_MODE_LOCKED_CLOSED
                else DrawerLayout.LOCK_MODE_UNLOCKED
            )
        }


    }

    override fun onMapReady(naverMap: NaverMap) {

        // 초기 심볼 스케일 설정
        naverMap.setSymbolScale(1.0f)


        // CustomLocationSource 사용
        // naverMap.locationSource = locationSource
        naverMap.locationSource = customLocationSource

        naverMap.isIndoorEnabled = true
        naverMap.uiSettings.isLocationButtonEnabled = true
        naverMap.uiSettings.isCompassEnabled = false
        naverMap.uiSettings.isZoomControlEnabled = false
        findViewById<CompassView>(R.id.compass).map = naverMap
        findViewById<ZoomControlView>(R.id.zoom).map = naverMap

        naverMap.locationTrackingMode = LocationTrackingMode.Follow
        naverMap.addOnOptionChangeListener {
            val mode = naverMap.locationTrackingMode
            gpsLocationSource.isCompassEnabled = mode == LocationTrackingMode.Follow || mode == LocationTrackingMode.Face
        }

        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED || ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_COARSE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED) {

            fusedLocationClient.lastLocation
                .addOnSuccessListener { location: Location? ->
                    location?.let { loc ->
                        //currentLocation = loc  // 멤버 변수에 할당( 나중에 쓸일이 있다면 다시 사용)

                        naverMap.locationOverlay.run {
                            isVisible = true
                            position = LatLng(loc.latitude, loc.longitude)
                        }

                        val cameraUpdate = CameraUpdate
                            .scrollTo(LatLng(loc.latitude, loc.longitude))
                            .animate(CameraAnimation.Fly)

                        naverMap.moveCamera(cameraUpdate)
                    }
                }
        } else {
            // 권한이 없을 때 동작 (필요하면 권한 요청 UI 띄우거나 안내)
        }

        setupCameraListener(naverMap)
    }

    override fun onDestroy() {
        super.onDestroy()
        // CustomLocationSource 정리
        if (::customLocationSource.isInitialized) {
            customLocationSource.deactivate()
        }
    }

    private fun requestAllPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_ADVERTISE
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN
            )
        }

        permissionLauncher.launch(permissions)
    }


    private fun initUI() {
        // 층수 관련 UI 컴포넌트 초기화
        floorsCardContainer = findViewById(R.id.floors_card_container)
        floorScrollView = findViewById(R.id.floor_scroll_view)
        floorButtonLayout = findViewById(R.id.floor_button_layout)

        // 카드뷰 초기에 숨김
        floorsCardContainer.visibility = View.GONE

        // 스크롤바 숨기기
        floorScrollView.isVerticalScrollBarEnabled = false

        // 층수 버튼 초기화
        initFloorButtons()
    }

    fun showTopLeftButton(show: Boolean) {
        if (::topLeftButton.isInitialized) {
            topLeftButton.visibility = if (show) View.VISIBLE else View.GONE
        }
    }

    // 층수 버튼 초기화
    private fun initFloorButtons() {
        val floorButtonIds = mapOf(
            "5F" to R.id.floor_5_button,
            "4F" to R.id.floor_4_button,
            "3F" to R.id.floor_3_button,
            "2F" to R.id.floor_2_button,
            "1F" to R.id.floor_1_button,
            "B1" to R.id.floor_b1_button,
            "B2" to R.id.floor_b2_button
        )

        // 버튼들을 맵에 저장하고 클릭 리스너 설정
        floorButtonIds.forEach { (floor, buttonId) ->
            val button = findViewById<TextView>(buttonId)
            floorButtons[floor] = button
            button.setOnClickListener { changeFloor(floor) }
        }

        initDefaultFloor()
    }

    // 기본 층 설정
    private fun initDefaultFloor() {
        scrollToFloorButton(currentFloor)
        highlightSelectedFloor(currentFloor)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == LOCATION_PERMISSION_REQUEST_CODE) {
            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                initMap()
            } else {
                Toast.makeText(this, "위치 권한이 필요합니다.", Toast.LENGTH_SHORT).show()
                // 권한 없을 때 처리
            }
        }
    }

    private fun initMap() {
        val mapFragment = supportFragmentManager.findFragmentById(R.id.map_fragment) as MapFragment?
            ?: MapFragment.newInstance(NaverMapOptions()
                .indoorEnabled(true)
                .compassEnabled(false)
                .zoomControlEnabled(false)
                .locationButtonEnabled(true)
            ).also {
                supportFragmentManager.beginTransaction().add(R.id.map_fragment, it).commit()
            }
        mapFragment.getMapAsync(this)
    }

//    override fun onBackPressed() {
//        if (drawerLayout.isDrawerOpen(navView)) {
//            drawerLayout.closeDrawer(navView)
//        } else {
//            val currentFragment = supportFragmentManager.findFragmentById(R.id.content_frame)
//            val dispatcher = onBackPressedDispatcher
//
//            val lifecycleOwner = (currentFragment?.viewLifecycleOwner)
//            val backStackCount = supportFragmentManager.backStackEntryCount
//
//            // 현재 Fragment에 OnBackPressedCallback이 있는 경우
//            if (lifecycleOwner != null && backStackCount > 0) {
//                dispatcher.onBackPressed() // 👉 Fragment에서 등록한 콜백으로 이동
//            } else if (backStackCount > 0) {
//                supportFragmentManager.popBackStack()
//            } else {
//                super.onBackPressed()
//            }
//        }
//    }

    private fun setupCameraListener(naverMap: NaverMap) {
        naverMap.addOnCameraIdleListener {
            val zoomLevel = naverMap.cameraPosition.zoom
            val cameraPosition = naverMap.cameraPosition.target

            val distanceToTarget = cameraPosition.distanceTo(TARGET_LOCATION)

            // 줌 레벨이 17 이상이고 타겟 위치에서 500m 이내일 때만 심볼 숨김
            if (zoomLevel >= MAP_ZOOM_THRESHOLD && distanceToTarget <= MAX_DISTANCE_TO_TARGET) {
                naverMap.setSymbolScale(0.0f)  // 심볼 숨김
                showOverlay(naverMap)  // 오버레이 표시
                showFloorUI()  // 층수 UI 표시
            } else {
                naverMap.setSymbolScale(1.0f)  // 기본 심볼 크기로 표시
                hideOverlay()  // 오버레이 숨김
                hideFloorUI()  // 층수 UI 숨김
            }
        }
    }

    // 오버레이 표시
    private fun showOverlay(naverMap: NaverMap) {
        if (groundOverlay == null) {
            val southWest = LatLng(36.16777, 128.46721) // 좌측 하단
            val northEast = LatLng(36.16796, 128.46813) // 우측 상단

            // LatLngBounds로 범위 설정
            val bounds = LatLngBounds(southWest, northEast)

            groundOverlay = GroundOverlay().apply {
                image = getOverlayImage(currentFloor)
                this.bounds = bounds
                setMap(naverMap)
            }

            // 오버레이가 표시될 때 현재 층에 맞는 버튼 강조 표시
            highlightSelectedFloor(currentFloor)

        } else {
            groundOverlay?.setMap(naverMap)
        }
    }

    // 오버레이 숨기기
    private fun hideOverlay() {
        groundOverlay?.setMap(null)
    }

    // 층수에 맞는 오버레이 이미지 리소스 ID 반환
    private fun getOverlayImage(floor: String): OverlayImage {
        val overlayImageRes = when(floor) {
            "5F" -> R.drawable.floor_image3 // 실제 5층 이미지로 변경 필요
            "4F" -> R.drawable.floor_image4_2_1
            "3F" -> R.drawable.floor_image3_2_1
            "2F" -> R.drawable.floor_image3 // 실제 2층 이미지로 변경 필요
            "1F" -> R.drawable.floor_image3 // 실제 1층 이미지로 변경 필요
            "B1" -> R.drawable.floor_image3 // 실제 B1층 이미지로 변경 필요
            "B2" -> R.drawable.floor_image3 // 실제 B2층 이미지로 변경 필요
            else -> R.drawable.floor_image3 // 기본 3층
        }
        return OverlayImage.fromResource(overlayImageRes)
    }

    // 층수에 맞는 이미지를 업데이트하는 함수
    private fun updateOverlayImage(floor: String) {
        groundOverlay?.let {
            it.image = getOverlayImage(floor)
        }
    }

    private fun showFloorUI() {
        floorsCardContainer.visibility = View.VISIBLE
        scrollToFloorButton(currentFloor)
    }

    private fun hideFloorUI() {
        floorsCardContainer.visibility = View.GONE
    }

    // 특정 버튼으로 스크롤
    private fun scrollToFloorButton(floor: String) {
        val buttonIndex = when (floor) {
            "5F" -> 0
            "4F" -> 1
            "3F" -> 2
            "2F" -> 3
            "1F" -> 4
            "B1" -> 5
            "B2" -> 6
            else -> 2
        }

        floorScrollView.post {
            val targetView = floorButtonLayout.getChildAt(buttonIndex)
            if (targetView != null) {
                val buttonTop = targetView.top
                val buttonHeight = targetView.height
                val scrollViewHeight = floorScrollView.height

                val scrollTo = buttonTop - (scrollViewHeight / 2) + (buttonHeight / 2)
                floorScrollView.smoothScrollTo(0, if (scrollTo < 0) 0 else scrollTo)
            }
        }
    }

    // 층 변경 처리
    private fun changeFloor(floor: String) {
        if (currentFloor != floor) {
            currentFloor = floor
            updateOverlayImage(floor)
            highlightSelectedFloor(floor)
            scrollToFloorButton(floor)
        }
    }

    // 선택된 층수 버튼 강조
    private fun highlightSelectedFloor(floor: String) {
        resetAllFloorButtons()

        floorButtons[floor]?.apply {
            isSelected = true
            setTextColor(ContextCompat.getColor(this@MainActivity, android.R.color.black))
            typeface = Typeface.DEFAULT_BOLD
        }
    }

    // 모든 층수 버튼 초기화
    private fun resetAllFloorButtons() {
        floorButtons.values.forEach { button ->
            button.isSelected = false
            button.setTextColor(ContextCompat.getColor(this, android.R.color.black))
            button.typeface = Typeface.DEFAULT
        }
    }
}