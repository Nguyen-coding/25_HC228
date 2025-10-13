package com.example.myapplication4

import android.os.Bundle
import android.text.Editable
import android.text.TextWatcher
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import kotlin.concurrent.thread
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import android.os.Handler
import android.os.Looper
import android.widget.Switch
import android.widget.Toast
import com.example.myapplication4.JoystickView


class MainActivity : AppCompatActivity() {
    private lateinit var editTextIpPort: EditText
    private lateinit var textView: TextView
    private lateinit var webport: TextView
    private lateinit var Leg1: Button
    private lateinit var Leg2: Button
    private lateinit var Leg3: Button
    private lateinit var Leg4: Button
    private lateinit var All: Button
    private lateinit var Stop: Button
    private lateinit var webView: WebView  // WebView 추가
    private lateinit var btnShowInput: Button
    private lateinit var port2:Button
    private lateinit var Torque:Button
    private lateinit var Angle:Button
    private lateinit var switchCrab: Switch
    private lateinit var AutoMode: Switch
    private var crabMode: Boolean = false
    private lateinit var joystick: JoystickView
    private var ipAddress: String = ""
    private var portNumber: String = ""
    private var portNumber2: String = ""
    private var lastCommand: String = ""
    private var lastDirectionTime: Long = 0L
    private var stopHandler = Handler(Looper.getMainLooper())
    private var stopRunnable: Runnable? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        editTextIpPort = findViewById(R.id.editTextIpPort)
        textView = findViewById(R.id.text_view)
        webport = findViewById(R.id.webport)
        webView = findViewById(R.id.webView)

        // WebView 설정
        val webSettings: WebSettings = webView.settings
        webSettings.javaScriptEnabled = true  // JavaScript 사용 설정
        webView.webViewClient = WebViewClient()  // WebView 내에서 새 창을 띄우지 않음

        // IP와 포트를 입력받아 저장
        editTextIpPort.addTextChangedListener(object : TextWatcher {
            override fun afterTextChanged(s: Editable?) {
                parseIpAndPort(s.toString())
                updateTextView()
                loadVideoInWebView()
            }

            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
        })


        webport.addTextChangedListener(object : TextWatcher {
            override fun afterTextChanged(s: Editable?) {
                portNumber2 = s.toString()
                updateTextView()
                loadVideoInWebView()
            }
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
        })

        Torque = findViewById(R.id.Torque)
        Angle = findViewById(R.id.Angle)
        Leg1 = findViewById(R.id.Leg1)
        Leg2 = findViewById(R.id.Leg2)
        Leg3 = findViewById(R.id.Leg3)
        Leg4 = findViewById(R.id.Leg4)
        All = findViewById(R.id.All)
        Stop = findViewById(R.id.Stop)
        btnShowInput = findViewById(R.id.btnShowInput)
        port2 = findViewById(R.id.port2)
        joystick = findViewById(R.id.joystickView)
        switchCrab = findViewById(R.id.switchCrab)
        AutoMode = findViewById(R.id.AutoMode)

        AutoMode.setOnClickListener { sendCommandToServer("F") }
        switchCrab.setOnCheckedChangeListener { _, isChecked ->
            crabMode = isChecked
            Toast.makeText(this, "Crab Mode: ${if (crabMode) "ON" else "OFF"}", Toast.LENGTH_SHORT)
                .show()
        }


        joystick.setOnMoveListener({ angle, strength ->
            val command = when {
                strength < 20 -> ""

                crabMode -> when {
                    angle >= 337.5 || angle < 22.5 -> ">"
                    angle < 67.5 -> "9"
                    angle < 112.5 -> "w"
                    angle < 157.5 -> "8"
                    angle < 202.5 -> "<"
                    angle < 247.5 -> "5"
                    angle < 292.5 -> "x"
                    else -> "6"
                }

                else -> when {
                    angle >= 337.5 || angle < 22.5 -> "d"
                    angle < 67.5 -> "e"
                    angle < 112.5 -> "w"
                    angle < 157.5 -> "q"
                    angle < 202.5 -> "a"
                    angle < 247.5 -> "z"
                    angle < 292.5 -> "x"
                    else -> "c"
                }
            }

            if (command != lastCommand && command.isNotEmpty()) {
                lastCommand = command
                sendCommandToServer(command)
                lastDirectionTime = System.currentTimeMillis()
                stopRunnable?.let { stopHandler.removeCallbacks(it) }
            }

            if (command.isEmpty()) {
                if (stopRunnable == null) {
                    stopRunnable = Runnable {
                        sendCommandToServer("s")
                        lastCommand = "s"
                        stopRunnable = null
                    }
                }
                stopHandler.postDelayed(stopRunnable!!, 10)
            }
        }, 100)

        Leg1.setOnClickListener {sendCommandToServer("Y")}
        Leg2.setOnClickListener {sendCommandToServer("k")}
        Leg3.setOnClickListener {sendCommandToServer("m")}
        Leg4.setOnClickListener {sendCommandToServer("-")}
        Torque.setOnClickListener {sendCommandToServer("t")}
        Angle.setOnClickListener {sendCommandToServer("I")}
        Stop.setOnClickListener {sendCommandToServer("s")}
        All.setOnClickListener {sendCommandToServer("S")}




        btnShowInput.setOnClickListener{
            if (editTextIpPort.visibility == View.GONE){
                editTextIpPort.visibility = View.VISIBLE}
            else {
                editTextIpPort.visibility = View.GONE

            }
        }

        port2.setOnClickListener {
            if (webport.visibility == View.GONE) {
                webport.visibility = View.VISIBLE
            } else {
                webport.visibility = View.GONE
            }
        }

        // 엣지 투 엣지 설정을 위해 WindowInsets 사용
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }
    }

    // IP와 포트 문자열을 파싱
    private fun parseIpAndPort(input: String) {
        val parts = input.split(":")
        if (parts.size == 2) {
            ipAddress = parts[0]
            portNumber = parts[1]
        } else {
            ipAddress = ""
            portNumber = ""
        }
    }

    // WebView를 통해 카메라 로드
    private fun loadVideoInWebView() {
        val portNumber2Int = portNumber2.toIntOrNull()
        if (ipAddress.isEmpty() || portNumber2Int == null) {
            textView.text = "Invalid IP address or port number"
            return
        }

        val videoUrl = "http://$ipAddress:$portNumber2Int/"
        webView.loadUrl(videoUrl)
        textView.text = "Loading video from $videoUrl"
    }

    private fun sendCommandToServer(command: String) {
        val port = portNumber.toIntOrNull()
        if (ipAddress.isEmpty() || port == null) {
            textView.text = "Invalid IP address or port number"
            return
        }
        sendUDPMessage(ipAddress, port, command)
    }

    private fun sendUDPMessage(ipAddress: String, port: Int, command: String) {
        thread {
            var socket: DatagramSocket? = null
            try {
                socket = DatagramSocket()
                val buffer = command.toByteArray()
                val serverAddress = InetAddress.getByName(ipAddress)
                val packet = DatagramPacket(buffer, buffer.size, serverAddress, port)
                socket.send(packet)
                runOnUiThread {
                    textView.text = "Command sent to $ipAddress:$port"
                }
                val responseBuffer = ByteArray(1024) // 응답을 저장할 버퍼 생성
                val responsePacket = DatagramPacket(responseBuffer, responseBuffer.size)
                socket.receive(responsePacket)
                val responseMessage = String(responsePacket.data, 0, responsePacket.length)
                runOnUiThread {
                    textView.text = "Received response: $responseMessage"
                }
            } catch (e: Exception) { // 예외처리
                e.printStackTrace()
                runOnUiThread {
                    textView.text = "Error: ${e.message}"
                }
            } finally {
                socket?.close()
            }
        }
    }


    private fun updateTextView() {
        textView.text = "Ip: $ipAddress\nPort: $portNumber"
    }

    override fun onBackPressed() {
        if (webView.canGoBack()) {
            webView.goBack()
        } else {
            super.onBackPressed()
        }
    }
}
