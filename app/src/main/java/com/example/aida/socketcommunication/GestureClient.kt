package com.example.aida.socketcommunication
import android.graphics.BitmapFactory
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import java.nio.ByteBuffer

/**
 * Client class that connects to server to receive
 * video data
 */
class GestureClient (ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){

    /**
     * Sends a request to start the image analysis to the server
     */
    fun sendStartGesture(){
        val id = MessageType.IMAGE_ANALYSIS.value
        val size = 2
        val start = Instructions.GESTURE.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start)
        sendDataToServer(id, buffer.array())
    }
    /**
     * Sends a request to stop the camera feed to the server
     */
    fun sendStopGesture() {
        val id = MessageType.IMAGE_ANALYSIS.value
        val size = 2
        val stop = Instructions.OFF.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(stop)
        sendDataToServer(id, buffer.array())
    }
}