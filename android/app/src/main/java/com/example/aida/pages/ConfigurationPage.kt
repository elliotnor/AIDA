package com.example.aida.pages

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.material3.VerticalDivider
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.aida.R
import com.example.aida.viewmodels.MainViewModel

/**
 * The configuration page contains information on how to connect to AIDA
 * It also shows the connections status of some of the nodes on AIDA
 *
 * @param barHeight used to ensure that the content is padded correctly
 * @param viewModel contains connection information, used to update IP
 * address, update port and connect to AIDA with new information
 * @param onButtonPress updates the state and the topBar title
 *
 * @author Elias
 */
@Composable
fun ConfigurationPage(
    barHeight: Dp,
    viewModel: MainViewModel,
    onButtonPress: () -> Unit
) {
    Row(
        modifier = Modifier
            .padding(top = barHeight)
            .fillMaxSize(),
    ) {
        val paddingTop = 20.dp
        val paddingSides = 30.dp
        val rowSpacing = 10.dp

        // Here comes the connection variables for the nodes
        // Lidar connection variables
        var lidarConnectedText by remember { mutableStateOf("Disconnected") }
        var lidarImageBoolean by remember { mutableStateOf(false) }

        if(viewModel.isLidarConnected){
            lidarConnectedText = "Connected"
            lidarImageBoolean = true
        }
        else if(!viewModel.isLidarConnected){
            lidarConnectedText = "Disconnected"
            lidarImageBoolean = false
        }

        val lidarImage = if (lidarImageBoolean) {
            painterResource(id = R.drawable.link_300)
        } else {
            painterResource(id = R.drawable.link_off_300)
        }

        // STT connection variables
        var STTConnectedText by remember { mutableStateOf("Disconnected") }
        var STTImageBoolean by remember { mutableStateOf(false) }

        if(viewModel.isSTTConnected){
            STTConnectedText = "Connected"
            STTImageBoolean = true
        }
        else if(!viewModel.isSTTConnected){
            STTConnectedText = "Disconnected"
            STTImageBoolean = false
        }

        val STTImage = if (STTImageBoolean) {
            painterResource(id = R.drawable.link_300)
        } else {
            painterResource(id = R.drawable.link_off_300)
        }

        // Camera connection variables
        var cameraConnectedText by remember { mutableStateOf("Disconnected") }
        var cameraImageBoolean by remember { mutableStateOf(false) }

        if(viewModel.isCameraConnected){
            cameraConnectedText = "Connected"
            cameraImageBoolean = true
        }
        else if(!viewModel.isCameraConnected){
            cameraConnectedText = "Disconnected"
            cameraImageBoolean = false
        }

        val cameraImage = if (cameraImageBoolean) {
            painterResource(id = R.drawable.link_300)
        } else {
            painterResource(id = R.drawable.link_off_300)
        }

        // gesture connection variables
        var gestureConnectedText by remember { mutableStateOf("Disconnected") }
        var gestureImageBoolean by remember { mutableStateOf(false) }

        if(viewModel.isGestureConnected){
            gestureConnectedText = "Connected"
            gestureImageBoolean = true
        }
        else if(!viewModel.isGestureConnected){
            gestureConnectedText = "Disconnected"
            gestureImageBoolean = false
        }

        val gestureImage = if (gestureImageBoolean) {
            painterResource(id = R.drawable.link_300)
        } else {
            painterResource(id = R.drawable.link_off_300)
        }


        // Joystick connection variables
        var JoystickConnectedText by remember { mutableStateOf("Disconnected") }
        var JoystickImageBoolean by remember { mutableStateOf(false) }

        if(viewModel.isJoystickConnected){
            JoystickConnectedText = "Connected"
            JoystickImageBoolean = true
        }
        else if(!viewModel.isJoystickConnected){
            JoystickConnectedText = "Disconnected"
            JoystickImageBoolean = false
        }

        val JoystickImage = if (JoystickImageBoolean) {
            painterResource(id = R.drawable.link_300)
        } else {
            painterResource(id = R.drawable.link_off_300)
        }



        // First column for SSH Connection Data
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing)
        ) {
            var ipInput by remember { mutableStateOf(viewModel.ipAddress.value) }
            var portInput by remember { mutableStateOf(viewModel.port.value.toString()) }
            val standardInputModifier = Modifier.weight(1f)

            Text(text = "SSH Connection Data",
                modifier = Modifier.align(Alignment.CenterHorizontally),
                fontWeight = FontWeight.Bold,
                fontSize = 25.sp
            )

            Spacer(modifier = Modifier.height(25.dp))

            var isIpError by rememberSaveable { mutableStateOf(false) }
            var isPortError by rememberSaveable { mutableStateOf(false) }

            // IP & Port input
            Row(
                horizontalArrangement = Arrangement.spacedBy(rowSpacing)
            ) {
                // IP input
                TextField(
                    value = ipInput,
                    onValueChange = { newValue ->
                        // Check that the address is correctly formatted
                        ipInput = newValue
                        isIpError =
                            !ipInput.matches(Regex("^((25[0-5]|(2[0-4]|1\\d|[1-9]|)\\d)\\.?\\b){4}\$"))
                    },
                    label = { Text("IP-address") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = standardInputModifier,
                    isError = isIpError,
                    supportingText = {
                        if (isIpError) {
                            Text(
                                modifier = Modifier.fillMaxWidth(),
                                text = "IP-address is not correctly formatted, should be of standard \"1.1.1.1\"",
                                color = MaterialTheme.colorScheme.error
                            )
                        }
                    },
                )

                // Port input
                TextField(
                    value = portInput,
                    onValueChange = { newValue ->
                        // Ensure the input is 0 to 4 characters
                        portInput = newValue
                        isPortError = !newValue.matches(Regex("^[0-9]{1,4}\$"))
                    },
                    label = { Text("Port") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = standardInputModifier,
                    isError = isPortError,
                    supportingText = {
                        if (isPortError) {
                            Text(
                                modifier = Modifier.fillMaxWidth(),
                                text = "Port is not correctly formatted, should be of standard 1 to 9999",
                                color = MaterialTheme.colorScheme.error
                            )
                        }
                    },
                )
            }


            // Button to confirm IP address and port
            Button(
                onClick = {
                    if (!isIpError || !isPortError) {
                        viewModel.updateIpAddress(ipInput)
                        viewModel.updatePort(portInput.toInt())
                        viewModel.connectToAIDA()
                        onButtonPress()
                    }

                },
                modifier = Modifier
                    .padding(20.dp)
                    .align(Alignment.CenterHorizontally)
                    .fillMaxWidth(0.4f)
            )
            {
                Text("Connect")
            }

            // AIDA logo at the bottom of the left column
            Image(
                painter = painterResource(id = R.drawable.aida_logo_new),
                contentDescription = "Example Image",
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
                    .padding(top = 10.dp)
                    .height(350.dp)
                    .width(350.dp)
            )
        }


        // Divider to separate the two columns
        VerticalDivider(
            modifier = Modifier
                .padding(top = 30.dp, bottom = 30.dp)
                .fillMaxHeight()
                .width(1.dp),
            color = Color.Gray
        )


        val scrollState = rememberScrollState()

        // Second column for the connection status of the nodes
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(top = 30.dp, bottom = 30.dp)
                .verticalScroll(scrollState),
            verticalArrangement = Arrangement.spacedBy(rowSpacing),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {


            // Top row
            Row(
                modifier = Modifier
                    .fillMaxWidth(),
                horizontalArrangement = Arrangement.Center
            ) {
                // Left column of top row, Mic node
                Column(
                    modifier = Modifier
                        .padding(top = 30.dp, bottom = 30.dp)
                        .width(250.dp),
                    verticalArrangement = Arrangement.Center,
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Image(
                        painter = painterResource(id = R.drawable.mic_500),
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(150.dp)
                            .width(150.dp)
                    )
                    Text(
                        text = "STT",
                        fontSize = 19.sp,
                        textAlign = TextAlign.Center
                    )
                    Image(
                        painter = STTImage,
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(50.dp)
                            .width(50.dp)
                    )
                    Text(
                        text = STTConnectedText,
                        fontSize = 12.sp,
                        textAlign = TextAlign.Center
                    )
                }

                // Spacer between left column and right column
                Spacer(modifier = Modifier.width(60.dp))

                // Right column of top row, Lidar node
                Column(
                    modifier = Modifier
                        .padding(top = 30.dp, bottom = 30.dp)
                        .width(250.dp),
                    verticalArrangement = Arrangement.Center,
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Image(
                        painter = painterResource(id = R.drawable.lidar_500),
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(150.dp)
                            .width(150.dp)
                    )
                    Text(
                        text = "LiDAR",
                        fontSize = 19.sp,
                        textAlign = TextAlign.Center
                    )
                    Image(
                        painter = lidarImage,
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(50.dp)
                            .width(50.dp)
                    )
                    Text(
                        text = lidarConnectedText,
                        fontSize = 12.sp,
                        textAlign = TextAlign.Center
                    )
                }
            }


            // Middle row
            Row(
                modifier = Modifier
                    .fillMaxWidth(),
                horizontalArrangement = Arrangement.Center
            ) {
                // Left column of middle row, Camera node
                Column(
                    modifier = Modifier
                        .padding(top = 30.dp, bottom = 30.dp)
                        .width(250.dp),
                    verticalArrangement = Arrangement.Center,
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Image(
                        painter = painterResource(id = R.drawable.videocam_500),
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(150.dp)
                            .width(150.dp)
                    )
                    Text(
                        text = "Camera",
                        fontSize = 19.sp,
                        textAlign = TextAlign.Center
                    )
                    Image(
                        painter = cameraImage,
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(50.dp)
                            .width(50.dp)
                    )
                    Text(
                        text = cameraConnectedText,
                        fontSize = 12.sp,
                        textAlign = TextAlign.Center
                    )
                }

                // Spacer between left column and right column
                Spacer(modifier = Modifier.width(60.dp))

                // Right column of bottom row, Gesture node
                Column(
                    modifier = Modifier
                        .padding(top = 30.dp, bottom = 30.dp)
                        .width(250.dp),
                    verticalArrangement = Arrangement.Center,
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Image(
                        painter = painterResource(id = R.drawable.hand_gesture_500),
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(150.dp)
                            .width(150.dp)
                    )
                    Text(
                        text = "Image recognition",
                        fontSize = 19.sp,
                        textAlign = TextAlign.Center
                    )
                    Image(
                        painter = gestureImage,
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(50.dp)
                            .width(50.dp)
                    )
                    Text(
                        text = gestureConnectedText,
                        fontSize = 12.sp,
                        textAlign = TextAlign.Center
                    )
                }
            }


            // Bottom row
            Row(
                modifier = Modifier
                    .fillMaxWidth(),
                horizontalArrangement = Arrangement.Center
            ) {
                // Left column of bottom row (there is no right column), Joystick node
                Column(
                    modifier = Modifier
                        .padding(top = 30.dp, bottom = 30.dp)
                        .width(250.dp),
                    verticalArrangement = Arrangement.Center,
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Image(
                        painter = painterResource(id = R.drawable.joystick_500),
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(150.dp)
                            .width(150.dp)
                    )
                    Text(
                        text = "Joystick",
                        fontSize = 19.sp,
                        textAlign = TextAlign.Center
                    )
                    Image(
                        painter = JoystickImage,
                        contentDescription = "Example Image",
                        modifier = Modifier
                            .height(50.dp)
                            .width(50.dp)
                    )
                    Text(
                        text = JoystickConnectedText,
                        fontSize = 12.sp,
                        textAlign = TextAlign.Center
                    )
                }
            }


        }
    }
}
