package com.example.aida.pages

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.text.KeyboardOptions
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
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.example.aida.viewmodels.MainViewModel

/**
 * The configuration page contains information on how to connect to AIDA
 * It's still in early development and might therefore change, currently
 * there is no logic and only an UI component
 *
 * @param barHeight used to ensure that the content is padded correctly
 * @param viewModel contains connection information, used to update IP
 * address, update port and connect to AIDA with new information
 * @param onButtonPress updates the state and the topBar title
 *
 * @author Elias
 */
@Composable
fun UserGuidePage(
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
        var textfield by remember { mutableStateOf("") }



        // First column for the SSH Terminal, currently unused
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing)
        ) {

            Column(
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
                    .padding(top = paddingTop, start = paddingSides, end = paddingSides),
                verticalArrangement = Arrangement.spacedBy(rowSpacing)
            ) {
                // Button to confirm IP address and port
                Button(
                    onClick = {
                        textfield = "Text for connection guide"
                    },
                    modifier = Modifier
                        .padding(20.dp)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Connection guide")
                }

                // Button to confirm IP address and port
                Button(
                    onClick = {
                        textfield = "Text for application features"
                    },
                    modifier = Modifier
                        .padding(20.dp)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Application features")
                }

                // Button to confirm IP address and port
                Button(
                    onClick = {
                        textfield = "Text for trubleshooting"
                    },
                    modifier = Modifier
                        .padding(20.dp)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Trubleshooting")
                }
            }
        }
        VerticalDivider(
            modifier = Modifier
                .padding(top = 30.dp, bottom = 30.dp)
                .fillMaxHeight()
                .width(1.dp),
            color = Color.Gray
        )

        // Second column for the SSH Terminal, currently unused
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing)
        ) {
            Text(
                text = textfield
            )
        }
    }
}



