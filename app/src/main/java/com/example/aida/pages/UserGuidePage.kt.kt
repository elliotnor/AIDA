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

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment

import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.example.aida.viewmodels.MainViewModel
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import java.io.File
import android.content.Context
import com.example.aida.MainActivity
import com.example.aida.R
import java.io.BufferedReader
import java.io.InputStreamReader
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

import androidx.compose.foundation.Image

import androidx.compose.ui.res.painterResource


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
    onButtonPress: () -> Unit,
    activity: MainActivity,
) {
    Row(
        modifier = Modifier
            .padding(top = barHeight)
            .fillMaxSize(),
    ) {
        val paddingTop = 20.dp
        val paddingSides = 30.dp
        val rowSpacing = 10.dp
        var textfield1 by remember { mutableStateOf("") }
        var textfield2 by remember { mutableStateOf("") }

        var image1 by remember { mutableStateOf(false) }
        val painter = if (image1) {
            painterResource(id = R.drawable.aida_homepage)
        } else {
            painterResource(id = R.drawable.white)
        }




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
                        val jsonContent = readRawResource(activity, R.raw.connection_guide)
                        textfield1 = "Start of AIDA"
                        textfield2 = jsonContent
                        image1 = false

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

                        textfield1 = "Application features"
                        textfield2 = ""
                        image1 = true
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
                        val jsonContent = readRawResource(activity, R.raw.trouble_shooting)
                        textfield1 = "Trubleshooting of AIDA"
                        textfield2 = jsonContent
                        image1 = false

                    },
                    modifier = Modifier
                        .padding(20.dp)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Trubleshooting")
                }

                Image(
                    painter =  painterResource(id = R.drawable.aida_logo_new),
                    contentDescription = "Example Image",
                    modifier = Modifier
                        .height(600.dp)
                        .width(600.dp)
                )
            }
        }
        VerticalDivider(
            modifier = Modifier
                .padding(top = 30.dp, bottom = 30.dp)
                .fillMaxHeight()
                .width(10.dp),
            color = Color.Gray
        )

        // Second column for the SSH Terminal, currently unused
        Column(
            modifier = Modifier
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing)
        ) {
            Text(
                text = textfield1,
                fontSize = 24.sp // Adjust the size as needed
            )


            Text(
                text = textfield2
            )

            Image(
                painter =  painter,
                contentDescription = "Example Image",
                modifier = Modifier
                    .height(700.dp)
                    .width(700.dp)
            )



        }
    }
}
//"app/src/main/res/raw/connection_guide.txt"


fun readRawResource(context: Context, resourceId: Int): String {
    val inputStream = context.resources.openRawResource(resourceId)
    val reader = BufferedReader(InputStreamReader(inputStream))
    val content = StringBuilder()
    reader.use {
        var line = it.readLine()
        while (line != null) {
            content.append(line).append("\n")
            line = it.readLine()
        }
    }
    return content.toString()

    // If you need to parse the JSON content, you can use a library like Gson or kotlinx.serialization
    // For example, using Gson:
    //val gson = Gson()
    //val instructions: Instructions = gson.fromJson(string, Instructions::class.java)
    //return instructions
}

data class Step(
    val row: Int,
    val text: String
)

data class Instructions(
    val row: List<Step>
)



