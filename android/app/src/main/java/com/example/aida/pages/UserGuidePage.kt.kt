package com.example.aida.pages

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.material3.VerticalDivider

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment

import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import android.content.Context
import com.example.aida.MainActivity
import com.example.aida.R
import java.io.BufferedReader
import java.io.InputStreamReader
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.sp

import androidx.compose.foundation.Image
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.ui.graphics.painter.Painter

import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight


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
    activity: MainActivity,
) {
    Row(
        modifier = Modifier
            .padding(top = barHeight)
            .fillMaxSize(),
    ) {

        val padding = 20.dp
        val paddingSides = 30.dp
        val paddingPage = 100.dp
        val rowSpacingForButton = 10.dp
        val spacer = 25.dp
        val smallSpacer = 5.dp
        val titleSize = 25.sp
        val logoSize = 700.dp
        val imageSizeWidth = 400.dp
        val imageSizeHeight = 300.dp
        val smallImageSize = 1.dp

        var pictureWidth by remember { mutableStateOf(imageSizeWidth) }
        var pictureHeight by remember { mutableStateOf(imageSizeHeight) }
        var title by remember { mutableStateOf("Connection guide") }
        var textContent by remember { mutableStateOf(readRawResource(activity, R.raw.connection_guide)) }


        //Creating variables for the images and if statement do decide if we should show the or not.
        // If it is false we only show 2 white pictures  that are one pixel long
        var showImage by remember { mutableStateOf(false) }
        var image1: Painter
        var image2: Painter
        if(showImage){
            image1 = painterResource(id = R.drawable.aida_demo_new_new)
            image2 = painterResource(id = R.drawable.aida_demo_connection)
            pictureWidth = imageSizeWidth
            pictureHeight = imageSizeHeight
        }
        else{
            image1 = painterResource(id = R.drawable.white)
            image2 = painterResource(id = R.drawable.white)
            pictureWidth = smallImageSize
            pictureHeight = smallImageSize
        }

        // First column with buttons switching between different guides
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding( start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacingForButton)
        ) {

            Column(
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
                    .padding(top = padding, start = paddingSides, end = paddingSides),
                verticalArrangement = Arrangement.spacedBy(rowSpacingForButton)
            ) {

                Text(text = "User guide",
                    modifier = Modifier.align(Alignment.CenterHorizontally),
                    fontWeight = FontWeight.Bold,
                    fontSize = titleSize,
                    fontFamily = FontFamily.SansSerif
                )


                Spacer(modifier = Modifier.height(spacer))

                // Button to access the Connection guide
                Button(
                    onClick = {
                        val jsonContent = readRawResource(activity, R.raw.connection_guide)
                        title = "Connection guide"
                        textContent = jsonContent
                        print(textContent)
                        showImage = false

                    },
                    modifier = Modifier
                        .padding(start = padding, end = padding, bottom = padding)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Connection guide")
                }

                // Button to access the Application features
                Button(
                    onClick = {

                        title = "Application features"
                        textContent = ""
                        showImage = true
                    },
                    modifier = Modifier
                        .padding(padding)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Application features")
                }

                // Button to access Troubleshooting
                Button(
                    onClick = {
                        val jsonContent = readRawResource(activity, R.raw.trouble_shooting)
                        title = "Troubleshooting"
                        textContent = jsonContent
                        showImage = false

                    },
                    modifier = Modifier
                        .padding(padding)
                        .align(Alignment.CenterHorizontally)
                        .fillMaxWidth(0.8f)
                ) {
                    Text("Troubleshooting")
                }

                // AIDA logo at the bottom of the left column
                Image(
                    painter =  painterResource(id = R.drawable.aida_logo_new),
                    contentDescription = "Example Image",
                    modifier = Modifier
                        .height(logoSize)
                        .width(logoSize)
                )
            }
        }

        // Divider between the two columns
        VerticalDivider(
            modifier = Modifier
                .padding(top = padding, bottom = padding)
                .fillMaxHeight()
                .width(padding),
            color = Color.Gray
        )

        val scrollState = rememberScrollState()
        // Second column to show of the page of the user guide
        Column(
            modifier = Modifier
                .padding(top = padding, start = paddingPage, end = paddingPage)
                .verticalScroll(scrollState),
            verticalArrangement = Arrangement.spacedBy(rowSpacingForButton)
        ) {
            // The tile of the page
            Text(
                text = title,
                modifier = Modifier.align(Alignment.CenterHorizontally),
                fontWeight = FontWeight.Bold,
                fontSize = titleSize,
                fontFamily = FontFamily.SansSerif

            )

            //Spacer(modifier = Modifier.height(spacer))
            Spacer(modifier = Modifier.height(smallSpacer))




            // The text content of the page
            Text(
                text = textContent,
                fontFamily = FontFamily.Monospace
            )

            // The first image in case the application features button was pressed
            Image(
                painter =  image1,
                contentDescription = "Example Image",
                modifier = Modifier
                    .height(pictureHeight)
                    .width(pictureWidth)
            )
            // The second image in case the application features button was pressed
            Image(
                painter =  image2,
                contentDescription = "Example Image",
                modifier = Modifier
                    .height(pictureHeight)
                    .width(pictureWidth)
            )


        }
    }
}

// Function to receive and handle text data from a file
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

}




