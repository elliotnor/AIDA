package com.example.aida.viewmodels

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.material.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import kotlinx.coroutines.delay
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.livedata.observeAsState
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.aida.R
import kotlinx.coroutines.MainScope
import kotlinx.coroutines.launch
/**
 * Simple ViewModel used to display the AIDA logo during launch
 */
class SplashViewModel : ViewModel() {
    private val _isLoading = MutableLiveData(true)
    val isLoading: LiveData<Boolean> = _isLoading

    init {
        val loadingTime = 3000 //milliseconds
        MainScope().launch {
            delay(loadingTime)
            _isLoading.value = false
        }
    }
}

/**
 * Create the splash screen composable
 */
@Composable
fun SplashScreen(splashViewModel: SplashViewModel = viewModel(), navigateToMain: () -> Unit) {
    val isLoading by splashViewModel.isLoading.observeAsState(true)

    if (!isLoading) { //When finished -> Go to main screen
        navigateToMain()
    }

    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        Image(
            painter =  painterResource(id = R.drawable.aida_logo_new),
            contentDescription = "Splash Image",
            modifier = Modifier.size(200.dp)
        )
    }
}

@Preview(showBackground = true)
@Composable
fun SplashScreenPreview() {
    MaterialTheme {
        SplashScreen(navigateToMain = {}) //Go to main screen after delay
    }
}