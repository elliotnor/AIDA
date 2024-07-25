package com.example.aida.viewmodels
// ButtonViewModel.kt
import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

class ButtonViewModel : ViewModel() {
    private val _isButtonTwoEnabled = MutableStateFlow(true)
    val isButtonTwoEnabled: StateFlow<Boolean> = _isButtonTwoEnabled

    fun toggleButtonTwoState() {
        _isButtonTwoEnabled.value = !_isButtonTwoEnabled.value
    }
}