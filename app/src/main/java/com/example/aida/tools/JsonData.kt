package com.example.aida.tools

data class Step(
    val step: Int,
    val action: String
)

data class Instructions(
    val steps: List<Step>
)