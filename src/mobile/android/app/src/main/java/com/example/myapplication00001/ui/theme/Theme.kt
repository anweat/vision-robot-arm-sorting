package com.example.myapplication00001.ui.theme

import android.os.Build
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.dynamicDarkColorScheme
import androidx.compose.material3.dynamicLightColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext

private val DarkColorScheme = darkColorScheme(
    primary = Color(0xFF4FB3D9),
    onPrimary = Color(0xFF0F1A22),
    background = Color(0xFF0F1A22),
    surface = Color(0xFF162531),
    surfaceVariant = Color(0xFF213441),
    onBackground = Color(0xFFE3EDF3),
    onSurface = Color(0xFFE3EDF3),
    onSurfaceVariant = Color(0xFFB4C7D3),
    outline = Color(0xFF8FA5B4),
)

private val LightColorScheme = lightColorScheme(
    primary = Color(0xFF146889),
    background = Color(0xFFF3F6F8),
    surface = Color.White,
    surfaceVariant = Color(0xFFE6EFF4),
    onBackground = Color(0xFF152633),
    onSurface = Color(0xFF152633),
    onSurfaceVariant = Color(0xFF405867),
    outline = Color(0xFF627B8B),
)

@Composable
fun MyApplication00001Theme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    // Dynamic color is available on Android 12+
    dynamicColor: Boolean = true,
    content: @Composable () -> Unit
) {
    val colorScheme = when {
        dynamicColor && Build.VERSION.SDK_INT >= Build.VERSION_CODES.S -> {
            val context = LocalContext.current
            if (darkTheme) dynamicDarkColorScheme(context) else dynamicLightColorScheme(context)
        }

        darkTheme -> DarkColorScheme
        else -> LightColorScheme
    }

    MaterialTheme(
        colorScheme = colorScheme,
        typography = Typography,
        content = content
    )
}
