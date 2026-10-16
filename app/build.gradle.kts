plugins {
    id("com.android.application")
}

android {
    namespace = "com.example.robotcar"
    compileSdk = 37

    defaultConfig {
        applicationId = "com.example.robotcar"
        minSdk = 26
        targetSdk = 36
        versionCode = 24
        versionName = "0.5.21-bounded-backoff-avoidance"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    packaging {
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
        }
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.19.0")
    implementation("androidx.appcompat:appcompat:1.8.0")
    implementation("androidx.activity:activity-ktx:1.13.0")

    implementation("androidx.camera:camera-camera2:1.6.2")
    implementation("androidx.camera:camera-lifecycle:1.6.2")
    implementation("androidx.camera:camera-view:1.6.2")

    implementation("com.microsoft.onnxruntime:onnxruntime-android:1.30.0")
    implementation("com.google.ai.edge.litert:litert:1.4.2")
    implementation("com.github.mik3y:usb-serial-for-android:3.11.0")

    testImplementation("junit:junit:4.13.2")
}
