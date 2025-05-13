// Top-level build file where you can add configuration options common to all sub-projects/modules.
plugins {
    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.kotlin.android) apply false
}

buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath("com.android.tools.build:gradle:8.6.1")
        classpath(kotlin("gradle-plugin:2.0.21"))
        // NOTE: Do not place your application dependencies here; they belong
        // in the individual module build.gradle files
    }
}
