// Top-level build file where you can add configuration options common to all sub-projects/modules.
plugins {
    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.kotlin.android) apply false
    id("com.google.dagger.hilt.android") version "2.56.2" apply false
    id("com.google.devtools.ksp") version "2.1.21-2.0.1"
}

buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath("com.android.tools.build:gradle:8.10.1")
        classpath(kotlin("gradle-plugin:2.0.21"))
        // NOTE: Do not place your application dependencies here; they belong
        // in the individual module build.gradle files
    }
}
