project21-firmware


## Installing and Setting up PlatformIO

1. Install VSCode.
1. Look for the extension 'PlatformIO' and install it.
1. Ensure that you also have installed the 'Arduino IDE' seperately.
1. In the 'Arduino IDE', ensure that you have installed the 'Intel Curie Boards' package internally.
1. In 'Platform IO', you should be able to open the firmware project from the homepage.
1. After editing, ensure that you have selected the correct Arduino board version "Arduino/Genuino 101" and programmer "USBtinyISP", on the toolbar below. If not found, restart VSCode or reinstall the packages.

## platformio.ini Configuration

You need to disable FreeRTOS by excluding it from the dev environment. See the below example configuration.

"""
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:genuino101]
platform = intel_arc32
board = genuino101
framework = arduino

lib_deps =
    #SD
    868
    Madgwick
    CurieBLE
    CurieIMU

lib_ignore =
    FreeRTOS
"""