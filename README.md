BimmerGarageSoftware



Overview

Dual-ESP32 setup that reads CAN data from the OBD2 port and displays it wirelessly on a TFT screen using LVGL.

CAN Sender – Reads vehicle CAN data via TWAI + CAN transceiver

Display Receiver – Receives data via ESP-NOW and renders dashboard UI

Features

            -Direct CAN Bus reading (no ELM327)

            -ESP-NOW wireless communication

            -LVGL real-time dashboard

Live data display:

            -Coolant temperature

            -Oil pressure

            -Oil temperature

            -Battery level

Hardware

            -ESP32 (WROOM-32)

            -SN65HVD230 CAN transceiver

            -OBD2 connector

            -TFT display (ST7789 240x320)

            12V → 5V DC-DC converter

⚠️ Never connect ESP32 directly to CAN lines without a transceiver.

Tech Stack

Arduino • ESP-NOW • ESP32 TWAI • LVGL • LovyanGFX
