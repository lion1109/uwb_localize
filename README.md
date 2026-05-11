# uwb-localization

Firmware for Makerfabs DW3000 ESP32 UWB boards providing precise indoor and GNSS-independent localization.

## Motivation

Ultra Wideband (UWB) localization enables accurate positioning without relying on GNSS/GPS signals.
This allows reliable positioning:

* indoors
* underground
* in industrial environments
* in urban canyons
* in areas with poor satellite reception

The goal of this project is to provide a simple and performant localization firmware for Makerfabs DW3000 ESP32 UWB boards.

---

## Features

* Supports operation as:

  * **Anchor**
  * **Rover**
* 2D and 3D localization
* GNSS-independent positioning
* Target accuracy of approximately **2 cm**
* Designed for areas up to:

  * **40 m diameter**
  * approximately **20 m radius**
* Rover output compatible with:

  * RTK GNSS receiver style data
  * simplified euclidean coordinate protocol
* Supports:

  * minimum of 4 anchors
  * unlimited number of passive/listening-only rovers
* Dynamic addition of anchors to:

  * improve range
  * improve localization accuracy
* Built on **FreeRTOS** for high-performance and scalable firmware architecture

---

## Hardware

Supported hardware:

* Makerfabs ESP32 UWB DW3000 boards

---

## Planned Architecture

### Anchors

Anchors are fixed-position UWB nodes with known coordinates.

Responsibilities:

* participate in ranging
* provide timing references
* extend localization coverage
* improve geometric accuracy

### Rovers

Rovers are mobile nodes whose position is calculated using distances to anchors.

Supported rover modes:

* active localization rover
* passive listening rover

---

## Localization Goals

| Parameter       | Target                   |
| --------------- | ------------------------ |
| Accuracy        | ~2 cm                    |
| Dimensions      | 2D / 3D                  |
| Coverage        | up to 40 m               |
| Minimum anchors | 4                        |
| Rover count     | unlimited passive rovers |

---

## Firmware Design

The firmware is based on FreeRTOS to allow:

* efficient task scheduling
* responsive communication handling
* scalable architecture
* future protocol extensions
* deterministic timing behavior

---

## Future Goals

* NMEA-compatible rover output
* Web configuration interface
* Dynamic network topology
* OTA firmware updates
* Anchor auto-discovery
* Mesh synchronization
* ROS2 integration

---

## Project Status

Work in progress.

---

## License

MIT License
