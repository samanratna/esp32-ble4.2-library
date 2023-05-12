| Supported Targets | ESP32 | ESP32-C3 |
| ----------------- | ----- | -------- |

# ESP32 BLE Server Library

## Features

- BLE Connectivity with MITM Protection
- Maintain Consistent BLE Connectivity
- BLE Auto Connection
- Have RSSI Value for Proximity of Connected Devices

## Supports

- BLE Beacons for Raw Advertisement
- BLE Gateway for routing data to the Internet

## Pre-requisites

- ESP-IDF is version v5.0 or above
- Requires console for BLE Connection Monitoring

## Execution

```
$ idf.py fullclean
$ idf.py build
$ idf.py flash monitor
```
