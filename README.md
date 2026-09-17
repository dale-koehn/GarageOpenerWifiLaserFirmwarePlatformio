# Garage Opener WiFi Laser

Firmware for a custom-built garage door opener/monitor based on a bare
ESP-12E module. It:

- Uses a VL53L0X time-of-flight laser distance sensor to measure the
  distance from a fixed point (e.g. the back wall) to the car bumper, and
  infers whether the garage door is open or closed from that distance.
- Publishes distance and door-open/closed status over MQTT.
- Exposes a small authenticated web UI (`/gui`) showing live status and a
  button to trigger the garage door relay.
- Can trigger the door relay either from the web UI or via an MQTT command.
- Uses [WiFiManager](https://github.com/tzapu/WiFiManager) for WiFi
  provisioning (connect to the `GarageOpener` AP to configure WiFi
  credentials on first boot or after a reset).

## Hardware

- Bare ESP-12E module (no dev board), on a custom PCB.
- VL53L0X distance sensor over I2C (SDA on GPIO4, SCL on GPIO5).
- A relay (GPIO13) that pulses to simulate a garage door button press.
- Status LEDs on GPIO12 (relay active), GPIO14 (door open/closed), GPIO16
  (power/general status).

## Building

```
pio run
```

This builds the default `esp12e` (serial) environment. To build both
environments (serial and OTA):

```
pio run -e esp12e -e esp12e_ota
```

## Flashing over serial

The ESP-12E on this board has no auto-reset circuitry, so it must be put
into flash mode manually before uploading:

1. Ground GPIO0.
2. With GPIO0 grounded, pulse RESET (or power-cycle the board).
3. Run the upload:

   ```
   pio run -e esp12e -t upload --upload-port COM8
   ```

   (Replace `COM8` with whatever serial port your USB-to-serial adapter
   enumerates as.)
4. Release GPIO0 and reset/power-cycle the board again to boot into the
   application (it will otherwise stay in the bootloader after flashing).

To watch serial output:

```
pio device monitor -e esp12e --port COM8
```

## Flashing over OTA (WiFi)

Once the device is running firmware with `ArduinoOTA` active (which every
build here includes), you can flash it over WiFi with no serial
connection or manual reset required:

```
pio run -e esp12e_ota -t upload
```

This uploads to `garage-opener_v3-esp8266.local` (the device's mDNS
hostname) by default. If mDNS resolution isn't working on your network,
override it with the device's IP address:

```
pio run -e esp12e_ota -t upload --upload-port 192.168.1.90
```

Note: OTA is currently configured with no password
(`ArduinoOTA.setPassword` is intentionally left disabled), so anyone on
the local network can push firmware to the device.

## Configuration

A few values are set as constants at the top of
`src/GarageOpenerWifiLaser.cpp` and may need adjusting for your setup:

- `mqtt_server` — IP address of your MQTT broker.
- `DOOR_OPEN_MIN_DIST` — distance (mm) below which the door is considered
  open.
- `webUser` / `webPass` — credentials for the web UI (`/gui`, `/status`,
  `/button`).
