# Garage Opener WiFi Laser

Firmware for a custom-built garage door opener/monitor based on a bare
ESP-12E module. It:

- Uses a VL53L0X time-of-flight laser distance sensor to measure the
  distance from a fixed point (e.g. the back wall) to the car bumper, and
  infers whether the garage door is open or closed from that distance.
- Publishes distance and door-open/closed status over MQTT.
- Exposes a small authenticated web UI (`/gui`) showing live status and a
  button to trigger the garage door relay, plus a `/config` page for tuning
  the sensor/door-detection behavior (values are stored in flash and
  persist across reboots).
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

## Using deploy.ps1

`deploy.ps1` is a PowerShell wrapper around the `pio` commands above, so
you don't have to remember environment names and upload-port flags.

Run it with no arguments for an interactive menu (build / flash via
serial / flash via OTA / monitor):

```
.\deploy.ps1
```

Or drive it non-interactively with switches:

```
.\deploy.ps1 -Build
.\deploy.ps1 -FlashSerial -Port COM8
.\deploy.ps1 -FlashOTA -IPAddress 192.168.1.90
.\deploy.ps1 -Monitor -Port COM8
```

`-Port`/`-IPAddress` are optional — if omitted, it'll prompt (with a
picker for available serial ports).

## Configuration

The MQTT server address and sensor/door-detection tuning (door-open
distance threshold, poll interval, MQTT resend interval, change-percent
threshold, sensor out-of-range threshold, and door-state confirm count)
are all editable at runtime on the `/config` page and stored in flash
(EEPROM), so changes survive reboots and reflashes and take effect
immediately (no reboot required, including MQTT server changes — the
client drops and reconnects to the new broker right away). The
`DEFAULT_*` `#define`s near the top of `src/GarageOpenerWifiLaser.cpp`
only supply the very first values written to flash — after that,
`/config` is the source of truth.

A few other values are set as constants at the top of
`src/GarageOpenerWifiLaser.cpp` and may need adjusting for your setup:

- `webUser` / `webPass` — credentials for the web UI (`/gui`, `/config`,
  `/status`, `/button`).
