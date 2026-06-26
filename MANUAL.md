# WX Station — Firmware Manual

Firmware for the 3D-printed weather station, running on an **Olimex ESP32-POE** board.
Firmware revision string: see `REV` in `wx.ino` (date-based, e.g. `20260626`).

- Wiki: https://remoteqth.com/w/doku.php?id=3d_print_wx_station
- Source: [`wx.ino`](wx.ino) · Web UI: [`data/`](data/) (SPIFFS)

---

## 1. Overview

The station samples wind (direction + speed), rain, temperature, humidity and
barometric pressure, and distributes the data over four channels:

- **MQTT** — telemetry to a broker (default public `54.38.157.134:1883`).
- **APRS-IS** — weather frame to the APRS network (`czech.aprs2.net:14580`).
- **windy.com** — PWS upload (HTTPS).
- **Web dashboard** — local responsive page plus a setup/diagnostics UI.

Sensors supported (compile-time `#define`, auto-detected at boot): HTU21D
(humidity + temperature), BMP280 (pressure + temperature), DS18B20 (external
temperature, used as master when present), optional RF69 radio sensor.

---

## 2. Network services & ports

| Service | Port | Notes |
|---|---|---|
| Web dashboard + setup | 80 | `/` dashboard, `/setup` config, `/wall` MQTT wall |
| OTA update (firmware + filesystem) | 82 | `http://<ip>:82/update` |
| Telnet console | 23 | key-authenticated (key printed on serial at boot) |
| MQTT | broker-defined | default broker `54.38.157.134:1883` |

mDNS is advertised, so the station is also reachable by hostname `WX-<CALLSIGN>`.

---

## 3. Web interface

### Dashboard `/`
Public responsive meteo page. Polls `/api/live` every 5 s (server-side cache
refreshes every 15 s).

### Setup `/setup`
Tabbed configuration and diagnostics page. Optionally protected by HTTP Basic
auth (see §6). Tabs:

- **Station** — altitude, callsign/name (`CALLSIGN-ssid`), rain mm/pulse, wind
  direction offset, temperature calibration.
- **Network** — MQTT broker IP/port, APRS (enable, password, coordinates),
  windy.com (API key, station ID). APRS sends as the **`CALLSIGN-ssid`** value
  from the Station tab.
- **System** — web security (password), diagnostics (firmware, uptime, MAC, IP,
  NTP, heap, MQTT, SPIFFS).
- **Status** — full live sensor dump including **raw / debug values** (mirrors the
  telnet status block): wind-direction register, RPM pulse periods, RainPin ADC,
  raw vs calibrated temperatures, raw barometric pressure. Polls `/api/status`
  every 5 s while the tab is open.
- **Debug** — serial debug level + recent log lines.

### HTTP API
| Endpoint | Auth | Purpose |
|---|---|---|
| `GET /api/live` | public | cached meteo JSON (dashboard) |
| `GET /api/status` | gated | full sensor + raw debug dump (Status tab) |
| `GET /api/config` | gated | settings + diagnostics (secrets masked) |
| `POST /api/config` | gated | save settings |
| `GET /api/debug` | gated | debug level + log ring buffer |
| `GET /api/wallcfg` | public | broker WS URI + topic for `/wall` |

---

## 4. Data publishing — intervals

All channels are driven by a single **publish cycle** (`wx.ino`, default
**300 000 ms = 5 min**, configurable via the telnet/serial `x` command). When the
cycle fires (ETH + MQTT online) it measures, then publishes to MQTT, APRS and
windy.com together.

| Channel | Interval | Trigger | Payload |
|---|---|---|---|
| **MQTT** | 5 min (configurable `x`) | publish cycle | all meteo values + FreeHeap |
| **MQTT on-demand** | immediate | message to `CALLSIGN/WX/get` (any payload) | meteo values (`MqttPubValue()`) — not APRS/windy |
| **MQTT RF sensor** | on packet (poll ~2 s) | RF69 data received | RF temperature / humidity / battery |
| **MQTT ip/mac** | once per (re)connect | broker connect | IP + MAC (retained) |
| **APRS** | 5 min | publish cycle | one WX frame |
| **windy.com** | 5 min | publish cycle | PWS upload (only if API key set) |
| **Dashboard `/api/live`** | cache 15 s, browser poll 5 s | `GetValue(false)` (fast sensors, no DS18B20) | temperature, humidity, wind, pressure, rain |
| **Setup → Status `/api/status`** | 5 s (tab open) | live read | all values incl. raw / debug |

**Notes**

- APRS and windy.com upload **only** in the 5-min cycle; `/get` republishes MQTT
  only, it does not trigger APRS/windy.
- Dashboard effective latency is ~15 s (cache refresh), except at the 5-min full
  measurement; the **Status** tab is the only truly 5 s live view.
- The whole cycle is gated on `eth_connected` **and** `mqttClient.connected()` —
  with no MQTT connection the cycle (and therefore APRS/windy) does not run.

### MQTT topics (base `CALLSIGN/WX/...`, published every cycle)
`utc`, `WindDir-azimuth`, `RainCount`, `RainToday-mm`, `WindSpeedAvg-mps`,
`WindSpeedMaxPeriod-mps`, `WindSpeedMaxPeriod-utc`, `Pressure-hPa-BMP280`,
`Temperature-Celsius-BMP280`, `HumidityRel-Percent-HTU21D`,
`DewPoint-Celsius-HTU21D`, `Temperature-Celsius-DS18B20`, `Pressure-hPa`,
`HumidityRel-Percent`, `DewPoint-Celsius`, `Temperature-Celsius` (published last
— acts as the e-ink update trigger), `FreeHeap`.

Subscribed: `CALLSIGN/WX/get` (any payload → immediate re-publish of meteo values).

---

## 5. Serial / Telnet console

A single-character menu is available on the USB serial port and over telnet
(telnet additionally requires the access key printed at boot). Press `?` to list
commands. Key commands:

| Key | Action |
|---|---|
| `?` | refresh status / list commands |
| `m` | set altitude (sea-level pressure conversion) |
| `L` | set callsign / location name (`CALLSIGN-ssid`) |
| `R` | set rain mm per pulse |
| `S` | set wind-direction offset |
| `C` | set temperature calibration |
| `x` | set TX repeat time (publish-cycle minutes) |
| `+` | set MQTT broker IP/port (restarts) |
| `A` | toggle APRS, `p` password, `c` coordinates |
| `K` | set windy.com API key · `I` set windy.com station ID |
| `P` | clear web password / disable auth (**recovery**, see §6) |
| `e` | dump EEPROM · `E` erase whole EEPROM (serial only) |
| `W` | erase wind-speed max memory |
| `@` | restart device · `*` toggle serial debug |
| `q` / `Q` | close telnet / logout + forget verified IP |

---

## 6. Web authentication & recovery

The `/setup` page and its write APIs are protected by HTTP Basic auth **only when
a web password is set** (System tab → *Web password*). User is fixed (`admin`).

Because `/setup` is locked behind that password, **disabling auth / recovering a
lost password is not done from the web** — it lives in the out-of-band console:

> Press **`P`** in the serial or telnet menu → confirm → the password is cleared,
> auth is disabled, and the device restarts.

Setting/changing the password is done from the web (when authenticated); clearing
it is console-only.

---

## 7. Building & flashing

Target board: **Olimex ESP32-POE**, ESP32 Arduino core 2.0.14.

```sh
# from the sketch directory
arduino-cli compile --fqbn esp32:esp32:esp32-poe .
```

- Firmware baseline footprint: ~92 % flash, ~18 % static RAM.
- The web UI lives in SPIFFS (`data/`). Changes to `data/*.html` require a
  **filesystem (SPIFFS) upload** — re-flashing only the firmware `.bin` does not
  update the web pages. Use the OTA page (`:82/update`, filesystem image) or the
  filesystem-upload tooling.

## 8. Releasing firmware & the download page

Compiled binaries are published to GitHub Pages at
<https://ok1hra.github.io/3D-print-WX-station/>, which offers both EasyOTA
`.bin` downloads and a browser-based USB recovery installer (esp-web-tools).
The OTA page (`:82/update`) reads that site's `manifest.json` and shows whether a
newer firmware is available.

**One-time setup:** in the GitHub repo, Settings → Pages → Source "Deploy from a
branch" → Branch `gh-pages`, folder `/(root)`. The `gh-pages` branch is created
automatically by the publish script on its first run.

**Release a new version:**

1. Increase `REV` in `wx.ino`.
2. Arduino IDE 1.8.19: Sketch → Export compiled Binary (board ESP32-POE; the
   sketch-local `partitions.csv` is applied automatically).
3. Build all images and publish to GitHub Pages:

   ```sh
   ./tools/all.sh --publish
   ```

4. `git commit` with the release number as the message, then `git push`.

Only the latest release is kept on `gh-pages` (older versioned `wx-*.bin`
files are replaced on each publish).

**Partial update (without a release):**

1. Arduino IDE: Sketch → Export compiled Binary.
2. Build the SPIFFS image: `tools/build_spiffs_image.sh`.
3. Upload `wx.ino.esp32-poe.bin` and `build/spiffs.bin` via EasyOTA
   (`:82/update`, firmware first, then filesystem).
4. `git commit` and push.

The `tools/` scripts:

| Script | Purpose |
| --- | --- |
| `build_spiffs_image.sh` | pack `data/` into `build/spiffs.bin` (mkspiffs) |
| `build_recovery_package.sh` | assemble `build/recovery-web/` (versioned `.bin`, merged image, `manifest.json`, `index.html`) |
| `publish_recovery_to_gh_pages.sh` | push `build/recovery-web/` to the `gh-pages` branch |
| `all.sh` | run the SPIFFS + recovery build, `--publish` also pushes |

See [`WINDY.md`](WINDY.md) for windy.com setup details.
