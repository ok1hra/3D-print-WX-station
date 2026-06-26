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

### Wind measurement architecture (dual-core)

Wind-speed pulses from the anemometer are counted in an **IRAM-safe interrupt
(ISR)** that does all min / max / average accumulation itself, under a spinlock.
Because the ISR runs on every pulse regardless of what `loop()` is doing, **no
gust is lost** — not while the main loop is busy serving the web, MQTT or the
multi-second windy.com TLS upload, and not during EEPROM writes (the interrupt is
no longer detached for those).

A dedicated **`WindTask` pinned to core 0** (the Ethernet/lwIP core, which has
spare capacity because there is no Wi-Fi driver task) refreshes the live wind
cache and pushes it to the dashboard. It is non-critical to measurement: the gust
accumulation lives in the ISR, so even if the task stalled, no data would be lost.
This makes high-wind capture and real-time web display fully independent.

---

## 2. Network services & ports

| Service | Port | Notes |
|---|---|---|
| Web dashboard + setup | 80 | `/` dashboard, `/setup` config, `/wall` MQTT wall |
| OTA update (firmware + filesystem) | 82 | `http://<ip>:82/update` |
| Live-wind SSE stream | 82 | `/events` — Server-Sent Events, CORS-enabled (dashboard needle) |
| Telnet console | 23 | key-authenticated (key printed on serial at boot) |
| MQTT | broker-defined | default broker `54.38.157.134:1883` |

mDNS is advertised, so the station is also reachable by hostname `WX-<CALLSIGN>`.

All web services (dashboard, setup, OTA, SSE) are served by the station itself, so
the UI works on an **isolated LAN without internet** — see §3.

---

## 3. Web interface

### Dashboard `/`
Public responsive meteo page, served from SPIFFS. Layout: large temperature, a
sensor grid (humidity, pressure, rain, dew point, **Wind max 5 min**, **wind
direction**) and a **wind-speed gauge** with a coloured needle.

Live wind is **pushed** from the firmware over **Server-Sent Events** (`/events`
on port 82) by the core-0 `WindTask`: a new pulse triggers a push (rate-limited to
≤10 Hz) plus a 1 s keep-alive, so the gauge needle and direction track gusts
within ~50–100 ms without polling. If the SSE stream is unavailable the page
transparently falls back to polling `/api/wind` every 2 s. The slower sensors
(temperature, humidity, pressure, rain) refresh from `/api/live` every 30 s.

- **Gauge** — 0–25 m/s, scale labels every 5 m/s, three colour zones (green ≤8,
  orange ≤14, red >14 m/s). The needle (and its colour) follow the instantaneous
  speed. Above 25 m/s the needle **pins at full scale** but the numeric read-out
  still shows the **true value** (e.g. 28.0 m/s); the same is true for *Wind max
  5 min*.
- **Wind max 5 min** — a true rolling maximum over the trailing **exactly 5.0 min**
  (1-second resolution), computed independently of the publish cycle. It is
  therefore **decoupled** from the `WindSpeedMaxPeriod-mps` value sent to MQTT /
  APRS / windy.com (which resets every publish cycle).

#### Offline / island operation
The dashboard is fully **local** and works on a LAN with **no internet**: the page,
`/api/live`, `/api/wind` and the `/events` push all come from the station. The
measurement path is **decoupled from MQTT** — the 5-min measurement cycle runs on
the ETH link alone, so the **DS18B20** external temperature (every 5 min), the
today's-rain EEPROM persistence and the period-max reset all keep working without a
broker. The software watchdog reboots only on **ETH** loss, never on MQTT loss, so
an island station stays up. Only genuinely internet-dependent extras degrade:

- **NTP clock** — without internet `configTime`/NTP can't sync, so timestamps
  (UTC, *Wind max* time) show `n/a`; all measured values still update.
- **External links** (aprs.fi, windy.com) and the **uploads** (MQTT / APRS /
  windy) need connectivity; their absence does not affect the live dashboard.

The station needs a valid IP on the LAN (local DHCP or a static address).

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
| `GET /api/live` | public | cached meteo JSON (dashboard, 30 s) |
| `GET /api/wind` | public | small wind JSON: live dir + speed + rolling 5-min max (fast-poll fallback) |
| `GET /events` (port 82) | public | SSE live-wind stream pushed by `WindTask` (dir, speed, rolling max) |
| `GET /api/status` | gated | full sensor + raw debug dump (Status tab) |
| `GET /api/config` | gated | settings + diagnostics (secrets masked) |
| `POST /api/config` | gated | save settings |
| `GET /api/debug` | gated | debug level + log ring buffer |
| `GET /api/wallcfg` | public | broker WS URI + topic for `/wall` |

---

## 4. Data publishing — intervals

All channels are driven by a single **measurement cycle** (`wx.ino`, default
**300 000 ms = 5 min**, configurable via the telnet/serial `x` command). The cycle
fires on the **ETH link alone** — it measures (incl. DS18B20), persists today's
rain and resets the period accumulators **regardless of MQTT**. The network uploads
(MQTT, APRS, windy.com) run within the same cycle **only when MQTT is connected**
(used as an "internet is up" proxy), so an offline station still measures and
refreshes everything locally.

| Channel | Interval | Trigger | Payload |
|---|---|---|---|
| **MQTT** | 5 min (configurable `x`) | measurement cycle, if MQTT connected | all meteo values + FreeHeap |
| **MQTT on-demand** | immediate | message to `CALLSIGN/WX/get` (any payload) | meteo values (`MqttPubValue()`) — not APRS/windy |
| **MQTT RF sensor** | on packet (poll ~2 s) | RF69 data received | RF temperature / humidity / battery |
| **MQTT ip/mac** | once per (re)connect | broker connect | IP + MAC (retained) |
| **APRS** | 5 min | measurement cycle, if MQTT connected | one WX frame |
| **windy.com** | 5 min | measurement cycle, if MQTT connected | PWS upload (only if API key set) |
| **DS18B20 / rain persist / period reset** | 5 min | measurement cycle (ETH only) | runs offline too — independent of MQTT |
| **Dashboard live wind** | push via SSE `/events`: on-pulse ≤10 Hz + 1 s keep-alive | core-0 `WindTask` | instantaneous speed, direction, rolling 5-min max |
| **Dashboard `/api/live`** | cache 30 s, browser poll 30 s | `GetValue(false)` (fast sensors, no DS18B20) | temperature, humidity, pressure, rain (+ wind snapshot) |
| **Setup → Status `/api/status`** | 5 s (tab open) | live read | all values incl. raw / debug |

**Notes**

- APRS and windy.com upload **only** in the 5-min cycle; `/get` republishes MQTT
  only, it does not trigger APRS/windy.
- Dashboard **wind** (speed, direction, rolling max) is real-time via SSE push;
  the other sensors have ~30 s latency (fast cache). The **Status** tab is a 5 s
  full live view.
- `WindSpeedMaxPeriod-mps` (MQTT/APRS/windy) is the max of the **current publish
  period** (resets every cycle); the dashboard *Wind max 5 min* is a separate
  rolling 5-min value — the two will usually differ.
- The cycle runs on `eth_connected`; only the MQTT/APRS/windy uploads are gated on
  `mqttClient.connected()`. With no broker the station still measures, refreshes
  DS18B20, persists rain and resets the period — only the uploads pause (see §3,
  offline operation).

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

- Firmware baseline footprint: ~93 % flash, ~18 % static RAM.
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
