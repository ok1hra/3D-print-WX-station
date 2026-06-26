# Sending data to windy.com (PWS)

The WX station can send its measured data to [windy.com](https://www.windy.com)
as a Personal Weather Station (PWS). Communication uses HTTPS to
`stations.windy.com` in a Wunderground-compatible format.

> The feature is enabled in firmware via `#define WINDY` (file `wx.ino`).
> Without an **API key** nothing is sent — uploading is therefore **off by
> default** until you enter the key.

---

## 1. Get an API key

1. Sign in / register at <https://stations.windy.com>.
2. Under **My stations**, *Add a station*:
   - fill in the name, location (lat/lon), altitude and type.
3. After saving, windy generates an **API key** (a long string, up to 123
   characters) and a **Station ID** for the station.
4. Copy the **API key** — you will enter it into the station.

---

## 2. Set the key in the WX station

The key is entered through the configuration menu (telnet or serial console),
just like the other parameters (callsign, coordinates, altitude…).

1. Connect to the station:
   - **telnet** to the station IP (port 23), or
   - **USB serial console** (115200 Bd).
2. Press `?` to list the menu. You will see a new item:

   ```
   K  windy.com API key [empty, upload OFF]
   ```

3. Press `K`.
4. Paste the API key from windy and confirm with **Enter** (`;` on the serial
   console).
5. The station replies, for example:

   ```
   windy API key saved (96 chars)
   ```

   and the menu item changes to `K  windy.com API key [SET, upload ON]`.

The key is stored in EEPROM (addresses 244–367), so it **survives a restart** and
a power loss.

### Disabling uploads / erasing the key
Press `K` and confirm an **empty** input (just Enter). The key is erased and
uploading is turned off:

```
windy upload disabled (key erased)
```

---

## 2b. Public Station ID and the link on the main page

To make a **WINDY** link appear on the station's main web page (port 80) next to
the **APRS** link, also enter the station's public **Station ID**.

- **API key** (step 2) = the secret key used for uploading (~96 characters).
- **Station ID** = the short public identifier in the station's URL, e.g.
  `pws-f06ea43a` (found at <https://stations.windy.com> for the station, i.e. in
  the public URL `https://www.windy.com/station/pws-…`).

To set it:

1. In the menu press `I`.
2. Paste the Station ID (e.g. `pws-f06ea43a`) and confirm with **Enter** (`;` on
   the serial console).
3. The station replies:

   ```
   windy Station ID saved | https://www.windy.com/station/pws-f06ea43a
   ```

The **WINDY** link is shown on the main page **only when both**:
- an API key is set (upload is active), and
- a Station ID is filled in.

Result in the page header:

```
… | APRS | WINDY | Upload FW | …
```

Erase the Station ID by entering an empty value for command `I` (the link then
hides).

---

## 3. Altitude and coordinates

For the data to make sense:

- Set the **altitude** in the menu with command `m` (also used to convert pressure
  to sea level).
- The station coordinates are entered directly on the windy.com website for that
  station.

---

## 4. How often it is sent

Data is sent at the same interval as the other outputs (MQTT, APRS) — the
**"TX repeat time"** in the menu (command `x`), default **5 minutes**.
Windy recommends sending no more often than about once every 5 minutes, so the
default value does not need changing.

---

## 5. What is sent (and in which units)

The firmware converts its internal metric values to the imperial units windy
expects:

| Quantity         | windy parameter | unit    | firmware conversion       |
|------------------|-----------------|---------|---------------------------|
| Wind direction   | `winddir`       | °       | –                         |
| Wind speed       | `windspeedmph`  | mph     | m/s ÷ 0.44704             |
| Wind gust        | `windgustmph`   | mph     | m/s ÷ 0.44704             |
| Temperature      | `tempf`         | °F      | °C × 1.8 + 32             |
| Rain (today)     | `rainin`        | inches  | mm ÷ 25.4                 |
| Pressure         | `baromin`       | inHg    | hPa × 0.02953             |
| Dew point        | `dewptf`        | °F      | °C × 1.8 + 32             |
| Humidity         | `humidity`      | %       | –                         |

---

## 6. Verification / debugging

Enable debugging from the menu with command `*` (debug). The console then shows:

```
[windy] connecting to stations.windy.com ...
[windy] free heap before: 224740
[windy] connected!
[windy] GET /pws/update/XXXX?winddir=...&windspeedmph=... HTTP/1.1
[windy] headers received
[windy RX] ... server response ...
[windy] stop
```

- `connection failed!` → check internet/DNS, the clock (TLS needs a valid time via
  NTP), and that the station has connectivity (Ethernet up, MQTT connected).
- `no API key set, skip upload` → no key is set (see step 2).
- Data appears for the station under **My stations** on windy.com within a few
  minutes.

---

## 7. Memory notes (important)

The feature was originally disabled because of low RAM. The current implementation
solves that:

- **The TLS client (`WiFiClientSecure`) is created locally** inside the upload
  function, so its ~30–40 kB of buffers are **freed** after each upload (it used to
  be global and held memory permanently).
- By default `client.setInsecure()` is used — the server **certificate is not
  verified**, which saves memory. For weather data this is acceptable.
- Free heap is printed before each upload (`free heap before:`), so the trend can
  be watched (it is also published via the MQTT topic `FreeHeap`).

### Optional: certificate verification
If you want full TLS certificate verification (higher RAM demand), uncomment in
`wx.ino`:

```cpp
#define WINDY_VERIFY_CERT
```

This enables verification against the built-in ISRG Root X1 certificate (valid
until 2035).
