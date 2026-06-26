# Odesílání dat na windy.com (PWS)

WX stanice umí posílat naměřená data do služby [windy.com](https://www.windy.com)
jako osobní meteostanice (PWS – Personal Weather Station). Komunikace probíhá přes
HTTPS na `stations.windy.com` ve formátu kompatibilním s Wunderground.

> Funkce je ve firmwaru zapnutá přes `#define WINDY` (soubor `wx.ino`).
> Bez vyplněného **API klíče** se nic neodesílá – odesílání je tedy ve výchozím
> stavu „vypnuté", dokud klíč nezadáte.

---

## 1. Získání API klíče

1. Přihlas se / zaregistruj na <https://stations.windy.com>.
2. V sekci **My stations** přidej novou stanici (*Add a station*):
   - vyplň název, polohu (lat/lon), nadmořskou výšku a typ.
3. Po uložení windy vygeneruje pro stanici **API key** (dlouhý řetězec, až 123 znaků)
   a **Station ID**.
4. Zkopíruj si **API key** – budeš ho zadávat do stanice.

---

## 2. Nastavení klíče ve WX stanici

Klíč se zadává přes konfigurační menu (telnet nebo sériová konzole), stejně jako
ostatní parametry (volačka, souřadnice, nadmořská výška…).

1. Připoj se na stanici:
   - **telnet** na IP stanice (port 23), nebo
   - **USB sériová konzole** (115200 Bd).
2. Stiskni `?` pro výpis menu. V seznamu uvidíš novou položku:

   ```
   K  windy.com API key [empty, upload OFF]
   ```

3. Stiskni `K`.
4. Vlož (paste) API klíč z windy a potvrď **Enter** (na sériové konzoli `;`).
5. Stanice odpoví např.:

   ```
   windy API key saved (96 chars)
   ```

   a v menu se položka změní na `K  windy.com API key [SET, upload ON]`.

Klíč se ukládá do EEPROM (adresy 244–367), takže **přežije restart** i výpadek napájení.

### Vypnutí odesílání / smazání klíče
Stiskni `K` a potvrď **prázdný** vstup (hned Enter). Klíč se vymaže a odesílání se vypne:

```
windy upload disabled (key erased)
```

---

## 2b. Veřejné Station ID a odkaz na hlavní stránce

Aby se na hlavní webové stránce stanice (port 80) objevil odkaz **WINDY** vedle odkazu
**APRS**, zadej ještě veřejné **Station ID** stanice.

- **API klíč** (krok 2) = tajný klíč pro upload (~96 znaků).
- **Station ID** = krátký veřejný identifikátor v adrese stanice, např. `pws-f06ea43a`
  (najdeš ho na <https://stations.windy.com> u stanice, resp. ve veřejné URL
  `https://www.windy.com/station/pws-…`).

Nastavení:

1. V menu stiskni `I`.
2. Vlož Station ID (např. `pws-f06ea43a`) a potvrď **Enter** (`;` na sériové konzoli).
3. Stanice odpoví:

   ```
   windy Station ID saved | https://www.windy.com/station/pws-f06ea43a
   ```

Odkaz **WINDY** se na hlavní stránce zobrazí, **jen když je zároveň**:
- nastavený API klíč (probíhá upload), a
- vyplněné Station ID.

Výsledek v hlavičce stránky:

```
… | APRS | WINDY | Upload FW | …
```

Station ID smažeš zadáním prázdné hodnoty u příkazu `I` (odkaz se pak skryje).

---

## 3. Nadmořská výška a souřadnice

Aby data dávala smysl:

- Nastav **nadmořskou výšku** v menu příkazem `m` (slouží i k přepočtu tlaku na hladinu moře).
- Souřadnice stanice se zadávají přímo na webu windy.com u dané stanice.

---

## 4. Jak často se odesílá

Data se posílají ve stejném intervalu jako ostatní výstupy (MQTT, APRS) –
**„TX repeat time"** v menu (příkaz `x`), výchozí **5 minut**.
Windy doporučuje neposílat častěji než cca jednou za 5 minut, takže výchozí
hodnotu není potřeba měnit.

---

## 5. Co se odesílá (a v jakých jednotkách)

Firmware převádí interní metrické hodnoty na imperiální jednotky, které windy očekává:

| Veličina         | windy parametr  | jednotka | převod ve firmwaru        |
|------------------|-----------------|----------|---------------------------|
| Směr větru       | `winddir`       | °        | –                         |
| Rychlost větru   | `windspeedmph`  | mph      | m/s ÷ 0,44704             |
| Náraz větru      | `windgustmph`   | mph      | m/s ÷ 0,44704             |
| Teplota          | `tempf`         | °F       | °C × 1,8 + 32             |
| Srážky (dnes)    | `rainin`        | palce    | mm ÷ 25,4                 |
| Tlak             | `baromin`       | inHg     | hPa × 0,02953             |
| Rosný bod        | `dewptf`        | °F       | °C × 1,8 + 32             |
| Vlhkost          | `humidity`      | %        | –                         |

---

## 6. Ověření / ladění

Zapni ladění v menu příkazem `*` (debug). V konzoli pak uvidíš:

```
[windy] connecting to stations.windy.com ...
[windy] free heap before: 224740
[windy] connected!
[windy] GET /pws/update/XXXX?winddir=...&windspeedmph=... HTTP/1.1
[windy] headers received
[windy RX] ... odpověď serveru ...
[windy] stop
```

- `connection failed!` → zkontroluj internet/DNS, čas (TLS potřebuje platný čas přes NTP),
  a že stanice má konektivitu (Ethernet připojen, MQTT připojeno).
- `no API key set, skip upload` → není nastavený klíč (viz krok 2).
- Data na webu windy.com se objeví u stanice v sekci **My stations** během několika minut.

---

## 7. Poznámky k paměti (důležité)

Původně byla funkce vypnutá kvůli nedostatku RAM. Aktuální implementace ji řeší:

- **TLS klient (`WiFiClientSecure`) je vytvářen lokálně** uvnitř funkce odesílání,
  takže jeho ~30–40 kB bufferů se po každém odeslání **uvolní** (dříve byl globální
  a držel paměť trvale).
- Ve výchozím stavu se používá `client.setInsecure()` – **neověřuje se certifikát**
  serveru, čímž se ušetří paměť. Pro meteodata je to akceptovatelné.
- Volný heap se vypisuje před každým odesláním (`free heap before:`), takže lze sledovat
  trend (publikuje se i přes MQTT topic `FreeHeap`).

### Volitelné: ověřování certifikátu
Pokud chceš plné ověření TLS certifikátu (vyšší nárok na RAM), odkomentuj v `wx.ino`:

```cpp
#define WINDY_VERIFY_CERT
```

Tím se aktivuje ověření proti vestavěnému ISRG Root X1 certifikátu (platný do 2035).
