# HT-CT62 LoRa API - Command Reference Manual

Ez a dokumentum a Heltec HT-CT62 (SX1262) LoRa modulhoz készült `loraAPI` firmware parancskészletét tartalmazza. A parancsok végrehajtása az USB Soros (Serial) porton vagy a hardveres UART (Serial1) interfészen keresztül történik 115200 baud sebesség mellett.

---

## 1. Lekérdező parancsok (Getters)

A modul aktuális belső állapotának és rádiós paramétereinek kiolvasására szolgáló parancsok.

| Parancs | Funkció | Válasz formátuma |
| :--- | :--- | :--- |
| `get_freq` | Aktuális hordozófrekvencia lekérése (MHz) | `OK: get_freq <float>` |
| `get_bw` | Jel sávszélességének lekérése (kHz) | `OK: get_bw <float>` |
| `get_sf` | Spreading Factor (szétterítési tényező) lekérése | `OK: get_sf <uint8_t>` |
| `get_cr` | Coding Rate (kódolási arány) lekérése | `OK: get_cr <uint8_t>` |
| `get_sw` | LoRa Sync Word lekérése hexadecimális formában | `OK: get_sw 0x<HEX>` |
| `get_power` | Adási teljesítmény lekérése (dBm) | `OK: get_power <int8_t>` |
| `get_pl` | Preamble (bevezető szinkronizációs) hossz lekérése | `OK: get_pl <uint16_t>` |
| `get_tcxo` | TCXO (hőmérséklet-kompenzált oszcillátor) feszültség lekérése | `OK: get_tcxo <float>` |
| `get_crc` | CRC ellenőrzés állapotának lekérése (0: kikapcsolva, 2: bekapcsolva) | `OK: get_crc <0\|2>` |
| `get_mode` | Aktuális rádiós üzemmód lekérése (`rx`, `tx` vagy `rxtx`) | `OK: get_mode <string>` |
| `get_RSSI` | Legutóbb fogadott csomag RSSI (vett jel erőssége) értéke (dBm) | `OK: get_RSSI <float>` |
| `get_SNR` | Legutóbb fogadott csomag SNR (jel-zaj viszony) értéke (dB) | `OK: get_SNR <float>` |
| `get_txto` | Szoftveres adási timeout korlát lekérése (milliszekundum) | `OK: get_txto <uint32_t>` |
| `get_config` | A teljes fizikai réteg konfigurációjának lekérése ömlesztve | `OK: freq:...;bw:...;sf:...;cr:...;sync:...;power:...;prelen:...;tcxo:...;crc:...` |
| `read` | Az utoljára sikeresen pufferelt LoRa csomag hasznos tartalma | `OK: read <string>` |

---

## 2. Beállító parancsok (Setters)

Ezen parancsok futtatásakor a szoftver automatikusan újraindítja a rádiós alrendszert (csendes re-init a háttérben) az új paraméter érvényesítéséhez.

### `set_freq <frekvencia_mhz>`
Beállítja a hordozófrekvenciát.
*   **Példa:** `set_freq 868.5`
*   **Sikeres válasz:** `OK: set_freq 868.5`

### `set_bw <savszelesseg_khz>`
Beállítja a LoRa csatorna sávszélességét. Érvényes értékek a RadioLib specifikáció szerint (pl. 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0).
*   **Példa:** `set_bw 125.0`
*   **Sikeres válasz:** `OK: set_bw 125.0`

### `set_sf <spreading_factor>`
Beállítja a modifikációs faktort. Érvényes tartomány: `5` - `12`.
*   **Példa:** `set_sf 7`
*   **Sikeres válasz:** `OK: set_sf 7`

### `set_cr <coding_rate>`
Beállítja a hibaáthajló kódolási arányt. Érvényes értékek: `5` (4/5), `6` (4/6), `7` (4/7), `8` (4/8).
*   **Példa:** `set_cr 5`
*   **Sikeres válasz:** `OK: set_cr 5`

### `set_sw <sync_word>`
Beállítja a LoRa hálózati szinkronszót. Elfogad decimális vagy hexadecimális (0x előtaggal) formátumot. A privát hálózatok alapértelmezett értéke `0x12`.
*   **Példa:** `set_sw 0x12`
*   **Sikeres válasz:** `OK: set_sw 0x12`

### `set_power <teljesitmeny_dbm>`
Beállítja a kimenő adási teljesítményt dBm-ben. SX1262 esetén a megengedett tartomány `-9` és `+22` dBm között van.
*   **Példa:** `set_power 14`
*   **Sikeres válasz:** `OK: set_power 14`

### `set_pl <preamble_hossz>`
Beállítja az adás elejére fűzött preamble szimbólumok számát.
*   **Példa:** `set_pl 8`
*   **Sikeres válasz:** `OK: set_pl 8`

### `set_tcxo <feszultseg>`
Beállítja az integrált TCXO oszcillátor tápfeszültségét voltban. Ha a hardver nem igényel TCXO-t, értéke `0`. HT-CT62 esetén az alapértelmezett érték `1.6`.
*   **Példa:** `set_tcxo 1.6`
*   **Sikeres válasz:** `OK: set_tcxo 1.6`

### `set_crc <0|2>`
Engedélyezi vagy tiltja a hardveres csomag-szintű ciklikus redundancia-ellenőrzést.
*   `0`: CRC kikapcsolva
*   `2`: 2-byte-os LoRa CRC engedélyezve (alapértelmezett)
*   **Példa:** `set_crc 2`
*   **Sikeres válasz:** `OK: set_crc 2`

### `set_txto <idokorlat_ms>`
Beállítja a szoftveres watchdog időkorlátot az adási fázisra. Ha az adás megszakad vagy beragad, a megadott idő lejárta után a modul hardveresen újraindítja a rádiót.
*   **Példa:** `set_txto 5000`
*   **Sikeres válasz:** `OK: set_txto 5000`

### `set_config <freq,bw,sf,cr,sw,power,pl,tcxo[,crc]>`
Lehetővé teszi az összes kritikus rádiós paraméter egyidejű, egyetlen sorban történő módosítását. A paramétereket szigorúan vesszővel kell elválasztani, szóközök nélkül.
*   **Példa (9 paraméter):** `set_config 868.0,125.0,7,5,0x12,14,8,1.6,2`
*   **Sikeres válasz:** `OK: set_config 868.0,125.0,7,5,0x12,14,8,1.6,2`

### `set_mode <rx|tx|rxtx>`
Rádió működési üzemmódjának direkt kényszerítése.
*   `rx`: Csak vétel. Adási parancsok elutasításra kerülnek.
*   `tx`: Csak adás. A modul készenléti (standby) állapotban marad, nem figyel a bejövő csomagokra.
*   `rxtx`: Kombinált automata üzemmód. Alapértelmezetten folyamatos vételen van, de adási parancs (`send`) indításakor átvált, kiküldi a csomagot, majd azonnal visszatér vételi üzemmódba.
*   **Példa:** `set_mode rxtx`
*   **Sikeres válasz:** `OK: set_mode rxtx`

---

## 3. Akció parancsok (Actions)

### `send "<hasznos_teher>"`
Kiküldi a megadott karakterláncot a levegőbe. A kiküldés előtt a firmware automatikusan végrehajt egy CAD (Channel Activity Detection / LBT - Listen Before Talk) csatorna-ellenőrzést. Ha a frekvencia foglalt, az adás megszakad.
*   *Megjegyzés:* Ha a küldendő szöveg nem tartalmaz szóközt, az idézőjelek elhagyhatók.
*   **Példa:** `send "S.E.A.T._CanSat_2026"`
*   **Sikeres indítás válasza:** `OK: send "S.E.A.T._CanSat_2026"`
*   **Hiba (foglalt csatorna):** `ERR: CHANNEL_BUSY_CAD`
*   **Hiba (beragadt hardver):** `ERR: SEND_FAIL_<kód>`

### `demo <intervallum_ms> [ismetlesek_szama]`
Elindítja az automatizált periodikus tesztcsomag-küldést. A kiküldött csomagok tartalma egy fixen 100 karakter hosszú, véletlenszerűen generált string (A-Z karakterek).
*   Ha az `ismetlesek_szama` paraméter nincs megadva vagy értéke `0`, a ciklus a végtelenségig fut, amíg le nem állítják.
*   **Példa (folyamatos 1 másodperces adás):** `demo 1000`
*   **Példa (5 darab csomag küldése 2 másodpercenként):** `demo 2000 5`
*   **Sikeres indítás válasza:** `OK: demo <argumentumok>`

### `demo off`
Azonnal leállítja az aktív periodikus automata tesztcsomag-küldést, a rádiót visszateszi a konfigurált alapmódba.
*   **Példa:** `demo off`
*   **Sikeres válasz:** `OK: demo off`

---

## 4. Aszinkron eseményértesítések (INFO logok)

A modul működés közben a végrehajtott akciókról és a bejövő rádióforgalomról aszinkron `INFO:` prefixszel ellátott naplósorokat küld ki az UART/USB interfészekre. Ezeket a fogadó MCU (XIAO) automatikusan feldolgozza.

*   `INFO: TX_START | Len: <hossz>` -> Az adási fázis elindult, a megadott byte-számmal.
*   `INFO: TX_DONE | Airtime: <idő>ms` -> Az adás sikeresen befejeződött, a kisugárzás ennyi ideig tartott a levegőben.
*   `INFO: TIMEOUT | TX failed. Resetting.` -> Az adás nem fejeződött be a szoftveres limiten belül, a rádió újraindult.
*   `INFO: RX_DATA | RSSI: <érték> | SNR: <érték> | <adat>` -> Sikeres LoRa csomag érkezett a megadott térerő adatokkal és tartalommal.
*   `INFO: RX_CMD | <parancs>` -> Olyan LoRa csomag érkezett, amely `CMD:` előtaggal rendelkezett. Ez a távoli parancsvégrehajtó csatorna.
*   `INFO: DEMO_STARTED` / `INFO: DEMO_STOPPED` -> Az automata tesztüzemmód állapota megváltozott.

---

## 5. Hibakezelési válaszok

Amennyiben a beadott parancs szintaktikailag hibás, vagy a fizikai réteg hibára fut, a következő sorok jelennek meg:
*   `INVALID: <beirt_parancs>` -> Ismeretlen parancsnév vagy rossz argumentum-struktúra.
*   `ERR: init_radio_fail_<kod>` -> Kritikus indítási hiba a modul bekapcsolásakor (hardver hiba vagy hibás SPI vonal).
