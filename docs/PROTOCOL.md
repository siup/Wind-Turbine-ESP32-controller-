# Protokół komunikacji, wersja 1

Jeden format JSON dla WebSocket (`ws://192.168.4.1/ws`), HTTP (`POST /api/cmd`, `GET /api/state`) i BLE. Każda ramka to jeden obiekt JSON z polem `t` (typ). Wersja protokołu jest w polu `v` ramek `state` i `info`. Ten plik opisuje stan faktyczny firmware; jeśli coś się tu nie zgadza z kodem, poprawiamy jedno albo drugie, nie zostawiamy rozjazdu.

Zasady ogólne:

- maksymalna długość ramki od klienta: 256 bajtów (dłuższa dostaje `err too_long`),
- nieznane pola są ignorowane, zły typ pola to `err bad_type`, wartość spoza zakresu to `err out_of_range`,
- każda komenda może mieć opcjonalne `"id"` (liczba całkowita 0..4294967295). ESP odbija je w `ack`, `err` i `pong`, więc klient może mierzyć opóźnienie i parować odpowiedzi,
- komendy od wielu klientów są wykonywane w kolejności przyjścia, bez arbitrażu. Wyjątek: `stop` wskakuje na początek kolejki.

## Komendy klient → ESP

### rotor

```json
{"t":"rotor","speed":55}
{"t":"rotor","speed":0,"brake":true}
```

- `speed` 0..100 (liczba, ułamki są zaokrąglane). Wartość powyżej ustawienia `rotorMax` jest przycinana do `rotorMax`, `state.rotor.target` pokazuje wartość po przycięciu.
- `speed > 0` uruchamia rotor z rampą `rotorRamp`. Wymaga zgody failsafe (`low_battery`, `busy_ota`). Nadawca staje się właścicielem ruchu (patrz Failsafe). Kasuje powód failsafe `runtime`.
- `speed 0` to zatrzymanie z rampą, `speed 0` + `"brake":true` to natychmiastowe hamowanie (oba wejścia mostka HIGH).

### yaw

Dokładnie jedno z pól `dir`, `hold`, `goto`, `zero`, `cal`:

```json
{"t":"yaw","dir":"left"}
{"t":"yaw","dir":"right","speed":60}
{"t":"yaw","dir":"stop"}
{"t":"yaw","hold":"left"}
{"t":"yaw","goto":135}
{"t":"yaw","zero":true}
{"t":"yaw","cal":"start"}
```

- `dir`: `left` | `right` uruchamia ruch na przytrzymanie, `stop` zatrzymuje z hamowaniem. Opcjonalne `speed` 20..100, domyślnie ustawienie `yawSpeed`. Blokady: `err endstop` (krańcówka w tę stronę), `err twist_limit` (limit skrętu przewodów), `low_battery`, `busy_ota`.
- `hold`: podtrzymanie (deadman). Klient wysyła je co 200 ms, dopóki użytkownik trzyma przycisk. Przedłuża tylko ruch, który już trwa w tym samym kierunku. Po `stop`, po failsafe, po zatrzymaniu przez limit albo przy innym kierunku jest ignorowane, więc spóźnione `hold` nigdy nie uruchomi silnika ponownie. Bez `hold` przez `max(700 ms, fsTimeout)` yaw staje sam.
- `goto`: kąt docelowy 0..359.9, tylko z czujnikiem AS5600 (inaczej `err no_sensor`). Jedzie krótszą drogą, chyba że łamie limit skrętu `yawMaxDeg`; wtedy dłuższą, a jeśli obie łamią, `err twist_limit`. Zwalnia do 30% w ostatnich 10°, kończy w oknie ±2°, timeout 10 s.
- `zero`: bieżąca pozycja staje się zerem (saldo czasu albo offset AS5600). Tylko gdy yaw stoi (`err busy`).
- `cal`: pomiar czasu pełnego obrotu w trybie bez czujnika: `start`, użytkownik obraca gondolę dokładnie raz, `stop` zapisuje `yawMs360` (min. 500), `cancel` przerywa. W trybie AS5600 `err no_sensor`.

### stop

```json
{"t":"stop"}
```

Wszystko stop natychmiast, bez rampy: rotor wybieg (oba wejścia LOW), yaw hamowanie, `goto` anulowane. Nie zatrzaskuje failsafe: następna komenda działa normalnie.

### ping

```json
{"t":"ping","id":17}
```

Odpowiedź `{"t":"pong","id":17,"up":12345}`. Klient wysyła `ping` co 300 ms, gdy przez ten czas nic innego nie wysłał.

### cfg

```json
{"t":"cfg","get":true}
{"t":"cfg","yawSpeed":70,"rotorMax":80}
{"t":"cfg","reset":true}
```

- `get` (albo samo `{"t":"cfg"}`) zwraca ramkę `cfg`.
- Zapis: dowolny podzbiór kluczy z tabeli niżej. Patch jest atomowy: albo przechodzi cały, albo nic się nie zmienia i wraca `err out_of_range` / `err bad_type` z polem `field`. Jeśli w patchu jest `batType` 0 albo 1, najpierw wpisywany jest preset progów baterii, potem reszta pól. Pola oznaczone "postój" wymagają stojących silników (`err busy`). Po zapisie: `ack` do nadawcy, a nowa ramka `cfg` leci do wszystkich klientów WS. Zapis do NVS jest odroczony o 300 ms (suwaki nie mielą flasha).
- `reset`: ustawienia fabryczne (nie rusza zapamiętanej pozycji yaw).

| Klucz | Typ | Zakres | Domyślnie | Uwagi |
| --- | --- | --- | --- | --- |
| `rotorMax` | int % | 10..100 | 100 | sufit prędkości rotora |
| `rotorMin` | int % | 0..60 | 0 | kompensacja martwej strefy silnika |
| `rotorRamp` | int ms | 50..10000 | 500 | czas rampy 0→100% |
| `rotorDuty` | int | 50..255 | 153 | sufit PWM (ochrona silnika 3 V na 5 V) |
| `rotorRev` | bool | | false | postój |
| `rotorMaxRun` | int s | 0..3600 | 0 | 0 = bez limitu |
| `yawSpeed` | int % | 20..100 | 60 | |
| `yawDuty` | int | 50..255 | 255 | |
| `yawInvert` | bool | | false | postój, zeruje pozycję |
| `yawMaxBal` | int ms | 0..600000 | 0 | limit skrętu bez czujnika, 0 = wyłączony |
| `yawMs360` | int ms | 0..600000 | 0 | czas pełnego obrotu, 0 = nieznany |
| `yawMaxDeg` | int ° | 0..3600 | 0 | limit skrętu z AS5600 |
| `yawSensRev` | bool | | false | postój, AS5600 liczy w drugą stronę |
| `batType` | int | 0..2 | 0 | 0 = 9V, 1 = 2S Li-ion, 2 = własne progi |
| `batCal` | float | 0.5..1.5 | 1.0 | mnożnik kalibracji |
| `batLow` | int mV | 3000..20000 | 7000 | ostrzeżenie |
| `batCrit` | int mV | 3000..20000 | 6200 | stop i blokada, musi być < `batLow` |
| `batEmpty` | int mV | 3000..20000 | 6200 | 0% |
| `batFull` | int mV | 3000..20000 | 9400 | 100%, musi być > `batEmpty` |
| `fsTimeout` | int ms | 400..1000 | 500 | cisza właściciela ruchu przed stopem |
| `name` | string | 0..24 znaki | "" | nazwa sieci i BLE, "" = `Turbina-XXXX`, restart |
| `apPass` | string | 8..63 znaki | `turbina123` | tylko zapis, nigdy nie wraca w `cfg`, restart |
| `apChan` | int | 1..13 | 1 | restart |
| `txPower` | int | 8..78 | 34 | moc WiFi w 0,25 dBm (34 = 8,5 dBm), na żywo |
| `bleMode` | int | 0..2 | 1 | 0 wył., 1 auto (tylko gdy nikt nie jest na WiFi), 2 zawsze |
| `captive` | bool | | false | captive portal, restart |
| `led` | bool | | true | dioda statusu |

Stringi: drukowalne ASCII bez `"` i `\`.

### info, log, reboot

```json
{"t":"info"}
{"t":"log"}
{"t":"reboot"}
```

`reboot`: `ack`, stop silników, zapis stanu, restart po 300 ms.

## Ramki ESP → klient

```json
{"t":"ack","of":"rotor","id":5}
{"t":"err","of":"yaw","code":"no_sensor","msg":"goto requires AS5600","id":6}
{"t":"err","of":"cfg","code":"out_of_range","field":"rotorMax","msg":"value out of range"}
{"t":"pong","id":7,"up":12345}
```

`of` to typ komendy, której dotyczy odpowiedź (`unknown`, gdy nie dało się jej ustalić). `ack` idzie dla każdej poprawnie wykonanej komendy poza `ping` (po BLE tylko gdy komenda miała `id`).

### state (telemetria)

```json
{"t":"state","v":1,
 "rotor":{"target":55,"actual":48,"run":12},
 "yaw":{"moving":"left","speed":60,"angle":null,"limitL":false,"limitR":false,"twist":null,
        "bal":-1520,"balMax":20000,"pos":-27,"est":true,"posMax":360,
        "goto":null,"sensor":"none","sensorOk":true,"cal":false},
 "bat":{"mv":7840,"pct":62,"low":false,"crit":false,"present":true},
 "link":{"rssi":-58,"clients":1,"ws":1,"ble":0,"sta":1},
 "fs":{"active":false,"reason":null},
 "ota":false,"cfgRev":3,"up":12345}
```

- `rotor.run`: sekundy ciągłej pracy rotora.
- `yaw.moving`: `left` | `right` | `null`.
- `yaw.angle`: kąt 0..359 z AS5600 albo `null` (brak czujnika, brak magnesu). Aplikacja pokazuje kompas tylko gdy to pole jest liczbą albo gdy jest estymacja (`pos` z `est:true`).
- `yaw.limitL` / `limitR`: ruch w tę stronę zablokowany (krańcówka albo limit skrętu). Przycisk wyszarzony.
- `yaw.twist`: `"L"` | `"R"` | `null`: to limit skrętu przewodów, aplikacja pokazuje "limit skrętu, wróć w drugą stronę".
- `yaw.bal` / `balMax`: saldo skrętu w ms ważonych prędkością (ms przy 100%), + = prawo. `balMax` 0 = limit wyłączony.
- `yaw.pos`: pozycja ciągła w stopniach (może wyjść poza ±360) albo `null`; `est:true` = szacunek z czasu ("około"), `false` = pomiar AS5600. `posMax`: limit skrętu w stopniach albo `null`.
- `yaw.goto`: kąt docelowy trwającego `goto` albo `null`. `cal`: trwa pomiar czasu obrotu.
- `bat.present`: false gdy odczyt < 2,5 V (zasilanie z USB, odpięty dzielnik). Nie zdejmuje zatrzaśniętego `crit`.
- `link.rssi`: najsłabsza stacja WiFi albo połączenie BLE, `null` gdy nikogo. `clients` = `ws` + `ble`.
- `fs.reason`: `timeout` | `disconnect` | `battery` | `ota` | `runtime` | `null`.
- `cfgRev`: rośnie przy każdej zmianie ustawień (klient HTTP po zmianie pobiera `cfg`).
- `up`: ms od startu.

Wysyłka: co 200 ms do wszystkich klientów WS i dodatkowo natychmiast po zmianie stanu, nie częściej niż co 50 ms. Zmiana `up`, `rssi`, `bat.mv` i `rotor.run` nie wyzwala wysyłki natychmiastowej.

### state po BLE (skrócona, max 180 bajtów)

```json
{"t":"state","v":1,"rotor":{"target":55,"actual":48},"yaw":{"moving":"left","angle":null},"bat":{"pct":62,"crit":false},"fs":{"active":false,"reason":null}}
```

Co 200 ms i po zmianie, nie częściej niż co 100 ms.

### cfg

```json
{"t":"cfg","rev":3,"reboot":false,"apPassSet":true,"rotorMax":100,"rotorMin":0, "...": "wszystkie klucze z tabeli poza apPass"}
```

`reboot:true` znaczy, że zmieniono pole, które zadziała dopiero po restarcie.

### info

```json
{"t":"info","v":1,"fw":"1.0.0","build":"Nov  9 2026 12:00:00","name":"Turbina-AB12","chip":"ESP32",
 "mac":"24:6F:28:AA:AB:12","sensor":"none","wifi":true,"ble":true,"ota":true,
 "heap":151234,"heapMin":120112,"reset":"poweron","up":12345}
```

### log, evt

```json
{"t":"log","items":[{"ms":5021,"lvl":"w","code":"fs_timeout","msg":"owner ws#3 silent 512 ms"}]}
{"t":"evt","ms":5021,"lvl":"w","code":"fs_timeout","msg":"owner ws#3 silent 512 ms"}
```

`log` to ostatnie 20 zdarzeń (najstarsze pierwsze), `evt` to pojedyncze zdarzenie wysyłane na żywo do klientów WS.

### Po połączeniu WS

ESP od razu wysyła `info`, `cfg` i `state`.

## Kody błędów

| Kod | Znaczenie |
| --- | --- |
| `bad_json` | ramka nie jest obiektem JSON |
| `bad_type` | brak wymaganego pola albo zły typ |
| `out_of_range` | wartość poza zakresem albo spoza listy |
| `unknown_cmd` | nieznane `t` |
| `too_long` | ramka dłuższa niż 256 bajtów |
| `no_sensor` | komenda wymaga czujnika, którego nie ma |
| `failsafe` | zarezerwowane |
| `low_battery` | bateria krytyczna, start zablokowany |
| `busy_ota` | trwa aktualizacja |
| `busy` | wymaga postoju silników albo kolejka pełna |
| `twist_limit` | limit skrętu przewodów w tę stronę |
| `endstop` | krańcówka w tę stronę |

## Failsafe

Właściciel ruchu to klient, którego komenda ruchu (rotor > 0, yaw `dir`, `goto`) została przyjęta jako ostatnia. Właściciel jest zwalniany, gdy wszystko stoi.

| Zdarzenie | Reakcja | Powrót |
| --- | --- | --- |
| Właściciel ruchu milczy dłużej niż `fsTimeout` (500 ms) | rotor rampa 200 ms do 0, yaw stop, `goto` anulowane, `reason=timeout` | pierwsza poprawna ramka od dowolnego klienta |
| Właściciel się rozłączył albo liczba klientów WS+BLE spadła do 0 | jak wyżej, `reason=disconnect` | pierwsza poprawna ramka po połączeniu |
| Bateria krytyczna | stop wszystkiego natychmiast, blokada startu, `reason=battery` | napięcie wróci do `batCrit + 0,3 V` |
| Krańcówka yaw | stop ruchu w tę stronę | ruch w drugą stronę od razu |
| Komenda `stop` | stop natychmiast, rotor wybieg, yaw hamowanie | następna komenda (bez zatrzasku) |
| OTA w toku | stop, blokada komend (`busy_ota`), `reason=ota` | restart |
| Rotor pracuje dłużej niż `rotorMaxRun` | rampa do 0, ostrzeżenie, `reason=runtime` | następna komenda startu rotora |
| Watchdog 5 s | reset, piny silników LOW jako pierwsza rzecz w `setup()` | restart |

Każde zadziałanie zeruje nastawy: po powrocie nic nie rusza samo, użytkownik musi wcisnąć START jeszcze raz. Klient nie odtwarza starych nastaw po ponownym połączeniu, rysuje kontrolki wyłącznie z telemetrii.

Klient blokuje przyciski ruchu tylko przy braku połączenia, `bat.crit` i `ota`. Przy `timeout` / `disconnect` nie blokuje, bo jego własna kolejna ramka i tak kasuje failsafe.

## HTTP

- `GET /api/state`: ostatnia ramka `state`.
- `POST /api/cmd`: treść to jedna ramka. Odpowiedź od razu: `err` przy błędzie parsowania, `pong` na `ping`, `cfg` / `info` / `log` z pamięci podręcznej, w pozostałych przypadkach `ack`. Błędy wykonania (np. `low_battery`) klient HTTP widzi w telemetrii, nie w odpowiedzi.
- `POST /update?type=fw|fs`: aktualizacja OTA (multipart, Basic Auth `admin` + hasło AP). 409 gdy silniki pracują.
- Klient HTTP jest identyfikowany po adresie IP (właściciel ruchu, timeout).

## BLE

- Serwis `a8d40001-6c7e-4f2b-9a3e-1b2c3d4e5f60`
- `cmd` `a8d40002-…` (write, write without response): jedna ramka JSON na zapis.
- `state` `a8d40003-…` (read, notify): skrócone `state`, `err`, `pong`, `ack` tylko dla komend z `id`.
- Nazwa w scan response. MTU preferowane 247; ramka dłuższa niż MTU-3 nie jest wysyłana do tego klienta.
- Domyślnie (`bleMode` 1) reklamuje się tylko wtedy, gdy nikt nie jest podłączony do WiFi.
- Web Bluetooth w Chrome działa tylko na stronach HTTPS, więc panel otwarty z `http://192.168.4.1` nie widzi BLE. Patrz README.
