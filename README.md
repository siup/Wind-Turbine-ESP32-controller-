# Turbina: sterownik modelu turbiny wiatrowej na ESP32

Firmware na ESP32 i panel w przeglądarce telefonu do sterowania modelem turbiny. Rotor kręci się z regulowaną prędkością i miękkim startem, gondola (yaw) obraca się w lewo i w prawo, dopóki trzymasz przycisk. ESP wystawia własną sieć WiFi i serwuje panel z pamięci, więc na telefonie nic nie trzeba instalować. Jest też BLE dla Chrome na Androidzie.

Kod i komentarze są po angielsku, interfejs po polsku. Specyfikacja, z której to wyrosło, leży osobno; tu jest opis tego, co faktycznie zrobiliśmy, jak to wgrać i jak sprawdzić na stole.

## Sprzęt

- ESP32 DOIT DevKit V1 (ESP32-WROOM-32D, 30 pinów, CH340C) do testów, docelowo ESP32-S3,
- moduł DRV8833 (dwa mostki H),
- silnik rotora: DC 3 V albo N20 z małą przekładnią,
- silnik gondoli: N20 z przekładnią 50:1 do 210:1,
- bateria 9 V na początek, docelowo 2x 18650, przetwornica MP1584 ustawiona na 5 V,
- dzielnik napięcia 100k / 33k i kondensator 100 nF,
- opcjonalnie dwie krańcówki albo enkoder magnetyczny AS5600 z magnesem.

Wszystkie numery pinów są w jednym miejscu: `include/config.h`. Pełna tabela dla ESP32 i S3, z pinami, których lepiej nie ruszać, jest w [docs/PINOUT.md](docs/PINOUT.md).

| Funkcja | ESP32 DevKit V1 | Dokąd |
| --- | --- | --- |
| Rotor IN1 / IN2 | GPIO 25 / 26 | DRV8833 AIN1 / AIN2, wyjścia AOUT1/AOUT2 do silnika rotora |
| Yaw IN1 / IN2 | GPIO 27 / 14 | DRV8833 BIN1 / BIN2, wyjścia BOUT1/BOUT2 do silnika gondoli |
| DRV8833 SLEEP | GPIO 33 | EEP (nSLEEP) na module |
| Pomiar baterii | GPIO 34 | środek dzielnika 100k / 33k, 100 nF do GND |
| Krańcówka L / R (opcja) | GPIO 16 / 17 | styk do GND, pull-up jest w ESP |
| AS5600 SDA / SCL (opcja) | GPIO 21 / 22 | zasilanie AS5600 z 3,3 V |
| Dioda statusu | GPIO 2 | wbudowana niebieska |

Zasilanie: bateria idzie na wejście MP1584, a 5 V z przetwornicy zasila ESP (pin VIN) i silniki (VCC/VM modułu DRV8833). Masy wszystkiego łączysz razem. Silniki dostają 5 V, dlatego firmware ogranicza wypełnienie PWM osobno dla każdego silnika (`rotorDuty`, domyślnie 153 z 255, czyli około 60 % dla gołego silnika 3 V).

```
 bateria (+) ----+---------------------------> MP1584 IN+
                 |                             MP1584 OUT 5V ---+---> ESP32 VIN
               [100k]                                           +---> DRV8833 VCC (VM)
                 |
                 +-----------> GPIO34 ---[100nF]--- GND
                 |
               [33k]
                 |
 bateria (-) ----+---- GND MP1584 ---- GND ESP32 ---- GND DRV8833

 ESP32                  DRV8833
 GPIO25 --------------> AIN1 \
 GPIO26 --------------> AIN2 /  AOUT1, AOUT2 ----> silnik rotora
 GPIO27 --------------> BIN1 \
 GPIO14 --------------> BIN2 /  BOUT1, BOUT2 ----> silnik gondoli
 GPIO33 --------------> EEP (nSLEEP)

 opcja: GPIO16 --[krańcówka L]-- GND     GPIO17 --[krańcówka R]-- GND
 opcja: AS5600 VCC 3V3, GND, SDA GPIO21, SCL GPIO22, DIR na stałe do GND
```

Jedna uwaga zanim podłączysz silniki: GPIO 14 przy starcie ESP przez chwilę macha stanem. Nic się nie dzieje, bo SLEEP jest wtedy LOW, ale wiele klonów DRV8833 ma rezystor podciągający EEP do VCC. Sprawdź to na swoim module, szczegóły w [docs/PINOUT.md](docs/PINOUT.md).

## Budowanie i wgrywanie

Potrzebny jest [PlatformIO](https://platformio.org/) (wtyczka do VS Code albo `pip install platformio`). Arduino core 3.x przychodzi z platformy pioarduino, wpisanej w `platformio.ini`, więc pierwsza kompilacja chwilę ściąga narzędzia.

Firmware i panel wgrywa się osobno. Panel (katalog `data/`) ląduje w systemie plików LittleFS:

```
pio run -e esp32dev -t upload      # firmware
pio run -e esp32dev -t uploadfs    # panel WWW
pio device monitor                 # konsola, 115200
```

Przed każdym buildem skrypt `tools/build_web.py` pakuje `data/` gzipem do `.pio/webfs` i pilnuje, żeby `index.html` nie przekroczył 60 kB. Jeśli wgrasz sam firmware bez panelu, ESP pokaże pod 192.168.4.1 prostą stronę z formularzem do wgrania `littlefs.bin`.

Środowiska:

| Env | Do czego |
| --- | --- |
| `esp32dev` | ESP32 DevKit V1, pełna wersja |
| `esp32-s3-devkitc-1` | ESP32-S3, `Serial` po natywnym USB |
| `esp32dev_bench` | etapy 1 i 2: bez WiFi i BLE, sterowanie tylko z konsoli. Najpierw sprawdzasz, czy zasilanie wytrzymuje silniki, dopiero potem dochodzi radio |
| `native` | testy jednostkowe logiki na komputerze |

## Pierwsze uruchomienie

1. Wgraj firmware i panel jak wyżej.
2. Na telefonie połącz się z siecią `Turbina-XXXX` (XXXX to końcówka adresu MAC, widać ją też w konsoli przy starcie). Hasło: `turbina123`.
3. Otwórz `http://192.168.4.1`.

Android potrafi po chwili zapytać, czy zostać w sieci bez internetu. Odpowiedz, że tak, inaczej telefon ucieknie na dane komórkowe. Hasło sieci zmienisz w panelu (Konfiguracja, Sieć) i warto to zrobić od razu. Panel można dodać do ekranu głównego, wtedy otwiera się na pełnym ekranie.

## Panel

Panel to jeden plik `index.html` bez zewnętrznych zasobów, ciemny, w stylu panelu operatorskiego. Od góry:

- **Pasek statusu.** Rodzaj połączenia (WS, HTTP albo BLE), siła sygnału, opóźnienie w ms mierzone na pingach i bateria w procentach z kolorem.
- **Banery.** Failsafe z powodem, niska bateria, brak połączenia z odliczaniem do kolejnej próby, limit skrętu przewodów, błąd czujnika.
- **Rotor.** Wskaźnik prędkości rzeczywistej i zadanej (widać rampę), START, STOP, HAMUJ, suwak nastawy z przyciskami -5 / +5 i czas pracy.
- **Gondola.** LEWO i PRAWO działają tylko na przytrzymanie. Kompas pojawia się, gdy jest czujnik AS5600 albo szacunek z czasu obrotu (wtedy kąt ma dopisek, że to przybliżenie). Pod spodem wskaźnik skrętu przewodów, suwak prędkości i pole "jedź do kąta" przy AS5600.
- **STOP WSZYSTKO.** Czerwony przycisk przyklejony do dołu ekranu, zawsze widoczny.
- **Konfiguracja.** Kalibracja czasu obrotu, zerowanie pozycji, bateria (typ, progi, kalibracja multimetrem), sieć, bezpieczeństwo, aktualizacja OTA, restart, reset fabryczny, log zdarzeń i ustawienia samego panelu (wibracje, pola zaawansowane, blokada wygaszania ekranu).

Panel łączy się przez WebSocket. Jeśli ten nie ruszy w 3 s, przechodzi na zapytania HTTP co 500 ms. Po zerwaniu próbuje ponownie co 2 s, z wydłużaniem do 10 s. Przyciski ruchu są nieaktywne bez połączenia, przy krytycznej baterii i w trakcie aktualizacji.

## Konfiguracja

Wszystkie ustawienia zmieniasz z panelu, zapisują się w pamięci NVS i przeżywają restart. Pełna lista z zakresami i wartościami domyślnymi jest w tabeli w [docs/PROTOCOL.md](docs/PROTOCOL.md), sekcja `cfg`. Najczęściej przydają się:

- `rotorMax`, `rotorRamp`, `rotorDuty`: sufit prędkości, czas rampy i sufit PWM rotora,
- `rotorMin`: kompensacja martwej strefy, gdy silnik rusza dopiero przy kilkunastu procentach,
- `yawSpeed`, `yawMaxBal`, `yawMs360`: prędkość gondoli, limit skrętu przewodów i czas pełnego obrotu,
- `batType`: preset progów baterii (9 V albo 2S Li-ion) albo własne progi,
- `fsTimeout`: po ilu ms ciszy od telefonu silniki stają (400 do 1000, domyślnie 500).

Zmiany nazwy sieci, hasła, kanału i captive portalu działają dopiero po restarcie, panel to pokazuje. Wartości domyślne i stałe sprzętowe (piny, częstotliwość PWM, progi) są w `include/config.h`.

## Czujnik pozycji gondoli

Tryb wybierasz jedną linijką w `include/config.h`:

```c
#define YAW_SENSOR_MODE YAW_SENSOR_NONE   // albo YAW_SENSOR_ENDSTOPS, YAW_SENSOR_AS5600
```

Panel niczego nie trzeba zmieniać, sam reaguje na pola w telemetrii.

**Bez czujnika** (domyślnie) firmware liczy saldo ruchu w lewo i w prawo, ważone prędkością. Po ustawieniu `yawMaxBal` nie pozwoli skręcić przewodów dalej niż ten próg. Saldo zapisuje się w pamięci, więc przeżywa reset. Najwygodniej zacząć od kalibracji: Konfiguracja, Kalibracja obrotu, START POMIARU, obróć gondolę dokładnie raz, ZAKOŃCZ. Od tej chwili panel pokazuje szacowany kąt, a limit ustawisz w stopniach z zapasem około 20 %. To szacunek, nie pomiar, więc co jakiś czas ustaw gondolę prosto i naciśnij ZERUJ POZYCJĘ.

**Krańcówki**: dwa styki zwierane do masy na GPIO 16 i 17. Dotknięcie krańcówki od razu zatrzymuje ruch w tę stronę, w drugą wolno jechać. Przy okazji krańcówka koryguje saldo czasu, więc szacowany kąt sam się kalibruje.

**AS5600**: prawdziwy kąt co 50 ms, limit skrętu w stopniach (`yawMaxDeg`) i "jedź do kąta" (krótszą drogą, zwalnia do 30 % na ostatnich 10 stopniach, kończy w oknie ±2 stopnie, najwyżej 10 s). Przy braku magnesu firmware zgłasza błąd i wraca do zachowania jak bez czujnika. Jeśli kąt rośnie w złą stronę, włącz `yawSensRev`.

## Sterowanie przez Bluetooth

Web Bluetooth w Chrome działa tylko na stronach HTTPS, a ESP serwuje panel po zwykłym HTTP. Dlatego są dwie drogi:

1. **Kopia panelu na GitHub Pages** (polecane). W ustawieniach repozytorium: Settings, Pages, Source: GitHub Actions. Workflow `.github/workflows/pages.yml` publikuje katalog `data/` przy każdym pushu na `main`. Panel będzie pod adresem w rodzaju `https://<login>.github.io/<repozytorium>/`. Otwórz go raz z internetem, potem zostaje w pamięci telefonu i działa też bez sieci. Z HTTPS panel od razu proponuje "Połącz BLE".
2. **Flaga w Chrome.** Wpisz `chrome://flags/#unsafely-treat-insecure-origin-as-secure`, dodaj `http://192.168.4.1`, uruchom Chrome ponownie. Wtedy przycisk BLE pojawi się też w panelu z ESP.

Domyślnie (`bleMode` 1) ESP ogłasza się po BLE tylko wtedy, gdy nikt nie jest podłączony do jego WiFi, bo oba radia naraz zjadają baterię i psują opóźnienia. Przed połączeniem BLE odłącz więc telefon od sieci `Turbina-XXXX` albo ustaw `bleMode` na 2. iPhone nie ma Web Bluetooth w żadnej przeglądarce, tam zostaje WiFi.

## Aktualizacja bez kabla (OTA)

W panelu: Konfiguracja, System, Aktualizacja OTA. Wybierasz plik i rodzaj:

- firmware: `.pio/build/esp32dev/firmware.bin`,
- system plików (panel): `.pio/build/esp32dev/littlefs.bin`, powstaje po `pio run -e esp32dev -t buildfs`.

Hasło to hasło sieci WiFi (użytkownik `admin`). Silniki muszą stać, inaczej ESP odmówi. W trakcie wgrywania wszystkie komendy ruchu są blokowane, a po zakończeniu ESP sam się restartuje. Adres `http://192.168.4.1/update` przekierowuje do tej sekcji panelu. Gotowe pliki z każdego builda można też pobrać z zakładki Actions na GitHubie (artefakt `firmware`).

## Konsola szeregowa

Monitor portu na 115200. Konsola przydaje się w etapach 1 i 2 oraz do diagnostyki:

| Komenda | Co robi |
| --- | --- |
| `r50` | rotor na 50 % (0 do 100) |
| `rb` | rotor stop z hamowaniem |
| `yl`, `yr` | gondola w lewo, w prawo |
| `ys` | gondola stop |
| `s` | stop wszystkiego |
| `g135` | jedź do kąta (tylko AS5600) |
| `b` | bateria: mV, mV na pinie ADC, procent, flagi |
| `tel on`, `tel off` | skrócona telemetria co 500 ms |
| `ka on`, `ka off` | podtrzymanie: ping i "trzymanie" gondoli co 200 ms |
| `{"t":"cfg","get":true}` | dowolna ramka JSON protokołu |
| `?` | pomoc |

Konsola przechodzi przez te same zabezpieczenia co telefon. Bez `ka on` rotor uruchomiony z konsoli stanie po pół sekundy, bo nikt nie wysyła pingów, a gondola po 0,7 s. Podtrzymanie wyłącza się samo, gdy podłączy się telefon.

## Failsafe, czyli kiedy silniki staną same

Zabezpieczenia są nadrzędne wobec komend. Po każdym zadziałaniu nastawy są zerowane: gdy połączenie wróci, nic nie rusza samo, trzeba jeszcze raz nacisnąć START.

- Telefon, który ostatnio uruchomił ruch, milczy dłużej niż `fsTimeout` (500 ms): rotor zwalnia do zera w 0,2 s, gondola staje od razu. Pingi z innego telefonu tego nie powstrzymają.
- Ten telefon się rozłączył albo odszedł ostatni klient: to samo, natychmiast.
- Gondola rusza tylko na przytrzymanie. Panel powtarza "trzymam" co 200 ms, a gdy przestanie, gondola staje najpóźniej po 0,7 s, nawet jeśli połączenie dalej żyje.
- Bateria spadła poniżej progu krytycznego: wszystko staje od razu i start jest zablokowany. Blokada puszcza dopiero, gdy napięcie wzrośnie o 0,3 V ponad próg, czyli praktycznie po wymianie baterii. Odczyt poniżej 2,5 V traktujemy jako brak baterii (praca z USB na biurku), ale to nigdy nie zdejmuje blokady.
- Przycisk STOP WSZYSTKO zatrzymuje oba silniki bez rampy. Nie blokuje niczego, następna komenda działa normalnie.
- Rotor pracuje dłużej niż `rotorMaxRun` (jeśli ustawione): zwalnia do zera, panel pokazuje ostrzeżenie.
- W trakcie OTA silniki są zablokowane aż do restartu.
- Pętla programu zawiesi się na 5 s: watchdog resetuje ESP. Piny silników idą w stan LOW jako pierwsza rzecz po starcie, zanim ruszy WiFi.

Każde zadziałanie trafia do logu zdarzeń (panel, sekcja Log zdarzeń) i na konsolę z czasem od startu.

## Odbiór etapów

Checklisty z etapów specyfikacji, z opisem jak sprawdzić każdy punkt na stole. Sprzętu tu nie mieliśmy, więc te punkty są do odhaczenia u Was.

**Etap 1: szkielet i silniki** (`pio run -e esp32dev_bench -t upload`, potem monitor)

- [ ] Oba środowiska się kompilują: `pio run -e esp32dev -e esp32-s3-devkitc-1`.
- [ ] `ka on`, potem `r50`: rotor rusza płynnie, bez szarpnięcia, `r0` zwalnia go z rampą. Żeby rampę było widać w liczbach, wydłuż ją na chwilę (`{"t":"cfg","rotorRamp":3000}`) i włącz `tel on`: `actual` rośnie co pół sekundy aż do 50.
- [ ] `yl` i `yr` obracają gondolę w obie strony, `ys` ją zatrzymuje.
- [ ] `s` zatrzymuje wszystko od razu.
- [ ] Kilka razy `r100` i `r0` na baterii 9 V. ESP nie może się zresetować. Po resecie w logu startu widać `reset brownout` i wtedy decyzja o 18650 zapada tutaj.

**Etap 2: bateria i failsafe**

- [ ] `b` pokazuje napięcie. Porównaj z multimetrem i popraw `batCal` (w panelu jest kalibracja multimetrem, z konsoli: `{"t":"cfg","batCal":1.02}`).
- [ ] Na zasilaczu regulowanym zejdź poniżej 6,2 V przy pracującym rotorze: silniki stają, w logu `bat_crit` i `fs_battery`, `r50` odpowiada `low_battery`. Po podniesieniu powyżej 6,5 V start znowu działa.
- [ ] Rotor na `r50` z `ka on`, potem `ka off`: po pół sekundzie rotor staje, w logu `fs_timeout`.
- [ ] W logu startu widać powód ostatniego resetu (`poweron`, `brownout`, `task_wdt`), watchdog ma 5 s.

**Etap 3: WiFi, WebSocket, panel**

- [ ] Telefon łączy się z `Turbina-XXXX`, `http://192.168.4.1` otwiera pełny panel.
- [ ] START, suwak i HAMUJ sterują rotorem, LEWO i PRAWO gondolą, bateria jest widoczna.
- [ ] Rotor kręci się, wyłączasz WiFi w telefonie: rotor staje w czasie poniżej sekundy, w logu `fs_timeout` albo `fs_disconnect`.
- [ ] Trzymasz PRAWO i blokujesz ekran telefonu: gondola staje.
- [ ] Pasek statusu pokazuje opóźnienie, przy dobrym zasięgu kilkadziesiąt ms.

**Etap 4: gondola bez czujnika**

- [ ] Kalibracja obrotu zapisuje czas pełnego obrotu, panel pokazuje szacowany kąt.
- [ ] Po ustawieniu limitu skrętu gondola zatrzymuje się na progu, przycisk w tę stronę jest wyszarzony, pojawia się baner o limicie, w drugą stronę jedzie.
- [ ] Restart z panelu: wskaźnik skrętu pokazuje to samo saldo co przed restartem.
- [ ] ZERUJ POZYCJĘ kasuje saldo.

**Etap 5: BLE**

- [ ] Chrome na Androidzie, panel z GitHub Pages, telefon odłączony od WiFi turbiny: Połącz BLE, oba silniki sterowane, telemetria płynie.
- [ ] Zerwanie BLE (wyłącz Bluetooth w telefonie) zatrzymuje silniki.
- [ ] Pobór prądu w spoczynku zmierzony multimetrem szeregowo z baterią: sam AP, AP i reklama BLE, połączenie BLE. Wyniki wpisz do `docs/` (na przykład `docs/POBOR_PRADU.md`).

**Etap 6: OTA i konfiguracja**

- [ ] Aktualizacja firmware z telefonu: po restarcie w sekcji System widać nową datę builda.
- [ ] Aktualizacja `littlefs.bin` z telefonu: po restarcie panel ładuje się w nowej wersji.
- [ ] Zmiana `rotorMax` przeżywa restart.
- [ ] OTA przy kręcącym się rotorze jest odrzucone.

**Etap 7: czujnik (gdy dojdzie sprzęt)**

- [ ] Krańcówki: najazd zatrzymuje ruch w tę stronę, w telemetrii zapala się limit, w drugą stronę jedzie od razu.
- [ ] AS5600: kąt w panelu zgadza się z kątomierzem z dokładnością ±3 stopnie w kilku położeniach.
- [ ] "Jedź do kąta" na 90, 180 i 270 stopni kończy w oknie ±2 stopnie.
- [ ] Zdjęcie magnesu: błąd w logu, kompas znika albo przechodzi na szacunek, sterowanie ręczne dalej działa.

**Etap 8: ESP32-S3**

- [ ] Piny S3 potwierdzone dla konkretnej płytki i poprawione w `include/config.h` (dioda RGB to GPIO 48 albo 38, zależnie od wersji płytki).
- [ ] `pio run -e esp32-s3-devkitc-1 -t upload` i `-t uploadfs`, konsola działa po natywnym USB.
- [ ] Checklisty etapów 1 do 7 przechodzą na S3 bez zmian w kodzie poza `config.h`.

## Odstępstwa od specyfikacji

- **OTA bez ElegantOTA.** ElegantOTA jest na licencji AGPL, ciągnie starszy fork AsyncWebServer i ma własny wygląd. Zamiast tego jest wbudowany upload firmware i systemu plików w panelu, z hasłem AP, paskiem postępu i odmową przy pracujących silnikach.
- **Czysta logika w `lib/turbine_core/`.** Parser komend, rampa, failsafe, progi baterii i logika gondoli nie zależą od Arduino, więc `pio test -e native` testuje je na komputerze. Moduły w `src/` mają nazwy i role ze specyfikacji.
- **Dodatkowe moduły w `src/`**: `settings`, `cmd_queue`, `event_log`, `status_led`, `serial_console`, `shared_state` i `app.h`. Bez nich `main.cpp` przestałby być samą sklejką.
- **Saldo skrętu ważone prędkością.** Liczymy ms przy 100 %, a nie gołe ms. Inaczej limit ustawiony przy jednej prędkości nie chroniłby przewodów przy innej.
- **Krańcówki dociągają saldo.** "Zeruje saldo po tej stronie" czytamy tak: pierwsze dotknięcie krańcówki zapamiętuje saldo jako punkt odniesienia, każde następne przywraca saldo do tej wartości i kasuje narosły dryf.
- **Deadman gondoli.** Panel przy trzymaniu przycisku powtarza ramkę `hold` co 200 ms, a firmware zatrzymuje gondolę, gdy jej zabraknie przez 0,7 s (albo `fsTimeout`, jeśli dłuższy). To łapie przypadek, gdy drugi telefon pinguje i utrzymuje połączenie przy życiu.
- **`fsTimeout` ustawiany w zakresie 400 do 1000 ms.** Dolna granica zostawia zapas nad pingiem co 300 ms, górna trzyma "zerwanie WiFi zatrzymuje silniki w czasie poniżej 1 s".
- **Brak baterii poniżej 2,5 V.** Taki odczyt to praca z USB albo odpięty dzielnik. Nie blokuje silników na biurku, ale też nigdy nie zdejmuje blokady krytycznej baterii, bo odpadnięty przewód pomiarowy nie może odblokować silników.
- **"Bez ostrzeżeń" dotyczy naszego kodu.** `-Wall -Wextra` obejmuje `src/` i `lib/turbine_core/`. Ostrzeżenia z bibliotek zewnętrznych zostawiamy ich autorom.
- **Partycje `min_spiffs.csv`.** WiFi, BLE i serwer nie mieszczą się w domyślnym slocie 1,25 MB. Mamy 2x 1,9 MB na firmware (z OTA) i 128 kB na LittleFS.
- **Jeden PR zamiast osobnego na każdy etap**, z logicznie podzielonymi commitami. Etapów nie dało się sprawdzić na sprzęcie, więc checklisty wyżej zastępują opis "co sprawdzono".

## Gdy coś nie działa

**ESP resetuje się przy starcie silnika na baterii 9 V.** To brownout: kostka 9 V ma duży opór wewnętrzny i napięcie na chwilę siada. W logu startu widać `reset brownout`. Przejdź na 2x 18650 albo dołóż pojemność: 470 do 1000 µF na wyjściu 5 V przy DRV8833 i 100 nF na zaciskach silnika. Pomaga też dłuższa rampa (`rotorRamp`).

**Silnik piszczy.** PWM powinien chodzić na 20 kHz (`MOTOR_PWM_FREQ_HZ` w `config.h`), czyli poza zakresem słuchu. Jeśli piszczy mimo to, sprawdź, czy wgrany jest aktualny firmware. Ciche buczenie przy bardzo małej prędkości to zwykle silnik, który jeszcze nie ruszył; podnieś `rotorMin`.

**Napięcie baterii za niskie albo skacze.** Skalibruj `batCal` multimetrem (Konfiguracja, Bateria) i sprawdź, czy przy GPIO 34 jest kondensator 100 nF do masy. Dzielnik ma dużą rezystancję i bez kondensatora ADC łapie zakłócenia od silników.

**Telefon odłącza się od sieci turbiny.** Telefon widzi sieć bez internetu i ucieka na dane komórkowe. Na pytanie o pozostanie w sieci odpowiedz tak, wyłącz dane komórkowe albo funkcję typu "inteligentne przełączanie sieci". Pomaga też włączenie captive portalu (`captive` w ustawieniach sieci, po restarcie): telefon traktuje wtedy panel jak stronę logowania.

**Pod 192.168.4.1 jest tylko strona z formularzem.** Panel nie został wgrany. Zrób `pio run -e esp32dev -t uploadfs` albo wgraj `littlefs.bin` przez ten formularz.

**Brak przycisku Połącz BLE.** Panel otwarty z `http://` nie widzi Bluetootha. Użyj kopii z GitHub Pages albo flagi Chrome opisanej wyżej.

## Praca nad kodem

**Symulator ESP.** `tools/mock_esp.mjs` (Node 18 lub nowszy, bez zależności) udaje firmware: serwuje `data/`, mówi tym samym protokołem po WebSocket i HTTP, ma rampy, failsafe, deadman gondoli, baterię, która siada pod obciążeniem, i OTA z restartem. Logika jest przeniesiona z firmware, więc panel, który działa na symulatorze, powinien działać na płytce.

```
node tools/mock_esp.mjs                       # http://localhost:8080
node tools/mock_esp.mjs --sensor as5600       # albo endstops
node tools/mock_esp.mjs --bat 6.5 --lag 120   # słaba bateria, opóźnienie sieci
node tools/mock_esp.mjs --no-ws               # wymusza tryb HTTP w panelu
```

Domyślnie słucha na wszystkich interfejsach, więc panel otworzysz też na telefonie w tej samej sieci (adres komputera i port 8080). Do testów automatycznych są dodatkowe adresy: `GET /__mock/frames` (ramki odebrane od klientów), `POST /__mock/clear`, `POST /__mock/drop` (zrywa wszystkie WebSockety), `POST /__mock/set` z JSON-em typu `{"bat":6.0}` i `GET /__mock/state`. Opis wszystkich opcji jest na początku pliku.

**Testy logiki:** `pio test -e native`.

**Ikony** `data/icon-192.png` i `data/icon-512.png` generuje `python3 tools/make_icons.py` (sama biblioteka standardowa, rysuje to samo co `data/icon.svg`).

**CI.** `.github/workflows/build.yml` przy każdym pushu i pull requeście kompiluje wszystkie środowiska, buduje obraz LittleFS i uruchamia testy natywne. Pliki `firmware.bin` i `littlefs.bin` lądują jako artefakt do pobrania. `.github/workflows/pages.yml` publikuje panel na GitHub Pages.
