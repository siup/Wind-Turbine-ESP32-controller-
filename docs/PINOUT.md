# Piny: ESP32 DevKit V1 i ESP32-S3

Numery GPIO są wyłącznie w `include/config.h`, w blokach `#if CONFIG_IDF_TARGET_ESP32` / `CONFIG_IDF_TARGET_ESP32S3`. Żaden inny plik nie zawiera numeru pinu. Piny S3 to propozycja do potwierdzenia, gdy przyjdzie konkretna płytka, bo zależą od wariantu modułu i pamięci PSRAM.

## Tabela

| Funkcja | Makro | ESP32 DevKit V1 | ESP32-S3 | Uwagi |
| --- | --- | --- | --- | --- |
| Rotor IN1 | `PIN_ROTOR_IN1` | GPIO 25 | GPIO 4 | PWM LEDC 20 kHz, 8 bit |
| Rotor IN2 | `PIN_ROTOR_IN2` | GPIO 26 | GPIO 5 | PWM LEDC |
| Yaw IN1 | `PIN_YAW_IN1` | GPIO 27 | GPIO 6 | PWM LEDC |
| Yaw IN2 | `PIN_YAW_IN2` | GPIO 14 | GPIO 7 | na ESP32 GPIO 14 macha stanem przy starcie, patrz niżej |
| DRV8833 SLEEP (EEP) | `PIN_DRV_SLEEP` | GPIO 33 | GPIO 15 | HIGH = mostek aktywny, LOW po 2 s bezczynności obu silników |
| Pomiar baterii | `PIN_BAT_ADC` | GPIO 34 | GPIO 1 | tylko ADC1, dzielnik 100k / 33k, 100 nF do GND |
| Krańcówka L | `PIN_ENDSTOP_L` | GPIO 16 | GPIO 16 | opcja, `INPUT_PULLUP`, aktywna w stanie niskim |
| Krańcówka R | `PIN_ENDSTOP_R` | GPIO 17 | GPIO 17 | opcja, jak wyżej |
| AS5600 SDA | `PIN_I2C_SDA` | GPIO 21 | GPIO 8 | opcja, I2C 400 kHz, adres 0x36 |
| AS5600 SCL | `PIN_I2C_SCL` | GPIO 22 | GPIO 9 | opcja |
| Dioda statusu | `PIN_LED` | GPIO 2 (niebieska) | GPIO 48 (WS2812) | na S3 `LED_IS_RGB 1`, kolory przez `rgbLedWrite` |
| Konsola | | UART0, GPIO 1 / 3 przez CH340C | natywne USB, GPIO 19 / 20 | S3: `ARDUINO_USB_CDC_ON_BOOT=1`, gniazdo opisane USB |

## Piny, których nie używamy

**ESP32 (WROOM-32D):**

- GPIO 6 do 11: pamięć flash. Nigdy.
- GPIO 0, 2, 5, 12, 15: piny konfiguracyjne (strapping), czytane przy starcie. Nie podłączamy tu niczego, co może wymusić stan przy włączaniu. Szczególnie GPIO 12: stan wysoki przy starcie przestawia napięcie flash na 1,8 V i ESP nie wstaje. GPIO 2 obsługuje tylko wbudowaną diodę, jako wyjście jest w porządku.
- GPIO 34 do 39: tylko wejścia, bez wewnętrznych rezystorów podciągających. Dobre na ADC, złe na krańcówki.
- GPIO 1 i 3: UART0, czyli konsola.
- ADC2 (GPIO 0, 2, 4, 12 do 15, 25 do 27) nie działa przy włączonym WiFi, dlatego bateria siedzi na ADC1 (GPIO 32 do 39).
- GPIO 16 i 17 są wolne na module WROOM. Na modułach WROVER zajmuje je PSRAM i wtedy krańcówki trzeba przenieść.
- Przy starcie stanem machają GPIO 1, 3, 5, 14 i 15. Z tej listy używamy tylko 14, opis niżej.

**ESP32-S3:**

- GPIO 0, 3, 45, 46: piny konfiguracyjne.
- GPIO 19 i 20: natywne USB, przez nie idzie konsola.
- GPIO 26 do 32: flash i PSRAM w trybie quad, zawsze zajęte.
- GPIO 35, 36, 37: zajęte na modułach z PSRAM octal (oznaczenia N8R8, N16R8, N32R8). Przy PSRAM quad (np. N4R2) są wolne, ale lepiej ich nie planować.
- GPIO 43 i 44: UART0.
- ADC2 (GPIO 11 do 20) nie działa przy WiFi, ADC1 to GPIO 1 do 10.
- GPIO 15 i 16 to piny na kwarc 32 kHz. DevKitC-1 go nie ma, więc są wolne.

## Uwagi

### GPIO 14 przy starcie i pin EEP na klonach DRV8833

ESP32 przez chwilę podczas startu, jeszcze przed `setup()`, wystawia na GPIO 14 przebieg PWM. Sam układ DRV8833 ignoruje wtedy wejścia, bo ma wewnętrzny rezystor ściągający nSLEEP do masy, a `motorsEarlyInit()` w pierwszej linijce `setup()` ustawia SLEEP i wszystkie wejścia mostka na LOW. Na gołym module to więc nieszkodliwe.

Problem robi się, gdy moduł sam podciąga EEP (nSLEEP) do VCC rezystorem. Wiele czerwonych klonów DRV8833 tak ma. Wtedy mostek jest aktywny od włączenia zasilania i silnik gondoli może drgnąć przy starcie ESP. Sprawdź to omomierzem przy odłączonym zasilaniu: opór między EEP a VCC modułu.

- Rezystor podciągający 47k albo większy: wystarczy dołożyć 10k z EEP do GND. Mostek śpi, dopóki ESP go nie obudzi.
- Mniejszy (np. 10k): sam rezystor do GND zrobi z nim dzielnik i stan będzie niepewny. Wylutuj podciąganie (albo przetnij ścieżkę lub zworkę) i daj 10k do GND.
- Jeśli nie chcesz ruszać modułu: przenieś YAW IN2 z GPIO 14 na pin, który przy starcie milczy, na przykład GPIO 32, i zmień `PIN_YAW_IN2` w `config.h`.

Pozostałe piny mostka (25, 26, 27, 33) są przy starcie w stanie wysokiej impedancji, a wejścia DRV8833 mają wewnętrzne rezystory do masy, więc silniki są na wybiegu.

### Kondensator na wejściu ADC

Między GPIO 34 a GND daj kondensator 100 nF, jak najbliżej ESP. Dzielnik 100k / 33k ma oporność wyjściową około 25 kΩ. Bez kondensatora próbkowanie ADC i zakłócenia od PWM silników dają odczyt, który skacze o dziesiątki miliwoltów. Firmware dodatkowo uśrednia 8 odczytów na próbkę i 8 kolejnych próbek.

### Dzielnik napięcia

Na pinie jest `V_bat × 33 / (100 + 33)`, czyli około 0,248 napięcia baterii.

| Bateria | Na pinie |
| --- | --- |
| 9,6 V (świeża kostka 9 V) | 2,38 V |
| 8,4 V (2S Li-ion, pełne) | 2,08 V |
| 7,0 V (próg ostrzeżenia 9 V) | 1,74 V |
| 6,2 V (próg krytyczny 9 V) | 1,54 V |
| 6,0 V (próg krytyczny 2S) | 1,49 V |

Firmware czyta `analogReadMilliVolts()` przy tłumieniu 11 dB i przelicza z powrotem: `mV = mV_pin × 133 / 33 × batCal`. Zakres tłumienia kończy się około 3,1 V na pinie (około 12,5 V baterii), a na klasycznym ESP32 najdokładniej jest do około 2,45 V na pinie, czyli mniej więcej 9,9 V baterii. Na 9 V i 2S wystarczy. Pakiet 3S (12,6 V) wymaga innego dzielnika. Dzielnik pobiera stale około 72 µA przy 9,6 V.

### Dioda na ESP32-S3

Na DevKitC-1 w wersji 1.0 dioda RGB (WS2812) siedzi na GPIO 48, na wersji 1.1 na GPIO 38. Wersja jest na spodzie płytki. Przy 1.1 zmień `PIN_LED` w `config.h`.

### PSRAM na ESP32-S3

Moduły z PSRAM octal zabierają GPIO 35, 36 i 37 (oprócz GPIO 26 do 32, które zawsze należą do flash). Proponowane piny S3 omijają oba zakresy, więc działają z każdym wariantem modułu. Sprawdź oznaczenie na metalowej osłonie modułu (np. `N16R8`) zanim przeniesiesz coś w ten obszar.

### Krańcówki i AS5600

Krańcówki podłączamy między pin a GND, styk normalnie otwarty: zwarcie znaczy "krańcówka aktywna", debounce trwa 20 ms. Obie aktywne naraz firmware traktuje jako błąd okablowania. Urwany przewód krańcówki wygląda jak "nieaktywna", wtedy przed skręceniem przewodów chroni już tylko limit z salda czasu, więc warto go ustawić także w tym trybie.

AS5600 zasilaj z 3,3 V, nie z 5 V: moduły mają rezystory podciągające SDA i SCL do własnego VCC, a piny ESP nie znoszą 5 V. Pin DIR podłącz na stałe (GND albo 3,3 V), nie zostawiaj go wiszącego. Magnes diametralny (zwykle 6 mm) 0,5 do 3 mm nad układem. Przewody I2C możliwie krótkie, z dala od przewodów silników.
