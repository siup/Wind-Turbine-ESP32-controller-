# Pobór prądu

Wyniki pomiarów z etapu 5 (patrz README, "Odbiór etapów"). Wpisz swoje wartości do tabeli, razem z płytką i wersją firmware, bo zależą od jednego i drugiego.

Płytka: ............ (np. ESP32 DevKit V1)
Firmware: ............ (z panelu: Konfiguracja, System, Firmware)
Zasilanie: ............ (np. bateria 9 V przez MP1584 na 5 V)

| Stan | Napięcie [V] | Prąd [mA] | Uwagi |
| --- | --- | --- | --- |
| Sam AP, nikt nie połączony, BLE wyłączony (`bleMode` 0) | | | |
| AP i reklama BLE (`bleMode` 1, nikt na WiFi) | | | |
| Telefon połączony przez WiFi, panel otwarty | | | |
| Telefon połączony przez BLE | | | |
| Rotor 50 %, gondola stoi | | | |
| Rotor 100 % | | | |
| Gondola w ruchu (`yawSpeed`) | | | |
| Rozruch rotora od zera (szczyt) | | | |

## Jak mierzyć

1. Multimetr na zakresie prądu (mA, przy silnikach 10 A) wpinasz szeregowo między plus baterii a wejście przetwornicy, czyli mierzysz prąd pobierany z baterii, nie z 5 V.
2. Napięcie mierzysz drugim multimetrem na zaciskach baterii w tym samym momencie (albo zaraz potem, bez zmiany stanu).
3. Każdy stan trzymaj co najmniej 10 s i zapisz wartość, która się ustaliła. Przy WiFi prąd skacze w rytm beaconów: zapisz średnią, a skrajne wartości w uwagach.
4. Rozruchu multimetr zwykle nie złapie. Jeśli masz miernik z funkcją MAX, użyj jej; jeśli nie, wpisz "nie zmierzono".
5. Po pomiarach przepnij przewód multimetru z powrotem na gniazdo napięciowe, żeby nie zewrzeć baterii przy następnym pomiarze napięcia.

Przy okazji warto sprawdzić w logu zdarzeń (panel, Log zdarzeń), czy przy rozruchu nie było resetu `brownout`. Jeśli był, opisz to w uwagach przy rozruchu.
