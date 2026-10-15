# ESP32 Home Automation MQTT

Dieses Projekt enthält eine Arduino/ESP32-Firmware zur Heimautomation über MQTT.

## Überblick

Die Firmware steuert bis zu **16 dimmbare Lichtkanäle** über PCA9685-Module und veröffentlicht regelmäßig Messwerte über MQTT. Außerdem sind **DHT22**, **INA219**, **ACS712** und **Arduino OTA** integriert.

## Hauptfunktionen

- MQTT-Anbindung mit Reconnect-Logik (WLAN + Broker)
- 16 Lichtkanäle per MQTT-Befehl (`brightness`) steuerbar
- Rückmeldung je Lichtkanal (`true`/`false`)
- Wetter-/Raumwerte via DHT22 (Humidity + Heat Index als Temperaturfeld)
- Spannungs-/Strom-/Leistungsmessung via INA219
- LED-Strom-/Leistungsabschätzung via ACS712
- OTA-Updates über `ArduinoOTA`

## Firmware über OTA flashen

1. Die Firmware zunächst per USB flashen und sicherstellen, dass die WLAN-Zugangsdaten in `_authentification.h` stimmen.
2. ESP32 und Entwicklungsrechner mit demselben Netzwerk verbinden.
3. In der Arduino IDE das passende ESP32-Board auswählen. Sobald die Firmware mit dem WLAN verbunden ist, erscheint unter **Werkzeuge → Port** der Netzwerk-Port `esp32-home-automation-mqtt` (gegebenenfalls mit IP-Adresse).
4. Diesen Netzwerk-Port auswählen und wie gewohnt **Hochladen** starten. Die Firmware initialisiert `ArduinoOTA` nach der WLAN-Verbindung und verarbeitet Uploads fortlaufend in `loop()`.

Falls der Netzwerk-Port nicht erscheint, WLAN-Verbindung und IDE-Netzwerkerkennung prüfen. MQTT ist für OTA nicht erforderlich.

## MQTT API (Kurzfassung)

### Eingehende Topics (Subscribe)

- `livingroom/lights/1/set/brightness`
- ...
- `livingroom/lights/16/set/brightness`

**Payload-Beispiel:**

```json
{"brightness":55}
```

### Ausgehende Topics (Publish)

- `livingroom/wdt/counter`
- `livingroom/weather`
- `livingroom/ina219/voltage`
- `livingroom/ina219/current`
- `livingroom/ina219/power`
- `livingroom/acs712/current`
- `livingroom/acs712/power`
- `livingroom/lights/<1..16>/get/on`

## Voraussetzungen

- ESP32-Board
- Arduino IDE (oder kompatible Umgebung)
- Bibliotheken laut Includes in `esp32-home-automation-mqtt.ino`
- `_authentification.h` mit WLAN/MQTT-Zugangsdaten (siehe Template unter `libraries/_authentification/`)

## Projektstruktur

- `esp32-home-automation-mqtt.ino` – Hauptfirmware
- `libraries/_authentification/_authentification.h.template` – Vorlage für Credentials

## Hinweis

Die Firmware verwendet MQTT auf Port `1883` und OTA ist aktuell ohne Passwort aktiviert. Für Produktivbetrieb sollten Sicherheitsaspekte (Netzsegmentierung, Auth/TLS, OTA-Passwort) geprüft werden.
