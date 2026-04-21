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
