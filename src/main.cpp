#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>

#include "User_Setup_Select.h"
#include "data.h"
#include "Settings.h"
#include <UbiConstants.h>
#include <UbidotsEsp32Mqtt.h>
#include <UbiTypes.h>
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "DHT.h"

#define DHTPIN 27 // pin 27 del ttgo
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
TFT_eSPI tft = TFT_eSPI();

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

const char *UBIDOTS_TOKEN = "BBUS-MhREkiB2dovQplRO9Jr77aisaszzZm"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "esp32";                                // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL1 = "Temperatura";                       // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "Humedad";
const char *SUBSCRIBE_DEVICE_LABEL = "esp32";      // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL1 = "switch1"; // Replace with the variable label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL2 = "switch2";

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;

const uint8_t LED = 12;

Ubidots ubidots(UBIDOTS_TOKEN);

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    if ((char)payload[0] == '1')
    {
      tft.fillCircle(40, 170, 19, TFT_DARKGREEN); // rellenos CAMBIAN CON LOS SWITCH
      digitalWrite(LED, HIGH);
    }
    else if ((char)payload[0] == '0')
    {
      tft.fillCircle(40, 170, 19, TFT_BLACK);
      digitalWrite(LED, LOW);
    }
    else if ((char)payload[0] == '3')
    {
      tft.fillCircle(95, 170, 19, TFT_BLACK);
    }
    else if ((char)payload[0] == '4')
    {
      tft.fillCircle(95, 170, 19, TFT_PURPLE);
    }
  }
  Serial.println();
}

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("TTGO_JMV", "");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();

  pinMode(LED, OUTPUT);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED);
  tft.drawString("Sin conexion a", 15, 100, 2);
  tft.drawString(settings.ssid, 20, 120, 2);

  // Rutinas de Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL1); // Insert the device and variable's Labels, respectively
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL2); // Insert the device and variable's Labels, respectively

  timer = millis();

  Serial.println(F("DHTxx test!")); // sensor
  dht.begin();

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Temperatura", 10, 18, 2);
  tft.drawString("Humedad", 10, 88, 2);
  tft.drawString("`C", 63, 40, 4);
  tft.drawString("%", 63, 110, 4);

  tft.drawCircle(40, 170, 20, TFT_GREEN); // CONTORNOS
  tft.drawCircle(95, 170, 20, TFT_MAGENTA);
}

void setup()
{

  tft.init();
  tft.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Configuracion WiFi", 10, 30, 2);
  tft.setTextColor(TFT_BLUE);
  tft.drawString("Conectese a", 35, 110, 2);
  tft.drawString("TTGO_JMV", 3, 135, 4);
  tft.drawString("e ingrese a", 40, 180, 2);
  tft.drawString("192.168.4.1", 2, 205, 4);

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    if (!ubidots.connected())
    {
      ubidots.reconnect();
    }
    if (ubidots.connected())
    {
      tft.setTextColor(TFT_BLUE);
      tft.drawString("Conectado a", 20, 200, 2);
      tft.drawString(settings.ssid, 20, 220, 2);
    }

    if ((millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      // float value = analogRead(analogPin);
      float h = dht.readHumidity();
      float t = dht.readTemperature();

      if (isnan(h) || isnan(t))
      { // revisar errores
        Serial.println(F("ERROR"));
        tft.setTextColor(TFT_RED);
        tft.drawString("ERROR", 10, 45, 2);
        tft.drawString("ERROR", 10, 115, 2);
        return;
      }

      Serial.print(F("Humedad: "));
      Serial.print(h);
      Serial.print(F("%  Temperatura: "));
      Serial.print(t);
      Serial.print(F("°C "));
      Serial.println();

      tft.fillRect(9, 39, 53, 30, TFT_BLACK); // borrar valor anterior
      tft.fillRect(9, 109, 53, 30, TFT_BLACK);

      tft.setTextColor(TFT_CYAN);
      tft.drawString(String(t, 1), 10, 40, 4);
      tft.drawString(String(h, 1), 10, 110, 4);

      ubidots.add(VARIABLE_LABEL1, t); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL2, h);

      ubidots.publish(DEVICE_LABEL);
      timer = millis();
    }
    ubidots.loop();
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}
