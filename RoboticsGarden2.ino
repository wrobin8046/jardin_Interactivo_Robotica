/*
   PULSE GARDEN - Proyecto de robotíca Dimitri Andreas Mitrofanakis y Ale mitrofanakis Gutiérrez
   6 Servos presentes: 3 plantas "normales" que bailan, 1 pájaro que se levanta y baila y dos flores que giran sobre su eje (scam MG90S)
   + Intento de integración con IA -Qwen2.5.0.5b.
*/

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <FastLED.h>   //  libreria para ws2812b,  Super facil para animaciones

// --- WIFI & AI STUFF
const char* ssid = "Piso3";
const char* password = "Gh1ll4rd1P3";
const char* pcIP = "192.168.50.164";     // mi pc (R5 + 3060)
WiFiUDP udp;
const int udpPort = 8888;
unsigned long lastSentToAI = 0;

// --- Lo de las LEDs 
#define LED_PIN 14  
#define NUM_LEDS 30 
#define LED_TYPE WS2812B 
#define COLOR_ORDER GRB 
CRGB leds[NUM_LEDS]; 
unsigned long lastLedUpdate = 0; 
const int ledDelay = 50;   //  delay depara animacion,  mas bajo = Mas apido

// --- PINES 
const int trigPin = 13;
const int echoPin = 12;
const int plantPins[3] = {18, 19, 21};       // 3 plantas que bailan
const int birdPin = 23;                     // pajarito
const int spinFlower1 = 22;                 // flor giratria 
const int spinFlower2 = 25;                 // 2da flor giratoria

Servo plants[3];
Servo bird;
Servo flowerSpinA;
Servo flowerSpinB;

// --- Edo. jardin 
bool someoneHere = false;
unsigned long lastSeenTime = 0;
const unsigned long dieAfter = 20000;       

unsigned long nextCheck = 0;
const unsigned long checkEvery = 5000;      // Sensor detecta periodicamente cada (5 segs
const float maxDist = 40.0;                

unsigned long lastUpdate = 0;
const unsigned long moveDelay = 40;

float moodSpeed = 1.0;                      // 0.5 = calm, 1.0 = normal, 2.0 = excited

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // conectar wifi para hablar con Qwen
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado! IP: " + WiFi.localIP().toString());
  udp.begin(udpPort);

  // servs inicializacion
  for(int i=0; i<3; i++) {
    plants[i].attach(plantPins[i]);
    plants[i].write(90);        // posición neutral
  }
  bird.attach(birdPin); bird.write(0);     // pájaro muerto
  flowerSpinA.attach(spinFlower1); flowerSpinA.write(90);  // parado parado :)
  flowerSpinB.attach(spinFlower2); flowerSpinB.write(90);

  // leds para el rio
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(64); 
  clearLeds();                 

  Serial.println("Pulse Garden listo... esperando visita humana");
}

void loop() {
  unsigned long ahora = millis();

  // Si  nadie ha venido todavía
  if (!someoneHere) {
    if (getDistance() < maxDist && getDistance() > 0) {
      wakeUpGarden();
    }
    delay(100);
    return;
  }

  //   movimientos suaves
  if (ahora - lastUpdate >= moveDelay) {
    moveEverything();
    lastUpdate = ahora;
  }

  // morir si nadie aparece en 20 segundos
  if (ahora - lastSeenTime >= dieAfter) {
    killGarden();
    return;
  }

  // Chequear presencia cada 5 segundos
  if (ahora >= nextCheck) {
    float d = getDistance();
    if (d > 0 && d < maxDist) {
      lastSeenTime = ahora;
      Serial.println("Persona detectada! Jardin sigue vivo");
    }
    nextCheck = ahora + checkEvery;

    // enviar distancia a qwen
    sendToAI(d);
    receiveMoodFromAI();
  }
}

// --- sensor Wall-e
float getDistance() {
  digitalWrite(trigPin, LOW);   delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 25000);
  if (dur == 0) return -1;
  return dur * 0.0343 / 2.0;
}

// --- Qwen
void sendToAI(float dist) {
  if (millis() - lastSentToAI < 3000) return;
  udp.beginPacket(pcIP, udpPort);
  udp.printf("DISTANCE:%.1f", dist);
  udp.endPacket();
  lastSentToAI = millis();
}

void receiveMoodFromAI() {
  int packet = udp.parsePacket();
  if (packet) {
    char buffer[64];
    int len = udp.read(buffer, 64);
    buffer[len] = 0;
    String msg = String(buffer);

    if (msg.indexOf("MOOD:EXCITED") != -1) {
      moodSpeed = 2.0;
      flowerSpinA.write(50);   // gira rapido
      flowerSpinB.write(130);
      Serial.println("AI dice: EXCITED MODE!");
    }
    else if (msg.indexOf("MOOD:NEUTRAL") != -1) {
      moodSpeed = 1.0;
      flowerSpinA.write(70);
      flowerSpinB.write(110);
    }
    else if (msg.indexOf("MOOD:CALM") != -1) {
      moodSpeed = 0.5;
      flowerSpinA.write(80);
      flowerSpinB.write(100);
      Serial.println("AI dice: modo calmado...");
    }
  }
}

// --- Miscelanious acciones
void wakeUpGarden() {
  someoneHere = true;
  lastSeenTime = millis();
  nextCheck = millis() + checkEvery;

  Serial.println("ALGUIEN LLEGO!! Jardin despierta!!");
  flowerSpinA.write(70);
  flowerSpinB.write(110);
  smoothRise(bird, 0, 90);
  moodSpeed = 1.0;

  // prender leds azul para el rio
  FastLED.setBrightness(64);   // Brillo cuando vivo
}

void killGarden() {
  someoneHere = false;
  Serial.println("20 segundos solo... el jardin muere :(");
  flowerSpinA.write(90);
  flowerSpinB.write(90);
  smoothRise(bird, 90, 0);

  // apagar leds para el rio
  clearLeds();
}

void moveEverything() {
  float secs = (millis() - lastSeenTime) / 1000.0;
  float fase = secs * PI * moodSpeed;   // La velocidad depende del mood
  float onda = sinf(fase);

  // Plantas  bailando
  int ang = 90 + (int)(85.0f * onda);
  for(int i=0; i<3; i++) plants[i].write(ang);

  // pájaro "respirando" (oscilacion entre 90 y 85 grados)
  int birdMove = 90 + (int)(5.0f * onda);
  bird.write(birdMove);

  // actualizar animacion del rio si jardin = vivo
  updateRiverFlow(fase);   // usa la fase para flujo
}

// --- ANIMACION DEL RIO (flujo de agua azul) ---
void updateRiverFlow(float fase) {
  if (millis() - lastLedUpdate < (ledDelay / moodSpeed)) return;   // velocidad depende de mood!!

  static uint8_t offset = 0;
  offset += moodSpeed * 2;    // offset para flujo,  mas rapido si mood = "excited"

  for(int i = 0; i < NUM_LEDS; i++) {
    uint8_t bright = sin8((i * 16) + offset);   // brillo ondulado para efecto agua
    leds[i] = CHSV(160, 255, bright);           // azul-"oso",  saturado,  brillo variable
  }

  FastLED.show();
  lastLedUpdate = millis();
}

// --- LEDS apagado
void clearLeds() {
  for(int i=0; i<NUM_LEDS; i++) leds[i] = CRGB::Black;   // Negro
  FastLED.show();
}

// --- Movto. suave (pajarito)
void smoothRise(Servo &s, int inicio, int fin) {
  int paso = (fin > inicio) ? 2 : -2;
  for(int p=inicio; (paso>0 && p<fin) || (paso<0 && p>fin); p += paso) {
    s.write(p);
    delay(18);
  }
  s.write(fin);
}