/*
   PULSE GARDEN - Proyecto de robotíca Dimitri Andreas Mitrofanakis y Ale mitrofanakis Gutiérrez
   6 Servos presentes: 3 plantas "normales" que bailan, 1 pájaro que se levanta y baila y dos flores que giran sobre su eje (scam MG90S)
   + Intento de integración con IA -Qwen2.5.0.5b. casi exitoso
*/

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- WIFI & AI STUFF ---
const char* ssid = "Piso3";
const char* password = "Gh1ll4rd1P3";
const char* pcIP = "192.168.50.164";     // mi pc en casa
WiFiUDP udp;
const int udpPort = 8888;
unsigned long lastSentToAI = 0;


const int trigPin = 13;
const int echoPin = 12;
const int plantPins[3] = {18, 19, 21};       // plantas baile
const int birdPin = 23;                     // Pajarito que sube y baja
const int spinFlower1 = 22;                 // flores giratori (continua)
const int spinFlower2 = 25;                 // 2da flor giratoria

Servo plants[3];
Servo bird;
Servo flowerSpinA;
Servo flowerSpinB;

// --- edo. Jardín ---
bool someoneHere = false;
unsigned long lastSeenTime = 0;
const unsigned long dieAfter = 20000;       // 20 segundos de soledad = muerte

unsigned long nextCheck = 0;
const unsigned long checkEvery = 5000;      // chequear sensor periódicamente cada (5) segs
const float maxDist = 40.0;                 // Distacnia del Humano

unsigned long lastUpdate = 0;
const int moveDelay = 40;

float moodSpeed = 1.0;                      // 0.5 = calm, 1.0 = normal, 2.0 = excited (para la ia)

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // conectar al wifi para hablar con qwencito
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado! IP: " + WiFi.localIP().toString());
  udp.begin(udpPort);

  // Serovs initialize
  for(int i=0; i<3; i++) {
    plants[i].attach(plantPins[i]);
    plants[i].write(90);        // posición neutral
  }
  bird.attach(birdPin); bird.write(0);     // pájaro muerto al inicio
  flowerSpinA.attach(spinFlower1); flowerSpinA.write(90);  // Pájaro parado :)
  flowerSpinB.attach(spinFlower2); flowerSpinB.write(90);

  Serial.println("Pulse Garden listo... esperando visita humana");
}

void loop() {
  unsigned long ahora = millis();

  // si nadie ha venido todavía
  if (!someoneHere) {
    if (getDistance() < maxDist && getDistance() > 0) {
      wakeUpGarden();
    }
    delay(100);
    return;
  }

  // movimientos suaves
  if (ahora - lastUpdate >= moveDelay) {
    moveEverything();
    lastUpdate = ahora;
  }

  // Morir si nadie aparece en 20 segundos
  if (ahora - lastSeenTime >= dieAfter) {
    killGarden();
    return;
  }

  // chequear presencia en perodos de 5 secs
  if (ahora >= nextCheck) {
    float d = getDistance();
    if (d > 0 && d < maxDist) {
      lastSeenTime = ahora;
      Serial.println("Persona detectada! Jardin sigue vivo");
    }
    nextCheck = ahora + checkEvery;

    // enviar distancia a la IA en la pc disponible
    sendToAI(d);
    receiveMoodFromAI();
  }
}

// --- HC SRO4 (sensor)---
float getDistance() {
  digitalWrite(trigPin, LOW);   delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 25000);
  if (dur == 0) return -1;
  return dur * 0.0343 / 2.0;
}

// --- comunicacion con la IA ---
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

// --- otras Acciones ---
void wakeUpGarden() {
  someoneHere = true;
  lastSeenTime = millis();
  nextCheck = millis() + checkEvery;

  Serial.println("ALGUIEN LLEGO!! Jardin despierta!!");
  flowerSpinA.write(70);
  flowerSpinB.write(110);
  smoothRise(bird, 0, 90);
  moodSpeed = 1.0;
}

void killGarden() {
  someoneHere = false;
  Serial.println("20 segundos solo... el jardin muere :(");
  flowerSpinA.write(90);
  flowerSpinB.write(90);
  smoothRise(bird, 90, 0);
}

void moveEverything() {
  float secs = (millis() - lastSeenTime) / 1000.0;
  float fase = secs * PI * moodSpeed;   // la velocidad depende del mood determinada por qwen
  float onda = sinf(fase);

  // Plantas baile
  int ang = 90 + (int)(85.0f * onda);
  for(int i=0; i<3; i++) plants[i].write(ang);

  // pájaro "respirando" (oscilar entre 90 y 85 grados)
  int birdMove = 90 + (int)(5.0f * onda);
  bird.write(birdMove);
}

void smoothRise(Servo &s, int inicio, int fin) {
  int paso = (fin > inicio) ? 2 : -2;
  for(int p=inicio; (paso>0 && p<fin) || (paso<0 && p>fin); p += paso) {
    s.write(p);
    delay(18);
  }
  s.write(fin);
}