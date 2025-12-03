# Pulse Garden  
Instalación interactiva – Proyecto Final 

- Cuando una persona se acerca (< 40 cm) → todo despierta:  
  → 3 plantas bailan suavemente  
  → el pájaro se levanta y respira  
  → dos flores locas giran como si estuvieran felices  
  → una tira de LEDs WS2812B crea un pequeño río azul que fluye entre las plantas

- Si te alejas más de 20 segundos → el jardín muere dramáticamente:  
  todo se para, el río se apaga, el pájaro cae… silencio total.

### Integración con ia
El ESP32 está conectado por Wi-Fi a mi PC y envía cada pocos segundos la distancia de la persona.

En mi PC corre **Qwen 2.5** (modelo local, descargado con Ollama) que interpreta la distancia
| Muy cerca (<20 cm) | “EXCITED” / emocionado         | todo va más rápido: flores giran fuerte, río turbulento, plantas bailan rápido |
| Normal (20-40 cm)  | “NEUTRAL”                      | velocidad normal                                  |
| Lejos (>40 cm)     | “CALM” / tranquilo             | todo más lento y suave, como brisa                |

La IA responde por UDP y el ESP32 ajusta inmediatamente la velocidad de los motores y del río de luz.  

Que se usó
- ESP32 DevKitV1  
- HC-SR04  
- 4 × SG90 + 2 × MG90S rotación continu  
- Tira LED WS2812B (AliExpres) 

Sofwares
- Arduino IDE
-  librerías ESP32Servo y FastLED  
- Qwen2.5
- Ollama

:)
