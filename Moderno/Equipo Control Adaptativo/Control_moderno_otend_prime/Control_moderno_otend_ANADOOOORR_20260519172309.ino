#define pinFan         8
#define pinHotend     10
#define termPin       13

const float Raux = 460.0; 
const float cA = 1.1384e-03, cB = 2.3245e-04, cC = 9.489e-08; 

bool estado_actuadores[4] = {false, false, false, false}; 

float velocidad_extrusor = 0; 
float temperatura_objetivo = 0;
float velocidad_dc_objetivo = 0;
float velocidad_fan = 0;

float temp_actual = 25.0; 
float rpm_actual = 0.0; 

float maxTemp = 250.0;
float minTemp = 10.0;

// Aquí Python va a inyectar la inteligencia de la Red Neuronal
float Nbar = 8.928891; 
float K_basura[6]; // Ya no usamos las matrices para el heater, solo el Nbar

unsigned long tiempo_anterior = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinFan, OUTPUT);
  pinMode(pinHotend, OUTPUT);
  analogWrite(pinFan, 0);
  analogWrite(pinHotend, 0);
  Serial.println("FrED AI - PI Adaptativo Listo");
}

void loop() {
  // 1. ESCUCHAR A LA RED NEURONAL DE PYTHON
  if (Serial.available() > 0) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    procesarComandoSerial(linea);
  }

  // 2. CONTROL FÍSICO CADA 100ms
  unsigned long tiempo_actual = millis();
  if (tiempo_actual - tiempo_anterior >= 100) {
    tiempo_anterior = tiempo_actual;

    leerSensoresReales(); 

    int pwm_heater_salida = 0;
    if (estado_actuadores[3]) { 
      if (temp_actual == -999 || temp_actual > maxTemp || temp_actual < minTemp) {
        analogWrite(pinHotend, 0); // Seguridad térmica
      } else {
        pwm_heater_salida = calcularControlHibrido();
        analogWrite(pinHotend, pwm_heater_salida);
      }
    } else {
      analogWrite(pinHotend, 0);
    }

    if (estado_actuadores[1]) {
      analogWrite(pinFan, map(velocidad_fan, 0, 100, 0, 255));
    } else {
      analogWrite(pinFan, 0);
    }

    enviarDatosAGUI(pwm_heater_salida);
  }
}

void leerSensoresReales() {
  int reading = analogRead(termPin);
  if (reading <= 10 || reading >= 1020) {
    temp_actual = -999; 
  } else {
    float R = Raux * ((float)reading / (1023.0 - (float)reading));
    float logR = log(R);
    float TempK = 1.0 / (cA + cB * logR + cC * logR * logR * logR);
    temp_actual = TempK - 273.15;
  }
  rpm_actual = 0.0;
}

int calcularControlHibrido() {
  float error = temperatura_objetivo - temp_actual;
  static float error_integral = 0;

  // Zona Activa: Solo usamos el integrador cuando estamos a menos de 20 grados del objetivo
  if (abs(error) < 20.0) {
    error_integral += error * 0.1; 
  } else {
    error_integral = 0; // Si estamos lejos, no acumulamos basura
  }
  
  // Le damos un límite sano al integrador (Suficiente para empujar, pero sin saturar)
  error_integral = constrain(error_integral, -120, 120);

  // LA MAGIA: 
  // Nbar (de Python) empuja la mayor parte, dependiendo de cómo la IA ve el sistema.
  // El integrador hace el trabajo sucio de subir esos últimos 5 grados.
  float u_crudo = (Nbar * error) + (1.2 * error_integral);

  return constrain((int)u_crudo, 0, 255);
}

void procesarComandoSerial(String cmd) {
  if (cmd.startsWith("<") && cmd.endsWith(">")) {
    cmd = cmd.substring(1, cmd.length() - 1); 
    int commaIndex;
    for (int i = 0; i < 6; i++) {
      commaIndex = cmd.indexOf(',');
      if (commaIndex != -1) {
        K_basura[i] = cmd.substring(0, commaIndex).toFloat();
        cmd = cmd.substring(commaIndex + 1);
      }
    }
    // ¡Aquí actualizamos el Nbar con lo que dice tu Red Neuronal!
    Nbar = cmd.toFloat(); 
  }
  else if (cmd.startsWith("ACTUATE:")) {
    String estados = cmd.substring(8);
    for (int i = 0; i < 4; i++) {
      estado_actuadores[i] = (estados.charAt(i) == '1');
    }
  }
  else if (cmd.startsWith("TEMP:")) {
    temperatura_objetivo = cmd.substring(5).toFloat();
  }
  else if (cmd.startsWith("SPEED:")) {
    velocidad_extrusor = cmd.substring(6).toFloat();
  }
  else if (cmd.startsWith("FANSPEED:")) {
    velocidad_fan = cmd.substring(9).toFloat();
  }
}

void enviarDatosAGUI(int pwm_heater) {
  Serial.print("Temp:"); Serial.println(temp_actual);
  Serial.print("Motor DC RPM:"); Serial.println(rpm_actual);
  Serial.print("Fan:"); Serial.println(estado_actuadores[1] ? "Encendido" : "Apagado");
  Serial.print("Extruder:"); Serial.println(estado_actuadores[2] ? "Encendido" : "Apagado");
  Serial.print("Heater:"); Serial.println(pwm_heater > 0 ? "Encendido" : "Apagado"); 
}