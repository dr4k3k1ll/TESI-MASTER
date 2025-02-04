#include <SoftwareSerial.h> //libreria che gestisce la comunicazione seriale tra il master (arduino nano) e lo slave (esp 32 cam)
#include <Adafruit_GFX.h>     // accellereometro
#include <Adafruit_MPU6050.h> // accellereometro
#include <Adafruit_Sensor.h>  // accellerometro
#include <Wire.h> //I2C
// DEFINIZIONI
#define N 5
#define M 3
#define G 9.81

//VARIABILI
float Accelerazione[M][N];
int flag = 0;
int A_X = 0;
int A_Y = 0;
float A_Z = 0;
unsigned long lastCommandSent = 0; // Timer per invio comandi
const long espInterval = 5000; // Intervallo di 5 secondi tra invii di comandi
unsigned long lastSensorRead = 0;
const long sensorInterval = 6000; // Intervallo di lettura dei dati del sensore
int command = 2;
float mediaX = 0, mediaY = 0, mediaZ = 0;



SoftwareSerial espSerial(7,6); // RX, TX: uso i pin 7 e 6 per comunicare con ESP32-CAM

//FUNZIONI DI CONFIGURAZIONE MPU6050 ( ACCELLEROMETRO)
Adafruit_MPU6050 mpu;//classe sensore

void I2CScanner(){ //Scanner I2C
  byte error, address;
  int nDevices;
  Serial.println("Scansione...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(" E'stato trovato un dispositivo I2C connesso all' indirizzo: 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Errore sconosciuto all indirizzo 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println(" Non è stato trovato alcun dipositivo I2C collegato.\n");
    
  }
  else {
    Serial.println("Connesso");
  }
  delay(5000);
  
}
void StartMPU(){  // avvio sensore
   while (!Serial)
    delay(10); 

  Serial.println(" inizializzazzione Sensore Adafruit MPU6050 ");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Impossibile trovare MPU6050 ");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 ONLINE!");
}
void SettingsMPU(){  //settaggi sensore

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}
void MeasureAcceleration(){ //misurazione della varizione di accelerazione
  // Leggi i dati dell'accelerometro dal MPU6050 ogni 1 secondo
    if (millis() - lastSensorRead >= sensorInterval) {
        lastSensorRead = millis();
        
        // Leggi i dati del sensore
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
      for(int i = 0;i<N;i++){
        Accelerazione[i][0]=a.acceleration.x;
        Accelerazione[i][1]=a.acceleration.y;
        Accelerazione[i][2]=a.acceleration.z;
         mediaX += Accelerazione[i][0];
         mediaY += Accelerazione[i][1];
         mediaZ += Accelerazione[i][2];
      }
      mediaX /= N;
      mediaY /= N;
      mediaZ /= N;

      for(int i = 0; i<M;i++){
        if((Accelerazione[i][0] - mediaX) <= -G || (Accelerazione[i][1] - mediaY) <= -G || (Accelerazione[i][2] - mediaZ) <= -2*G){
          // La variazione è troppo repentina, fai qualcosa
          Serial.println("Variazione di accelerazione troppo repentina rilevata!");
          command = 1;
          break;  // Esci dal ciclo se trovi una variazione significativa
        }
      }
      A_X = int(mediaX);
      A_Y = int(mediaY);
      A_Z = int(mediaZ);
    }

    // Breve pausa per evitare che le operazioni siano troppo frequenti
    delay(10); 
}
void setup() {
    Serial.begin(9600); // Comunicazione con PC
    espSerial.begin(9600); // Comunicazione con ESP32-CAM
    delay(3000); // Breve ritardo per stabilizzare la connessione
    Serial.println("Arduino Master Ready");
    StartMPU();
    SettingsMPU();
     // Attendi il segnale di WiFi connesso dall'ESP32
    Serial.println("In attesa del segnale WiFi connesso dalla ESP32...");
    while (true) {
        if (espSerial.available() > 0) {
            int response = espSerial.read();  // Leggi la risposta

            if (response == 3) {
                // Se riceviamo il segnale che il WiFi è connesso
                Serial.println("WiFi connesso! Stampando DNS...");
                
                Serial.println("DNS: http://DASHCAM32_V2.local/"); // Stampa il DNS noto al master (puoi modificare questo valore)
                delay(3000);
                break;  // Esci dal ciclo dopo aver ricevuto il segnale
            }else{
              // Non abbiamo ricevuto il segnale atteso, comunichiamo allo slave di ritentare
              Serial.println(".");
              espSerial.write(4);  // Invia comando "ritenta" (codice 4)
              delay(100);  // Breve ritardo per dare allo slave il tempo di elaborare
            }
        }
        delay(10);      
    }
    
}
void loop() { 

  Serial.flush();
  // Invia il comando alla ESP32 ogni espInterval millisecondi
    if (millis() - lastCommandSent >= espInterval) {
        lastCommandSent = millis();
        if (command == 1) {
            espSerial.write(command);  // Invia comando "TAKE_PICTURE" (codice 1)
            Serial.println("Comando inviato a ESP32: TAKE_PICTURE");
        } else {
            espSerial.write(command);  // Invia comando "STOP" (codice 2)
            Serial.println("Comando inviato a ESP32: STOP");
        }
        command = 2;
    }

    Serial.flush();
    // Controlla la risposta dalla ESP32
    if (espSerial.available() > 0) {
        delay(50);
        int response = espSerial.read();  // Leggi la risposta

        switch (response) {
            case 100:
                Serial.println("ACK ricevuto correttamente per TAKE_PICTURE");
                break;
            case 101:
                Serial.flush();
                Serial.println("ACK ricevuto correttamente per STOP");
                // Aspettati di ricevere i valori di accelerazione subito dopo
                if (espSerial.available() >= 3) { // Verifica se ci sono almeno 3 byte disponibili per lettura
                    espSerial.write(A_X);
                    espSerial.write(A_Y);
                    espSerial.write(A_Z);
                    Serial.print("Accelerazione ricevuta - X: ");
                    Serial.print(A_X);
                    Serial.print(", Y: ");
                    Serial.print(A_Y);
                    Serial.print(", Z: ");
                    Serial.println(A_Z);
                }
                break;
            case 255:
                Serial.println("Errore: comando non riconosciuto dalla ESP32");
                break;
            case 3:
                Serial.println("connessione con esp32, stabilita correttamente!");
                command = 2; // connessione stabilita correttamente!!
                break;
            default:
                Serial.println("Errore: risposta non valida (valore ricevuto:");
                Serial.print(response);
                Serial.println(")");
                break;
        }
    }

    delay(25); // Breve pausa per evitare loop troppo veloci
    
    MeasureAcceleration();
    delay(10);
}

