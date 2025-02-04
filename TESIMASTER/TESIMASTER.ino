#include <SoftwareSerial.h> //libreria che gestisce la comunicazione seriale tra il master (arduino nano) e lo slave (esp 32 cam)
#include <Adafruit_GFX.h>     // accellereometro
#include <Adafruit_MPU6050.h> // accellereometro
#include <Adafruit_Sensor.h>  // accellerometro
#include <Wire.h> //I2C
// DEFINIZIONI
#define N 5
#define M 3
#define G 9.81
#define BATTERY_PIN A0

//VARIABILI
float Accelerazione[M][N];//accelerometro
unsigned long tempoUltimaLettura = 0;
unsigned long IntervalloSensore = 6000;
float mediaX = 0; 
float mediaY = 0; 
float mediaZ = 0;
int rawValue = 0;
//batteria
float voltage = 0;
float batteryVoltage = 0;
float batteryPercentage = 0;
const float R1 = 10000; // Resistenza 1 (in Ohm)
const float R2 = 7000; // Resistenza 2 (in Ohm)
const float ADC_MAX = 1023.0; // Valore massimo del ADC (10-bit)
const float VREF = 4.46; // Riferimento di tensione del Nano (misurato con il multimetro)
// Intervallo di tensione delle batterie 18650 da 3,7 V poste in parallelo (3 batterie)
const float V_full = 12.6;   // Tensione a batteria piena
const float V_empty = 9.00;  // Tensione a batteria scarica
//comunicazione seriale
struct mydata{
  int32_t AccelerazioneX;
  int32_t AccelerazioneY;
  int32_t AccelerazioneZ;
  int32_t PBatteria;
  int32_t Comando;
};
mydata dato = {0,0,0,0,2};
int response = 0;
unsigned long lastCommandSent = 0;
unsigned long espInterval = 5000;


 SoftwareSerial espSerial(7,6); // RX, TX: uso i pin 7 e 6 per comunicare con ESP32-CAM
 //FUNZIONI DI CONFIGURAZIONE MPU6050 ( ACCELLEROMETRO)
Adafruit_MPU6050 mpu;//classe sensore

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
    if (millis() - tempoUltimaLettura >= IntervalloSensore) {
         tempoUltimaLettura = millis();
        
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
          dato.Comando = 1;
          break;  // Esci dal ciclo se trovi una variazione significativa
        }
      }
      dato.AccelerazioneX= int32_t(mediaX);
      dato.AccelerazioneY= int32_t(mediaY);
      dato.AccelerazioneZ= int32_t(mediaZ);
    }

    // Breve pausa per evitare che le operazioni siano troppo frequenti
    delay(10); 
}
void ReadChargeBattery(){
 // Lettura del valore analogico dal pin della batteria
  rawValue = analogRead(BATTERY_PIN);

  // Converti il valore ADC in tensione
  voltage = (rawValue / ADC_MAX) * VREF;

  // Calcola la tensione reale della batteria (usando il rapporto del partitore di tensione)
   batteryVoltage = voltage * (R1 + R2) / R2;

  // Calcola la percentuale di carica della batteria
  batteryPercentage = (batteryVoltage - V_empty) / (V_full - V_empty) * 100;

  // Assicuriamoci che il valore della percentuale rimanga tra 0% e 100%
  if (batteryPercentage > 100) batteryPercentage = 100;
  if (batteryPercentage < 0) batteryPercentage = 0;

  // Stampa la tensione della batteria e la percentuale su Serial Monitor
  Serial.print("Tensione batteria: ");
  Serial.print(batteryVoltage);
  Serial.print(" V, Percentuale di carica: ");
  Serial.print(batteryPercentage);
  Serial.println(" %");
  dato.PBatteria= int32_t(batteryPercentage);

}
void onlineAP(){
   while (true) {
        if (espSerial.available() > 0) {
            int response = espSerial.read();  // Leggi la risposta

            if (response == 3) {
                // Se riceviamo il segnale che il WiFi è connesso
                Serial.println("\nWiFi connesso! Stampando DNS...");
                
                Serial.println("\nIP STATICO: http://192.168.4.1/"); // Stampa l'IP statico noto al master
                delay(3000);
                break;  // Esci dal ciclo dopo aver ricevuto il segnale
            }else{
              // Non abbiamo ricevuto il segnale atteso, comunichiamo allo slave di ritentare
              Serial.print(".");
              espSerial.write(4);  // Invia comando "ritenta" (codice 4)
              delay(100);  // Breve ritardo per dare allo slave il tempo di elaborare
            }
        }
        delay(10); 
   }
}
void setup() {
  Serial.begin(9600); //Inizializziamo la comunicazione seriale
  espSerial.begin(9600);
  delay(3000); // Breve ritardo per stabilizzare la connessione
  //StartMPU();
  //SettingsMPU();
  // Attendi il segnale di WiFi connesso dall'ESP32
  //Serial.println("In attesa del segnale WiFi connesso dalla ESP32...");
  //onlineAP();
  
}
void loop() {
    
     //if (millis() - lastCommandSent >= espInterval) {
        //lastCommandSent = millis();
      espSerial.write((const uint8_t *)&dato , sizeof(struct mydata));
      //espSerial.write((byte)'\n');
      delay(1000);
    // }
    //MeasureAcceleration();
    //ReadChargeBattery();
    //dato.Comando = 2;
      
}

