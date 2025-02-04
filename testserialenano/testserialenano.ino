 #include <SoftwareSerial.h> 

 SoftwareSerial espSerial(7,6); // RX, TX: uso i pin 7 e 6 per comunicare con ESP32-CAM

struct mydata{
  int32_t AccelerazioneX;
  int32_t AccelerazioneY;
  int32_t AccelerazioneZ;
  int32_t PBatteria;
};
mydata dato = {2,3,5,100};
void setup() {
  Serial.begin(9600); //Inizializziamo la comunicazione seriale
  espSerial.begin(9600);
}

void loop() {
   
      espSerial.write((const uint8_t *)&dato , sizeof(struct mydata));
      //espSerial.write((byte)'\n');
      dato.PBatteria--;
      delay(1000);
}

