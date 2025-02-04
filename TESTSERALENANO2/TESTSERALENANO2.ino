#include <SoftwareSerial.h>

// Configurazione delle porte per la SoftwareSerial
SoftwareSerial mySerial(7, 6);  // RX, TX (D7 = RX, D6 = TX)

// Definizione dei comandi
#define COMMAND_ACTION 0xA1  // Comando per una determinata azione
#define COMMAND_IDLE 0xB0    // Comando di default (idle)

// Funzione per inviare un comando con dati
void sendCommand(byte command, int data) {
  byte startByte = 0x02;  // Byte di inizio (STX)
  byte endByte = 0x03;    // Byte di fine (ETX)
  
  // Iniziamo a inviare il pacchetto
  mySerial.write(startByte);    // Invia il byte di inizio
  mySerial.write(command);      // Invia il comando
  
  // Convertiamo l'int in due byte e li inviamo
  mySerial.write((byte)(data >> 8));  // Invia byte alto del dato
  mySerial.write((byte)(data & 0xFF)); // Invia byte basso del dato
  
  mySerial.write(endByte);      // Invia il byte di fine
}

void setup() {
  // Inizializza la SoftwareSerial
  mySerial.begin(9600);

  // Inizializza la seriale hardware per il debug
  Serial.begin(9600);

  Serial.println("Master Arduino Nano pronto per inviare comandi.");
}

void loop() {
  // Esempio di invio di un comando per un'azione specifica
  int sensorValue = analogRead(A0);  // Legge un valore da un sensore
  
  // Invia un comando di azione con il valore del sensore
  sendCommand(COMMAND_ACTION, sensorValue);
  Serial.print("Comando di azione inviato con valore sensore: ");
  Serial.println(sensorValue);
  
  delay(1000);  // Aspetta 1 secondo prima di inviare il prossimo comando
  
  // Invia un comando di default (idle)
  sendCommand(COMMAND_IDLE, 0);
  Serial.println("Comando di default inviato (idle).");
  
  delay(1000);  // Aspetta 1 secondo prima di inviare il prossimo comando
}
