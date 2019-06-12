//Legger til biblioteket for enheter som tar bruk av SPI data protokollen (I dette tilfellet RFID skanneren). 
//#include <SPI.h> 
//Legger til biblioteket for RFID skanneren av typen MFRC522 
#include <MFRC522.h>  
//Legger til biblioteket for MP3 spiller koden 
#include <SoftwareSerial.h>
//Definerer hvilken pin RX på MP3 spilleren er koblet til 
#define ARDUINO_RX 5
//Definerer hvilken pin TX på MP3 spilleren er koblet til 
#define ARDUINO_TX 6
//Oppretter et SoftwareSerial objekt slik at vi kan sende og motta data fra MP3 spilleren 
SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX); 
//Definerer kommandoen for å spille av sang.  
#define CMD_PLAY_W_INDEX 0X03   
//Definerer kommando for å spille av en valgt sang i en valgt mappe
#define CMD_PLAY_WITHFOLDER 0X0F//DATA IS NEEDED, 0x7E 06 0F 00 01 02 EF;(play the song with the directory \01\002xxxxxx.mp3
//En statisk variabel som skal inneholde en 8 int string som sendes til mp3 spilleren 
static int8_t Send_buf[8] = {0} ;
//Definerer hvilken pin RST (Reset) på RFID skanneren er koblet til 
#define RST_PIN         9         
//Definerer hvilken pin SS(Slave Select) på RFID skanneren er koblet til 
#define SS_PIN          10      

MFRC522 mfrc522(SS_PIN, RST_PIN);  //Oppretter et mfrc522 objekt 
//knappA med informasjon om utseende 
int knappA = 5; 
//knappB med informasjon om vasking 
int knappB = 4;
//En debounce på 2/10 av et sekund 
int debounce = 200;
//Variabler som holder styr på hvor lang tid det har tatt siden siste knapp ble trykket på 
unsigned long sisteATrykk = 0; 
unsigned long sisteBTrykk = 0;

void setup() {  
  //Definerer knappene som input 
  pinMode(knappA,INPUT);
  pinMode(knappB,INPUT);   
  mySerial.begin(9600);
  Serial.begin(9600);  
  while (!Serial);   
  SPI.begin();      //Initialiserer SPI bussen 
  mfrc522.PCD_Init();   // Initialiserer MFRC522  
  sendCommand(0x03,1);
}  
//sendCommand tar inn en kommando og data som den sender videre til MP3 spilleren. Fant ut at det var lurt aa bruke denne metoden ved aa soeke etter eksempelkode til bruk av MP3 serial monitor. https://create.arduino.cc/projecthub/javier-munoz-saez/arduino-mp3-player-catalex-2effef
void sendCommand(int8_t command, int16_t dat){
 delay(20); 
 //Det første bytet er konstant 
 Send_buf[0] = 0x7e; 
 //Versjonen 
 Send_buf[1] = 0xff; 
 //Nummeret av bytes i hele kommandoen utenom bytene på starten og slutten. 
 Send_buf[2] = 0x06;  
 //Selve kommandoen 
 Send_buf[3] = command;  
 //Om det skal være feedback eller ikke
 Send_buf[4] = 0x00; 
 //Bytes som inneholder hvilken lydfil som er valgt 
 Send_buf[5] = (int8_t)(dat >> 8);//datah
 Send_buf[6] = (int8_t)(dat); //datal 
 //Det siste bytet er konstant
 Send_buf[7] = 0xef; //ending byte 
 //Gaar gjennom hvert byte og sender det til MP3 spilleren 
 for(uint8_t i=0; i<8; i++)//
 {
   mySerial.write(Send_buf[i]) ;
 } 
 //Metode for aa sjekke om en knapp blir trykket 
}
boolean isClicked(int knapp){ 
  //Sjekker om knapp a er trykket 
  if(knapp == knappA){ 
    //Gaar bare videre dersom tiden siden siste trykk er stoerre enn debounce 
    if((millis() - sisteATrykk) >= debounce){ 
      //Dersom knappen blir trykket paa returneres true 
      if (digitalRead(knapp)){
        sisteATrykk = millis();
        return true; 
      }
    } 
  }   
  //Sjekker om knapp a er trykket 
  if(knapp == knappB){ 
    //Gaar bare videre dersom tiden siden siste trykk er stoerre enn debounce 
    if((millis() - sisteBTrykk) >= debounce){ 
      //Dersom knappen blir trykket paa returneres true 
      if (digitalRead(knapp)){ 
        sisteBTrykk = millis();
        return true; 
      }
    } 
  }
  return false;
}  

  
  //Variabel som sjekker om det sistregistrerte kortet allerede har informasjon som har blitt lest opp eller ikke 
boolean igjen = false; 
//Variabel som skal holde styr på det registrerte kortet sin ID 
unsigned long hex_number = 0;  
//Det var problemer med aa faa loopen til aa gaa mer enn en gang dersom det aldri kom et nytt kort. Derfor er det en egen metode for dette 
void fremdelesSammePlagg(){  
  //Booleans som blir true dersom brukeren velger den tilsvarende knappen  
  boolean vaskeInfo = false; 
  boolean utseendeInfo = false; 
  Serial.print("Leste andre kort");   
      //Definerer booleans som sjekker om knappene blir trykket paa 
      boolean aTrykket = isClicked(knappA); 
      boolean bTrykket = isClicked(knappB);  
      //Mens ikke noe er trykket paa kjoerer en while loekke 
      while(!bTrykket && !aTrykket){  
        //Dersom det kommer et nytt kort fortsetter loop();
        if(mfrc522.PICC_IsNewCardPresent()){ 
          //Dersom det kommer et nytt kort blir igjen false 
          igjen = false;  
          return;
        } 
        //Sjekker om knappene har blitt trykket paa
       aTrykket = isClicked(knappA); 
       bTrykket = isClicked(knappB); 
      }    
      Serial.print("Trykket");   
      //Lagrede IDer 
      unsigned long hex_num1 = 4294955139; 
      unsigned long hex_num2 = 28534;  
      unsigned long hex_num3 = 29338;
      unsigned long hex_num4 = 29594;
      //Dersom a ble trykket skal informasjon om utseende bli lest opp
      if(aTrykket){utseendeInfo = true;} 
      //Dersom b ble trykket skal informasjon om vasking bli lest opp
      if(bTrykket){vaskeInfo = true;} 
      //Dersom IDen er lik IDen til det foerste plagget 
      if(hex_number == hex_num1){ 
        //Dersom informasjon om utseende ble valgt
         if(utseendeInfo){ 
            sendCommand(0x03,3);
         } else if(vaskeInfo){  
            sendCommand(0x03,2);
         }
      }  
      //Dersom IDen er lik IDen til det andre plagget 
      if(hex_number == hex_num2){  
        //Dersom informasjon om utseende ble valgt
        if(utseendeInfo){
          sendCommand(0x03,4);
          //Dersom informasjon om vasking ble valgt
        } else if(vaskeInfo){  
          sendCommand(0x03,5);
        }
      } 
    if(hex_number == hex_num3){  
        //Dersom informasjon om utseende ble valgt
        if(utseendeInfo){
          sendCommand(0x03,8); 
          //Dersom informasjon om vasking ble valgt
        } else if(vaskeInfo){ 
          sendCommand(0x03,6);
        }
      } 
     if(hex_number == hex_num4){  
        //Dersom informasjon om utseende ble valgt
        if(utseendeInfo){
          sendCommand(0x03,8); 
          //Dersom informasjon om vasking ble valgt
        } else if(vaskeInfo){ 
          sendCommand(0x03,7);
        }
      }
    
} 

void loop() {
  // Dersom det ikke kommer et nytt plagg skjer ingenting 
   
  if ( ! mfrc522.PICC_IsNewCardPresent()) { 
    //Dersom et plagg allerede har informasjon som har blitt lest opp
    if(igjen){
      fremdelesSammePlagg();
    }
    return;
  }    
  //Lyd som signaliserer at et plagg ble registrert 
  sendCommand(0x03,1);
  Serial.print("Fant nytt kort"); 
  //Finner IDen ved hjelp av metoden getIDen
  unsigned long uid = getIDen();     
  Serial.print(uid);
  //Dersom uiden er -1 betyr det at kortet ikke blir lest ordentlig 
  if(uid != -1){ 
    Serial.print("Card detected, UID: "); Serial.println(uid);  
    //Lagrede IDer
    unsigned long hex_num1 = 4294955139; 
    unsigned long hex_num2 = 28534;  
    unsigned long hex_num3 = 29338;
    unsigned long hex_num4 = 29594;
     //Definerer booleans som sjekker om knappene blir trykket paa 
    boolean aTrykket = isClicked(knappA); 
    boolean bTrykket = isClicked(knappB); 
     //Venter til en knapp blir trykket paa
    while(!bTrykket && !aTrykket){  
      //Dersom det dukker opp et annett kort starter loopen paa nytt
      if(mfrc522.PICC_IsNewCardPresent()){ 
        return;
        } 
     //Sjekker om knappene har blitt trykket paa
     aTrykket = isClicked(knappA); 
     bTrykket = isClicked(knappB); 
    }    
    boolean vaskeInfo = false; 
    boolean utseendeInfo = false;  
      //Dersom a ble trykket skal informasjon om utseende bli lest opp
    if(aTrykket){utseendeInfo = true;} 
     //Dersom b ble trykket skal informasjon om vasking bli lest opp
    if(bTrykket){vaskeInfo = true;}
        if(uid == hex_num1){ 
          //Dersom informasjon om utseende ble valgt
           if(utseendeInfo){
              //Spiller av lydklipp for dette plagget og denne typen informasjon 
              sendCommand(0x03,2);  
            //Dersom informasjon om vasking ble valgt
           } else if(vaskeInfo){    
            //Spiller av lydklipp for dette plagget og denne typen informasjon 
              sendCommand(0x03,3);
           } 
           
        }  
        //Dersom IDen er lik IDen til det andre plagget 
        if(uid == hex_num2){  
          //Dersom informasjon om utseende ble valgt
          if(utseendeInfo){  
            //Spiller av lydklipp for dette plagget og denne typen informasjon 
            sendCommand(0x03,4); 
          //Dersom informasjon om vasking ble valgt
          } else if(vaskeInfo){
            //Spiller av lydklipp for dette plagget og denne typen informasjon 
            sendCommand(0x03,5);
          }
        } 
       if(hex_number == hex_num3){  
        //Dersom informasjon om utseende ble valgt
        if(utseendeInfo){
          sendCommand(0x03,8);  
          //Dersom informasjon om vasking ble valgt
        } else if(vaskeInfo){ 
          sendCommand(0x03,6);
        }
      }  
     if(hex_number == hex_num4){  
        //Dersom informasjon om utseende ble valgt
        if(utseendeInfo){
          sendCommand(0x03,8); 
          //Dersom informasjon om vasking ble valgt
        } else if(vaskeInfo){ 
          sendCommand(0x03,7);
        }
      }  
        //Lagrer IDen slik at 
        hex_number = uid;  
        //Naar informasjon har blitt lest opp er igjen true 
        igjen = true;
  }

 
} 

//Metode som sjekker og returnerer ID  
//Denne metoden fant vi paa nettet. https://stackoverflow.com/questions/32839396/how-to-get-the-uid-of-rfid-in-arduino
unsigned long getIDen(){ 
  //Forsoeker aa lese av informasjon om kortet 
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return -1;
  } 
  unsigned long hex_num; 
  //Finner alle bytene i UIDen 
  hex_num =  mfrc522.uid.uidByte[0] << 24;
  hex_num += mfrc522.uid.uidByte[1] << 16;
  hex_num += mfrc522.uid.uidByte[2] <<  8;
  hex_num += mfrc522.uid.uidByte[3];
  //Avslutter lesingen 
  mfrc522.PICC_HaltA(); 
  return hex_num;
} 
