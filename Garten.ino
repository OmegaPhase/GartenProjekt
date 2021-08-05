//Bibliotheken(Board: ESP Dev module)
//Temperatur
#include <DHT.h>
#include <DHT_U.h>

//LED STRIP
#include <Adafruit_NeoPixel.h>

//WEB PROTOKOLL

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

//SERVO

//#include <Servo.h>    //Auskomentiert, da sie anscheinend fehlermeldungen erzeugt hat




//WERTE setzen(defines)

//MOTOR(Pin belegung)

#define pinMotor1Geschwindigkeit  14   //Geschwindigkeit
#define in1  12
#define in2  13
#define pinMotor2Geschwindigkeit  21   //Geschwindigkeit
#define in3  18
#define in4  19

//Servo
#define servopin 2

//LEDSTRIP
#define LEDPIN 15
#define NUMPIXELS 12
Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_RGB + NEO_KHZ800);


//SENSOREN

//WASSERSENSOR
#define pinTemperaturSensor 4

//LICHTSENSOR
#define pinLichtSensor 35

//WASSERSENSOR
#define pinWasserSensor 27

//Temperatursensor
#define DHTPIN 4
#define DHTTYPE DHT11

//DHT dht(DHTPIN, DHTTYPE);     //auskomentiert, da es die timings der Wifi und MQTT bibliothek stört


//W-lan informationen

// SSID/Password combination
const char* ssid = "SICK Summer University";
const char* password = "SSU2021!";

// MQTT Broker IP address:
const char* mqtt_server = "192.168.8.11";


//WERTE FÜR MSQTT(Objekte)
WiFiClient espClient;
PubSubClient client(espClient);

//long lastMsg = 0;         //habe ich einfach aus dem Beispiel übernommen sollte aber unnötig sein
//char msg[50];



//Zahlen für die automatische Steuerung

int duengerKonzentration = 0;     //Wert zwischen 0 und 50 in Prozent

bool automatischBewaessern = false;

bool automatischBeleuchten = false;




//BASISMETHODEN

int HexToInt(String ss)
{
  char arr[2];
  arr[0] = ss[0];
  arr[1] = ss[1];
  return(int(strtol(arr, 0, 16)));
}






//SENSOR AUSLESE METHODEN

int prozentWasserSensor() 
{

  double Eingangwat = analogRead(pinWasserSensor);

  double Prozent = (Eingangwat / 4095) * 100;

  return Prozent;
}

int prozentLichtSensor() 
{
  /*                                //Programm um den maximal und minimalWert des Sensors herauszufinden
     int mini = 4000;
    int MAxi = 0;
    int Count =0;
    int akt = (analogRead(pinLichtSensor));
    if (akt >= MAxi){
    MAxi = akt;
    }
    if(akt <= mini){
    mini = akt;
    }
    Count = Count + 1;

    Serial.println("MINI"+String(mini));
    Serial.println("MAx"+String(MAxi));

  */
  Serial.println(analogRead(pinLichtSensor));     //DEBUG
  
  double minimumLichtSensor = 0;   //Bei Dunkelheit

  double maximumLichtSensor = 3000;   //Bei Helligkeit

  double wertLichtSensor = analogRead(pinLichtSensor);

  double prozent = (wertLichtSensor / maximumLichtSensor) * 100;

  //prozent = (Prozent * -1) + 100;       //invertierung wird nicht gebraucht

  return prozent;

}

int prozentTemperaturSensor()  //nicht implementiert
{
  int temp = 23;
  
  //temp = dht.readTemperature();
  
  
  return temp;
}








//AKTOR SENDE METHODEN

//LED:
void LedStrip() {
  //pixels.clear();
  pixels.fill(pixels.Color(0, 0, 0));
  pixels.fill(pixels.Color(0, 0, 0));
  //pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
}

void setLedColor(String Eingang)   //Format HEX showed direct
{
  int red = 0;
  int green = 0;
  int blue = 0;

  red = HexToInt(Eingang.substring(0, 2));
  green = HexToInt(Eingang.substring(2, 4));
  blue = HexToInt(Eingang.substring(4, 6));

  pixels.fill(pixels.Color(green, red, blue));

  
  pixels.show();
  Serial.println(red);
  Serial.println(green);
  Serial.println(blue);
}



//SERVO
void setServoPos(int pos) {           //Methode funktioniert nicht, da die Servo bibliothek anscheinend nicht funktioniert
  pos = 0;
  //Servo myservo;

  for (pos = 0; pos <= 180; pos += 1) {

    // myservo.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    // myservo.write(pos);
    delay(15);
  }
}



//MOTOR
/*        //alter Motor control code
void setMotor(int speedm1, int speedm2, bool M1dir, bool M2dir)   //True ist vorwaertz, False rueckwaerts     //soll ersetzt werden durch setMotorSpeed und setMotorDirection
{
  //M1#####################################################
  if (M1dir == 0) {   //FALSE
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {              //TRUE
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  //M2#####################################################
  if (M2dir == 0) { //FALSE
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {            //TRUE
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  //MOTORspeed
  
  sigmaDeltaWrite(0, speedm1);
  sigmaDeltaWrite(1, speedm2);
}
*/
void setMotorSpeed(int motor, int motorSpeed)     //motor muss entweder 1 oder 2 sein, motorspeed ein Wert zwischen 0 und 255
{
  if(motor == 1)  //MOTOR1
  {
    sigmaDeltaWrite(0, motorSpeed);
    
  }
  if(motor == 2)  //MOTOR2
  {
    sigmaDeltaWrite(1, motorSpeed);
    
  }
}

void setMotorDirection(int motor, int motorDirection) //motor muss entweder 1 oder 2 sein, motorDirection ein Wert über 0(vorwärts), gleich 0(stehen), unter 0(rückwärts)
{
  if(motor == 1)
  {
    if(motorDirection>0)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    if(motorDirection==0)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
    }
    if(motorDirection<0)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
  }
  if(motor == 2)
  {
    if(motorDirection>0)
    {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }
    if(motorDirection==0)
    {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, HIGH);
    }
    if(motorDirection<0)
    {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
  }
}



//WLAN

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}


//MQTT

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TestESP")) {
      Serial.println("connected");
      client.subscribe("garden/light/color");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}






//Ansteuerung Wasser

void bewaessern(int TimeMilli, int konzentration)//konzentration: 0-50%
{
  int requiredSpeed = (double(2.55)*(konzentration*2));


  setMotorSpeed(1,255);
  setMotorSpeed(2,requiredSpeed);

  setMotorDirection(1,1);   //beide Motoren vorwärts
  setMotorDirection(2,1);
  
  delay(TimeMilli);

  setMotorSpeed(1,0);
  setMotorSpeed(2,0);

  setMotorDirection(1,1);   //beide Motoren vorwärts
  setMotorDirection(2,1);
}




//AUTOMATISCHE AUFRUFE





//automatisch BEWÄSSERN

int letzteBewaesserungsZeit = 0;

void autoBewaessern()
{
  if(letzteBewaesserungsZeit + 120000 > millis() && prozentWasserSensor() < 10)
  {
    letzteBewaesserungsZeit = millis();
    
    bewaessern(5000,duengerKonzentration);
  }
}


//automatisch LICHT

void autoLicht()
{
  double aktuelleHelligkeit = (prozentLichtSensor() * -1) + 100;
  
  int zwischenWert = (aktuelleHelligkeit * 2.55);

  String dreiMalWertHex = String(zwischenWert, HEX);

  dreiMalWertHex = dreiMalWertHex + String(zwischenWert, HEX);

  dreiMalWertHex = dreiMalWertHex + String(zwischenWert, HEX);
  
  setLedColor(dreiMalWertHex);
}


//ruft automationsmethoden wenn diese gerufen werden sollen

void triggerAutomation()
{
  if(automatischBewaessern)
  {
    autoBewaessern();
  }
  if(automatischBeleuchten)
  {
    autoLicht();
  }
}







void sendInfo()   //Wasser Temperatur und Licht muss ausgelesen und gesendet werden
{
  String StrWasser = String(prozentWasserSensor());
  String StrLicht = String(prozentLichtSensor());
  String StrTemperatur = String(prozentTemperaturSensor());

  char chWasser[StrWasser.length()];
  char chLicht[StrLicht.length()];
  char chTemperatur[StrTemperatur.length()];

  StrWasser.toCharArray(chWasser,StrWasser.length());
  StrLicht.toCharArray(chLicht,StrLicht.length());
  StrTemperatur.toCharArray(chTemperatur,StrTemperatur.length());

  Serial.println("Wasser:");
  Serial.println(StrWasser);
  Serial.println("Licht:");
  Serial.println(StrLicht);
  Serial.println("Temperatur:");
  Serial.println(StrTemperatur);

  
  client.publish("garden/wetness", chWasser);
  client.publish("garden/light", chLicht);
  client.publish("garden/temperature", chTemperatur);
}






void callback(char* topic, byte* message, unsigned int length) {        //Wenn Nachrich vom MQTT Brooker gesendet wurde       //über alle topics neu drübergucken
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println("");


  //manuelle Steuerung

    
  //LED
  if (String(topic) == "garden/light/manual")  //geändert    
  {
    setLedColor(messageTemp);
  }

  //Bewaesserung

  if (String(topic) == "garden/water/manual")  //erwartet eine Zeit in sekunden
  {
    bewaessern((messageTemp.toInt()*1000), duengerKonzentration);
  }




  //toggle auto an/aus
  //LED
  if (String(topic) == "garden/light/auto")
  {
    if(messageTemp == "True")               //Ich weiß nicht wie das true von NodeRed aussieht
    {
      automatischBeleuchten = true;
    }
    if(messageTemp == "False")
    {
      automatischBeleuchten = false;
    }
  }
  //WASSER
  if (String(topic) == "garden/water/auto")
  {
    if(messageTemp == "True")               //Ich weiß nicht wie das true von NodeRed aussieht
    {
      automatischBewaessern = true;
    }
    if(messageTemp == "False")
    {
      automatischBewaessern = false;
    }
  }

  //festlegen der konzentration
  
  if(String(topic) == "garden/water/concentration")       //erwarteter Wert liegt zwischen inklusive 0-50
  {
    duengerKonzentration = messageTemp.toInt();
  }
  
  
}










void setup() {
  //SERVO
  
  //myservo.attach(servopin);
  
  //SENSOREN
  pinMode(pinLichtSensor, INPUT);
  pinMode(pinTemperaturSensor, INPUT);
  pinMode(pinWasserSensor, INPUT);

  //DEBUG
  pinMode(34,INPUT_PULLUP);

  //Sigma(analog write auf esp32)

  //SETUP FÜR ANALOG WRITE FÜR MOTOREN
  sigmaDeltaSetup(0, 312500);
  sigmaDeltaSetup(1, 312500);
  sigmaDeltaAttachPin(pinMotor1Geschwindigkeit, 0);
  sigmaDeltaAttachPin(pinMotor2Geschwindigkeit, 1);
  
  //MOTOR
  pinMode(pinMotor1Geschwindigkeit, OUTPUT);      //Motor 1
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pinMotor2Geschwindigkeit, OUTPUT);      //Motor 2
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  //LED
  pixels.begin();
  
  //Serial
  Serial.begin(9600);

  //MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //TEMPERATURSENSOR (beginne Messung)
  //dht.begin();
}














int timeLastSendMQTT  = 0;

int timeLastExecutedAutomationCheck = 0;

void loop() {


  //reconect für das WLAN
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  //für den callback methoden check
  



  //sendinfo loop (event trigerd alle 2,5 sec)
  
  if(millis()>timeLastSendMQTT + 2500)
  {
    sendInfo();
    timeLastSendMQTT = millis();
  }

  //Automations loop (event trigerd alle 5 sec)
  
  if(millis()>timeLastExecutedAutomationCheck + 5000)
  {
    triggerAutomation();
    timeLastExecutedAutomationCheck = millis();
  }



  //debug für motoren
  if(digitalRead(34)==LOW)
  {
  setMotorSpeed(1,255);
  setMotorSpeed(2,255);

  setMotorDirection(1,1);   //beide Motoren vorwärts
  setMotorDirection(2,1);
  }
  else
  {
  setMotorSpeed(1,0);
  setMotorSpeed(2,0);

  setMotorDirection(1,1);   //beide Motoren vorwärts
  setMotorDirection(2,1);
  }






  //Serial.println(analogRead(35));
  //client.publish("test", "Hallo Welt");
  //LedStrip();
  //setServoPos();
  //Serial.println(prozentWasserSensor());
}
