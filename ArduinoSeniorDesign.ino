#include <Parser.h>
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <UDPServer.h>

#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER); 

#define NEWALBUM        1
#define POSITIONUPDATE  9
#define ATBEGINNING     10

#define ON              15
#define OFF             16

#define READY           21
#define PLAY            22
#define GOTOTRACK       23 
#define PAUSE           24
#define STOP            25
#define SCAN            26

#define NETGEAR

#ifdef NETGEAR
  #define WLAN_SSID       "NETGEAR84"        // cannot be longer than 32 characters!
  #define WLAN_PASS       "strongshrub599"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
  #define WLAN_SECURITY   WLAN_SEC_WPA2
#endif

#ifdef HOME
  #define WLAN_SSID       "ASUSPAT"        // cannot be longer than 32 characters!
  #define WLAN_PASS       "strongshrub"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
  #define WLAN_SECURITY   WLAN_SEC_WPA2
#endif

#ifdef SCHOOL
  #define WLAN_SSID       "UAGuest"        // cannot be longer than 32 characters!
  #define WLAN_PASS       ""
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
  #define WLAN_SECURITY   WLAN_SEC_UNSEC
#endif

uint sendAddress;

Adafruit_CC3000_Client client;

const unsigned long
  connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  responseTimeout = 15L * 1000L; // Max time to wait for data from server

uint32_t myIP;

#define UDP_READ_BUFFER_SIZE 150
#define LISTEN_PORT_UDP 30003

UDPServer udpServer = UDPServer(LISTEN_PORT_UDP);

Parser p = Parser();

bool IsPowerOn = false;

const int totPixels = 1536;
int IntArray[totPixels];
int voltPerUnit = .0049;

short si_1 = 13;
short CLKpin = 3;
short AOpin1 = A1;

const int Threshold = 1000;
const int ThresholdCount = 5;
const int intKeySize = 10;
int intKey[intKeySize];
const int byteKeySize = 20;
byte byteKey[byteKeySize];

//Need to set currentIndex upon getting a gototrack message
//Compare it with key values
int currentIndex= 0;
bool hasRecord = false;

void setup(void){
  Serial.begin(115200);
  
  ClearKey();
  
  byteKey[0] = 0;
  byteKey[2] = 0;
  byteKey[4] = 0;
  byteKey[6] = 0;
  byteKey[8] = 0;
  byteKey[1] = 10;
  byteKey[3] = 20;
  byteKey[5] = 30;
  byteKey[7] = 40;
  byteKey[9] = 50;
  
  intKey[0] = 10;
  intKey[1] = 20;
  intKey[2] = 30;
  intKey[3] = 40;
  intKey[4] = 50;
  
  analogWriteResolution(12);
    
  doSetup();
    
  udpServer.begin();
  
  sendUpdatePosition(1,4);
}

void loop(void) {
  
  //Check for UDP Messages
  readUDP();
  //Check to see if we are at a new song
  if(hasRecord){
    //Do an analog read for the magnet sensor
    int magnetValue = analogRead(A8);
    //Compare this value to byteKey[currentIndex] (needs scaling factor since we are comparing magnet to diode array)
    //If the key value is lower we have moved to next song
    //Else we are still at current song
    //Therefore we need to send an update position message for key[currentIndex]
    //Increment currentIndex
    if(magnetValue > intKey[currentIndex]){
      sendUpdatePosition(byteKey[currentIndex*2],byteKey[(currentIndex*2)+1]);
      currentIndex++;
    }
  }
}

///////////////////////////////////////////////////
/////////////////POWER FUNCTIONS///////////////////
///////////////////////////////////////////////////

//Set Power on or off
//Send a Power message based on result
void setPower(bool pwr){
   IsPowerOn = pwr;
   sendPowerMessage();
}

///////////////////////////////////////////////////
//////////////////MEDIA COMMANDS///////////////////
///////////////////////////////////////////////////

void moveMotorTo(int x){
  Serial.print("Moving To: ");Serial.println(x);
}

//Raise the tonearm
void play(){
  
}


void goToTrack(Parser::TrackMessage m){ 
  //Lift the Tonearm
  pause();
  //Serial.print("m.fByte"); Serial.println(m.fByte);
  //Serial.print("m.sByte"); Serial.println(m.sByte);
  for(int i =0; i< byteKeySize;i+=2){
    if(byteKey[i] == m.fByte && byteKey[i+1] == m.sByte){
      //Divide i by 2 to get the intKey Index
      //Then move motor to that location (use the offset)
      moveMotorTo(intKey[i/2]);
      //Drop the tonearm
      play();
      currentIndex = (i/2) + 1;
      delay(2500);
      sendUpdatePosition(m.fByte,m.sByte);
      break;
    } 
  }
}

//Bring the tonearm to the first song
void goToBeginning(){
  moveMotorTo(0);
  currentIndex = 0;
}

//Lift the tonearm
void pause(){
  
}

///////////////////////////////////////////////////
//////////////////SCAN FUNCTIONS///////////////////
///////////////////////////////////////////////////

void ClearKey(){
  for(int i = 0;i<byteKeySize;i++){
    byteKey[i] = 0;
  }
  for(int i = 0;i<intKeySize;i++){
    intKey[i] = 0;
  }
}

//Doesn't need a parameter simply use the diode array
byte* CreateKey(uint sourceIP, int* x){
  int iter = 0;
  int smallIter = 0;
  int partialValue = 0;
  int partialCount = 0;
  for(int i = 0; i<totPixels;i++){
    if(x[i] > 1000){
      partialValue += i;
      partialCount += 1;
    }
    else{
      //NOTE:This ignores end cases
      if(partialCount > 0){
        int totalValue = partialValue/partialCount;
        intKey[smallIter++] = totalValue;
        byte sByte = totalValue;
        byte fByte = totalValue>>8;
        byteKey[iter++] = fByte;
        byteKey[iter++] = sByte;
      }
      partialCount =0;
      partialValue = 0;
    }
  }
    
  delay(1000);
  sendNewRecord(sourceIP);
  delay(1000);
}

void ClockPulse()
{
  delayMicroseconds(1);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(CLKpin, LOW);
}

void sendPulse(int pin)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(12);
  digitalWrite(pin, LOW);
}

void PrintData()
{
  for(int i = 0; i < totPixels; i++)
  {
      int test = IntArray[i];
      Serial.println(test);
  } 
}

void initializeScan(){
  pinMode(si_1, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(AOpin1, INPUT);

  //set all of the digital pins low
  for( int i = 0; i < 14; i++ )
  {
      digitalWrite(i, LOW);  
  }
  
  // Clock out any existing SI pulse through the ccd register:
  for(int i = 0; i< totPixels; i++)
  {
      ClockPulse(); 
  }

  // Create a new SI pulse and clock out that same SI pulse through the sensor register:
  digitalWrite(si_1, HIGH);
  ClockPulse(); 
  digitalWrite(si_1, LOW);
  
  for(int i = 0; i < totPixels; i++)
  {
      ClockPulse(); 
  }
}

//Scan the record
//Write a newRecord message
void scan(uint sourceIP){
  
  ClearKey();
  
  digitalWrite(si_1, HIGH);
  ClockPulse();
  digitalWrite(si_1, LOW);
  
  for(int i=0; i < totPixels; i++)
  {
      //delayMicroseconds(20);// <-- We add a delay to stabilize the AO output from the sensor
      if(i < totPixels)
      {
        IntArray[i] = analogRead(AOpin1);
      }
      
      ClockPulse(); 
  }

  digitalWrite(si_1, HIGH);
  ClockPulse(); 
  digitalWrite(si_1, LOW);

  CreateKey(sourceIP,IntArray);

  //for(int i = 0; i < totPixels; i++)
  //{
      //Serial.println(IntArray[i]);
  //}
  
  //Serial.println("**************Done**************");

  for(int i = 0; i < totPixels; i++)
  {
      if(i==18)
      {
        
      }
  }    
  
  delay(1);// <-- Add 15 ms integration time
}

///////////////////////////////////////////////////
/////////////////SENDING MESSAGES//////////////////
///////////////////////////////////////////////////

//When the microcontroller performs a task it needs to let the applications know we are busy
//After the task is completed we let the apps know we are listening and ready
void sendStatusMessage(int ctrl){
  uint8_t buf[15];

  byte sIP[4] = {myIP >> 24, myIP >> 16, myIP >> 8, myIP};
  byte dIP[4] = {192, 168, 1, 255};
  byte cutoff[6] = {111, 111, 111, 111, 111, 111};
  
  int pointer = 0;
  
  do {
      client = cc3000.connectUDP(3232236031, 30003);
    } while(!client.connected());

    if(client.connected()) {
      
      memset(buf, 0, sizeof(buf));
      
      memcpy_P(buf, sIP, sizeof(sIP));
      pointer += sizeof(sIP);
      
      memcpy_P(&buf[pointer], dIP, sizeof(dIP));
      pointer += sizeof(dIP);
      
      buf[pointer] = ctrl;
      pointer += 1;
      
      memcpy_P(&buf[pointer], cutoff, sizeof(cutoff));
      pointer += sizeof(cutoff);
      
      client.write(buf, sizeof(buf));
  }
   client.close();
}

void sendPowerMessage(){
  uint8_t buf[15];

  byte sIP[4] = {myIP >> 24, myIP >> 16, myIP >> 8, myIP};
  byte dIP[4] = {192, 168, 1, 255};
  byte cutoff[6] = {111, 111, 111, 111, 111, 111};
  
  int pointer = 0;
  
  do {
      client = cc3000.connectUDP(3232236031, 30003);
    } while(!client.connected());

    if(client.connected()) {
      
      memset(buf, 0, sizeof(buf));
      
      memcpy_P(buf, sIP, sizeof(sIP));
      pointer += sizeof(sIP);
      
      memcpy_P(&buf[pointer], dIP, sizeof(dIP));
      pointer += sizeof(dIP);
      
      if(IsPowerOn){
         buf[pointer] = 15;
      }
      else{
        buf[pointer] = 16;
      }
      
      memcpy_P(&buf[pointer], cutoff, sizeof(cutoff));
      pointer += sizeof(cutoff);
      
      client.write(buf, sizeof(buf));
  }
   client.close();
}

void sendUpdatePosition(byte a, byte b){
  uint8_t buf[17];

  byte sIP[4] = {myIP >> 24, myIP >> 16, myIP >> 8, myIP};
  byte dIP[4] = {192, 168, 1, 255};
  byte cutoff[6] = {111, 111, 111, 111, 111, 111};
  
  int pointer = 0;
  
  do {
      client = cc3000.connectUDP(3232236031, 30003);
    } while(!client.connected());

    if(client.connected()) {
      
      memset(buf, 0, sizeof(buf));
      
      memcpy_P(buf, sIP, sizeof(sIP));
      pointer += sizeof(sIP);
      
      memcpy_P(&buf[pointer], dIP, sizeof(dIP));
      pointer += sizeof(dIP);
      
      buf[pointer++] = 9;
      
      memcpy_P(&buf[pointer], cutoff, sizeof(cutoff));
      pointer += sizeof(cutoff);
      
      buf[pointer++] = a;
      buf[pointer++] = b;
      
      client.write(buf, sizeof(buf));
  }
   client.close();
}

void sendNewRecord(uint destIP) {  
  //Size depends on id
  int realbyteKeySize = byteKeySize;
  for(int i =0; i<byteKeySize;i+=2){
    if(byteKey[i] == 0 && byteKey[i+1] == 0)
    {
      realbyteKeySize = i;
    }
  }
  
  uint8_t buf[21 + realbyteKeySize];
  
  byte sIP[4] = {myIP >> 24, myIP >> 16, myIP >> 8, myIP};
  byte dIP[4] = {destIP >> 24, destIP >> 16, destIP >> 8, destIP};
  byte cutoff[6] = {111, 111, 111, 111, 111, 111};
  
  int pointer = 0;
  
  do {
      client = cc3000.connectUDP(3232236031, 30003);
    } while(!client.connected());

    if(client.connected()) {
      
      memset(buf, 0, sizeof(buf));
      
      memcpy_P(buf, sIP, sizeof(sIP));
      pointer += sizeof(sIP);
      
      memcpy_P(&buf[pointer], dIP, sizeof(dIP));
      pointer += sizeof(dIP);
      
      buf[pointer] = NEWALBUM;
      pointer += 1;
      
      memcpy_P(&buf[pointer], cutoff, sizeof(cutoff));
      pointer += sizeof(cutoff);
      
      for(int i = 0;i<sizeof(realbyteKeySize);i+=2){
        buf[pointer++] = byteKey[i];
        buf[pointer++] = byteKey[i+1];
      }
      
      memcpy_P(&buf[pointer], cutoff, sizeof(cutoff));
      
      client.write(buf, sizeof(buf));
  }
   client.close();
}


///////////////////////////////////////////////////
////////////////////READING UDP////////////////////
///////////////////////////////////////////////////

void doCommand(char* m, Parser::Header header){
 switch(header.command){
   //Send a Message stating current status, which is free
    case 3:
      delay(2500);
      sendStatusMessage(READY);
      break;
    //Scan  
    case 4:
      delay(2500);
      sendStatusMessage(SCAN);
      delay(2500);
      scan(header.sourceIP);
      delay(2500);
      sendStatusMessage(READY);
      break;
    //Send a Message stating current power  
    case 6:
      delay(2500);
      sendPowerMessage();  
      break;
    //Switch Power On
    case 7:
      delay(2500);
      setPower(true);
      delay(2500);
      sendPowerMessage();
      break;
    //Switch Power Off
    case 8:
      delay(2500);
      setPower(false);
      delay(2500);
      sendPowerMessage();
      break;
    //Go to Track
    case 30: 
    case 34:
      delay(2500);
      sendStatusMessage(GOTOTRACK);
      delay(2500);
      goToTrack(p.ParseTrackMessage(m));
      delay(2500);
      sendStatusMessage(READY);
      break;
    //Drop the ToneArm 
    case 31:
      delay(2500);
      sendStatusMessage(PLAY);
      delay(2500);
      play();
      delay(2500);
      sendStatusMessage(READY);
      break;
    //Lift the Tone Arm
    case 32:
      delay(2500);
      sendStatusMessage(PAUSE);
      delay(2500);
      pause();
      delay(2500);
      sendStatusMessage(READY);
      break;
    //Go To Beginning of Record
    case 33:
    case 35:
      delay(2500);
      sendStatusMessage(GOTOTRACK);
      delay(2500);
      goToBeginning();
      delay(2500);
      sendStatusMessage(READY);
      delay(2500);
      sendStatusMessage(ATBEGINNING);
      break;
    default:
      break;
  }
}

void readUDP(){
  if (udpServer.available()) {
      char buffer[UDP_READ_BUFFER_SIZE];
      
      int n = udpServer.readData(buffer, UDP_READ_BUFFER_SIZE);  // n contains # of bytes read into buffer
      Serial.print("n: "); Serial.println(n);


      for (int i = 0; i < n; ++i) {
         uint8_t c = buffer[i];
         Serial.print("c: "); Serial.println(c);
      }
      
      Parser::Header header = p.ParseHeader(buffer);
      Serial.print("Command = ");Serial.println(header.command);
      doCommand(buffer, header);
      
      p.resetPointer();
   }
   else{
     //Serial.println("No Data");
   }
}

///////////////////////////////////////////////////
/////////////////SETUP FUNCTIONS///////////////////
///////////////////////////////////////////////////


void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}

bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    myIP = ipAddress;
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

void doSetup(){
  Serial.println(F("Hello, CC3000!\n")); 

  displayDriverMode();
  
  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    for(;;);
  }


  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  } 
  
  displayMACAddress();
  
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while(1);
  }

  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);
  
  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP()) {
    delay(100); // ToDo: Insert a DHCP timeout!
  }

  /* Display the IP address DNS, Gateway, etc. */  
  while (!displayConnectionDetails()) {
    if(cc3000.checkConnected()){
      Serial.println("Not Connected");
    }
    delay(1000);
  }
}
