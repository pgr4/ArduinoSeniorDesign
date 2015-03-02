#include <Parser.h>

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <UDPServer.h>

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed but DI

uint sendAddress;

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

Adafruit_CC3000_Client client;

const unsigned long
  connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  responseTimeout = 15L * 1000L; // Max time to wait for data from server

uint32_t myIP;

#define UDP_READ_BUFFER_SIZE 20
#define LISTEN_PORT_UDP 30003

UDPServer udpServer = UDPServer(LISTEN_PORT_UDP);

Parser p = Parser();
const int totPixels = 1536;
int IntArray[totPixels];

int d50 = 50;
int d51 = 51;
int d52 = 52;

void setup(void){
  
  pinMode(50,OUTPUT);
  pinMode(51,OUTPUT);
  digitalWrite(50, LOW); 
  digitalWrite(51, LOW); 
  
  doSetup();
  
  Serial.println("Done with Setup");
    
  udpServer.begin();
}


void loop(void) {
  readUDP();
}

void play(){
}

void goToTrack(Parser::TrackMessage m){ 
}

void pause(){
}

void stopLift(){
}

void doCommand(char* m, int command){
 switch(command){
   //Status
    case 3:
      digitalWrite(50, HIGH);
      delay(2500);
      sendStatusMessage(21);
      digitalWrite(50, LOW); 
      break;
    //Scan  
    case 4:
      digitalWrite(51, HIGH);
      delay(2500);
      sendStatusMessage(26);
      delay(2500);
      writeNewRecord();
      delay(2500);
      sendStatusMessage(21);
      digitalWrite(51, LOW); 
      break;
   //Go to Track
    case 10: 
      goToTrack(p.ParseTrackMessage(m));
      break;
    //Play
    case 11:
      play();
      break;
    //Lift
    case 12:
      pause();
      break;
    //Pause
    case 13:
      stopLift();
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
      Serial.println("Command Start");
      doCommand(buffer, header.command);
      Serial.println("Command Done");
      
      p.resetPointer();
   }
   else{
     //Serial.println("No Data Homes");
   }
}

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

void writeNewRecord() {  
  //Size depends on id
  uint8_t buf[25];
  
  byte sIP[4] = {myIP >> 24, myIP >> 16, myIP >> 8, myIP};
  byte dIP[4] = {192, 168, 1, 255};
  byte cutoff[6] = {111, 111, 111, 111, 111, 111};
  byte id[10] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
  
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
      
      buf[pointer] = 1;
      pointer += 1;
      
      memcpy_P(&buf[pointer], cutoff, sizeof(cutoff));
      pointer += sizeof(cutoff);
      
      memcpy_P(&buf[pointer], id, sizeof(id));
      pointer += sizeof(id);
           
      client.write(buf, sizeof(buf));
  }
   client.close();
}

//////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////

void doSetup(){
Serial.begin(115200);
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
