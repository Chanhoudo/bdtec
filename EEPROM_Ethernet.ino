#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>

#define EEPROM_SIZE 512  // Flash에서 사용할 메모리 크기
#define W5500_CS 17  // W5500 Chip Select 핀
// 구조체 정의
struct Settings {
  //네트워크 설정
  char mac[18];
  char ip[16];
  char subnet[16];
  char gateway[16];
  char dns[16];
  char serverIp[16];
  char protocol[4];
  int port;
  //RS-485설정
  uint8_t slaveAddress;   // 국번(1 ~ 255)
  uint16_t readAddress;   // 주소(Read Address)
  uint16_t numOfBlocks;   // 읽을 블록 개수 (1 ~ 125)

};

Settings deviceSettings;  // 전역 설정 구조체

// 함수 프로토타입
void loadSettingsFromEEPROM();
void saveSettingsToEEPROM();
void parseAndSaveSettings(String command);
void sendSettings();
void applyNetworkSettings();

void setup() {
  Serial.begin(115200);
  
  // Flash에서 EEPROM 기능을 활성화 (RP2040에서는 Flash 사용)
  EEPROM.begin(EEPROM_SIZE);
  
  Serial.println("EEPROM 초기화 완료, 데이터 불러오는 중...");
  loadSettingsFromEEPROM();  // Flash에서 네트워크 설정 불러오기
  applyNetworkSettings();    // 네트워크 설정 적용
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("WRITE")) {
      parseAndSaveSettings(command);  // 설정 저장
      applyNetworkSettings();         // 새로운 네트워크 설정 적용 write명령
    } else if (command.startsWith("READ_SETTINGS")) {
      sendSettings();                 // 저장된 설정 전송
    }
  }
}

// Flash에서 데이터 불러오기
void loadSettingsFromEEPROM() {
  EEPROM.get(0, deviceSettings);
  Serial.println("EEPROM에서 데이터를 불러왔습니다.");
}

// Flash에 데이터 저장
void saveSettingsToEEPROM() {
  EEPROM.put(0, deviceSettings);
  EEPROM.commit();  // Flash에 저장 확정
  Serial.println("EEPROM (Flash) data save!");
}




// 시리얼 데이터 파싱 및 저장
void parseAndSaveSettings(String command) {
  Serial.print("recevied: ");
  Serial.println(command);
  int macIndex = command.indexOf("MAC=") + 4;
  int ipIndex = command.indexOf("IP=");
  int subnetIndex = command.indexOf("Subnet=");
  int gatewayIndex = command.indexOf("Gateway=");
  int dnsIndex = command.indexOf("DNS=");
  int serverIpIndex = command.indexOf("ServerIP=");
  int protocolIndex = command.indexOf("Protocol=");
  int portIndex = command.indexOf("Port=");
  int slaveIndex = command.indexOf("Slave=");       
  int readAddrIndex = command.indexOf("ReadAddr=");  
  int blockIndex = command.indexOf("Block=");        

  if (macIndex < 4 || ipIndex < 0 || subnetIndex < 0 || gatewayIndex < 0 || dnsIndex < 0 ||
      serverIpIndex < 0 || protocolIndex < 0 || portIndex < 0 || slaveIndex < 0 || readAddrIndex < 0 || blockIndex < 0) {
    Serial.println("잘못된 명령 형식입니다.");
    return;
  }

  // 기존 네트워크 설정
  command.substring(macIndex, ipIndex - 1).toCharArray(deviceSettings.mac, ipIndex - macIndex);
  command.substring(ipIndex + 3, subnetIndex - 1).toCharArray(deviceSettings.ip, subnetIndex - ipIndex - 3);
  command.substring(subnetIndex + 7, gatewayIndex - 1).toCharArray(deviceSettings.subnet, gatewayIndex - subnetIndex - 7);
  command.substring(gatewayIndex + 8, dnsIndex - 1).toCharArray(deviceSettings.gateway, dnsIndex - gatewayIndex - 8);
  command.substring(dnsIndex + 4, serverIpIndex - 1).toCharArray(deviceSettings.dns, serverIpIndex - dnsIndex - 4);
  command.substring(serverIpIndex + 9, protocolIndex - 1).toCharArray(deviceSettings.serverIp, protocolIndex - serverIpIndex - 9);
  command.substring(protocolIndex + 9, portIndex - 1).toCharArray(deviceSettings.protocol, portIndex - protocolIndex - 9);
  deviceSettings.port = command.substring(portIndex + 5, slaveIndex - 1).toInt();

  deviceSettings.slaveAddress = command.substring(slaveIndex + 6, readAddrIndex - 1).toInt();  // 국번
  deviceSettings.readAddress = command.substring(readAddrIndex + 9, blockIndex - 1).toInt();   // 주소(Read)
  deviceSettings.numOfBlocks = command.substring(blockIndex + 6).toInt(); 

  saveSettingsToEEPROM();
}

// 저장된 설정 전송
void sendSettings() {
  Serial.print("MAC=");
  Serial.print(deviceSettings.mac);
  Serial.print(";IP=");
  Serial.print(deviceSettings.ip);
  Serial.print(";Subnet=");
  Serial.print(deviceSettings.subnet);
  Serial.print(";Gateway=");
  Serial.print(deviceSettings.gateway);
  Serial.print(";DNS=");
  Serial.print(deviceSettings.dns);
  Serial.print(";ServerIP=");
  Serial.print(deviceSettings.serverIp);
  Serial.print(";Protocol=");
  Serial.print(deviceSettings.protocol);
  Serial.print(";Port=");
  Serial.print(deviceSettings.port);
  Serial.print(";Slave=");
  Serial.print(deviceSettings.slaveAddress);
  Serial.print(";ReadAddr=");
  Serial.print(deviceSettings.readAddress);
  Serial.print(";Block=");
  Serial.println(deviceSettings.numOfBlocks);
}

void applyNetworkSettings() {
  byte mac[6];
  byte ip[4];
  byte subnet[4];
  byte gateway[4];
  byte dns[4];

  // SPI 및 Ethernet 초기화
  SPI.begin();
  Ethernet.init(W5500_CS); // CS 핀 설정

  // MAC 주소 변환 (16진수)
  sscanf(deviceSettings.mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
         &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

  // IP, 서브넷, 게이트웨이, DNS 변환 (10진수)
  sscanf(deviceSettings.ip, "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]);
  sscanf(deviceSettings.subnet, "%hhu.%hhu.%hhu.%hhu", &subnet[0], &subnet[1], &subnet[2], &subnet[3]);
  sscanf(deviceSettings.gateway, "%hhu.%hhu.%hhu.%hhu", &gateway[0], &gateway[1], &gateway[2], &gateway[3]);
  sscanf(deviceSettings.dns, "%hhu.%hhu.%hhu.%hhu", &dns[0], &dns[1], &dns[2], &dns[3]);

  Ethernet.begin(mac, ip, dns, gateway, subnet);

  // 현재 설정된 네트워크 정보 출력
  Serial.print("Network Configuration Applied:");
  Serial.print(" MAC Address: ");
  Serial.print(deviceSettings.mac);
  Serial.print("; IP Address: ");
  Serial.print(Ethernet.localIP());
  Serial.print("; Subnet Mask: ");
  Serial.print(Ethernet.subnetMask());
  Serial.print("; Gateway: ");
  Serial.print(Ethernet.gatewayIP());
  Serial.print("; DNS Server: ");
  Serial.println(Ethernet.dnsServerIP());
}

// 네트워크 설정 적용
// Apply network settings
/*void applyNetworkSettings() {
  
  
  byte mac[6];
  byte ip[4];
  byte subnet[4];
  byte gateway[4];
  byte dns[4];
  SPI.begin();
  Ethernet.init(17);

  SPI.begin();
  Ethernet.init(W5500_CS);  // CS 핀 설정
  // Convert MAC address (Hexadecimal)
  sscanf(deviceSettings.mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
         &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

  // Convert IP, Subnet, Gateway, DNS (Decimal)
  sscanf(deviceSettings.ip, "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]);
  sscanf(deviceSettings.subnet, "%hhu.%hhu.%hhu.%hhu", &subnet[0], &subnet[1], &subnet[2], &subnet[3]);
  sscanf(deviceSettings.gateway, "%hhu.%hhu.%hhu.%hhu", &gateway[0], &gateway[1], &gateway[2], &gateway[3]);
  sscanf(deviceSettings.dns, "%hhu.%hhu.%hhu.%hhu", &dns[0], &dns[1], &dns[2], &dns[3]);
  
  // // 네트워크 설정 적용 (DHCP 시도)
  // Serial.println("Initializing W5500 with DHCP...");
  // if (Ethernet.begin(mac) == 0) {
  //   Serial.print("DHCP failed! Switching to static IP mode...");

  //   // 고정 IP 모드로 설정
  //   Ethernet.begin(mac, ip, dns, gateway, subnet);
  // }

  // // 네트워크 연결 상태 확인
  // if (Ethernet.linkStatus() == LinkOFF) {
  //   Serial.println("Ethernet cable is not connected!");
  // } else {
  //   Serial.print("Ethernet connection established.");
  // }


  // Print network configuration
  Serial.print("Network Configuration Applied:");
  Serial.print("MAC Address: ");
  Serial.println(deviceSettings.mac);
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DNS Server: ");
  Serial.println(Ethernet.dnsServerIP());
}*/
