#include <Arduino.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <EthernetUdp.h>
//--------------------------------------------------
// 공용 정의
//--------------------------------------------------
#define W5500_CS 17       // Ethernet용 W5500 Chip Select 핀
#define EEPROM_SIZE 512   // EEPROM(Flash) 메모리 크기

//--------------------------------------------------
// [1] DAQ보드와의 Serial1 Modbus 통신 (센서 데이터 수집)
//--------------------------------------------------
constexpr unsigned char SLAVE_ADDRESS      = 0x01;
constexpr unsigned char FUNCTION_CODE      = 0x03;
constexpr uint16_t      REGISTER_START     = 0x0100;
constexpr unsigned char REGISTER_COUNT     = 4;
constexpr unsigned char RESPONSE_MAX_LEN   = 32;
char receivedDataByte = 0;


// 평균 내기 위해 사용했던 데이터 스토리지
// unsigned char dataStorage[MAX_STORAGE_SIZE][255];
// unsigned char avgStorage[MAX_AVG_STORAGE_SIZE][RECEIVED_DATA_BYTE];
// int avgIndex  = 0;  // 5분 평균 저장 개수
//int dataIndex = 0;  // 현재 데이터 개수
//constexpr int MAX_STORAGE_SIZE = 600;   // 5분 저장 (0.5초 간격 x 600)
// constexpr int MAX_AVG_STORAGE_SIZE = 6;   // 30분 저장 (5분 평균 x 6)

// 최신 센서 데이터 (Modbus 응답에서 받은 8바이트)
unsigned char latestSensorData[255] = {0};

volatile bool requestFlag = false; // DAQ 요청 flag
volatile bool avgCalcFlag = false;   // 평균 계산 flag

repeating_timer_t modbusTimer;    // 0.5초마다 실행
// repeating_timer_t modbusAvgTimer; // 5분마다 실행

//--------------------------------------------------
// [2] PC와의 Serial 통신 (네트워크/RS485 설정 via EEPROM)
//--------------------------------------------------
struct Settings {
  // 네트워크 설정
  char mac[18];      // 예: "DE:AD:BE:EF:FE:ED"
  char ip[16];       // 예: "192.168.1.100"
  char subnet[16];   // 예: "255.255.255.0"
  char gateway[16];  // 예: "192.168.1.1"
  char dns[16];      // 예: "8.8.8.8"
  char serverIp[16]; // 서버 IP (필요 시)
  char protocol[4];  // 프로토콜 문자열
  int port;          // Ethernet 서버 포트
  
  // RS-485 설정
  uint8_t slaveAddress;   // 국번 (1~255)
  uint16_t readAddress;   // 읽기 주소
  uint16_t numOfBlocks;   // 읽을 블록 개수 (1~125)
};

Settings deviceSettings;  // 전역 설정 구조체

//--------------------------------------------------
// [3] Ethernet ModbusTCP 서버 (센서 데이터 전송)
//--------------------------------------------------
// Ethernet 서버 객체 (동적 할당하여 네트워크 설정 변경 시 재생성)
EthernetServer* modbusTCPServer = nullptr;
EthernetUDP udp;
bool udpActive = false; //UDP열려있는지 확인용 flag
//--------------------------------------------------
// 함수 프로토타입
//--------------------------------------------------
// [1] DAQ보드 관련
bool requestModbusData(repeating_timer_t *t);
bool calculateFiveMinAverageTimer(repeating_timer_t *t);
void sendModbusRequest();
void receiveModbusResponse();
bool validateModbusResponse(unsigned char *response, int length);
uint16_t calculateCRC(unsigned char *data, unsigned char length);
// void calculateFiveMinAverage();
// void calculateThirtyMinAverage();
void clearSerial1Buffer();

// [2] Serial(Ethernet 설정) 관련
void loadSettingsFromEEPROM();
void saveSettingsToEEPROM();
void parseAndSaveSettings(String command);
void sendSettings();
void applyNetworkSettings();

// [3] Ethernet ModbusTCP 관련
void sendModbusTCPResponse(EthernetClient &client, byte *request);
bool validateModbusTCPRequest(byte *request, int len);
void sendUDPSensorData();

//--------------------------------------------------
// setup() 및 loop()
//--------------------------------------------------
void setup() 
{
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1);
    
    // EEPROM 초기화 및 네트워크 설정 불러오기
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("EEPROM 초기화 완료, 데이터 불러오는 중...");
    loadSettingsFromEEPROM();
    applyNetworkSettings();

    // Ethernet 서버 생성 (deviceSettings.port가 0이면 기본 503 사용)
    startModbusServer();

    // DAQ 보드 통신 타이머 시작
    add_repeating_timer_ms(500, requestModbusData, NULL, &modbusTimer);
    //   add_repeating_timer_ms(300000, calculateFiveMinAverageTimer, NULL, &modbusAvgTimer);
}

void loop() 
{
    // [1] DAQ 보드와의 통신 처리
    if (requestFlag) 
    {
        requestFlag = false;
        sendModbusRequest();
        receiveModbusResponse();
    }
    
    // [2] PC와의 Serial 명령 처리 (네트워크/RS485 설정)
    if (Serial.available()) 
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("WRITE")) 
        {
            parseAndSaveSettings(command);
            applyNetworkSettings();
            // Ethernet 서버도 새 네트워크 설정에 맞게 재시작
            startModbusServer();
        } 
        else if (command.startsWith("READ_SETTINGS")) 
        {
            sendSettings();
        }
    }
  
    // [3] Ethernet ModbusTCP 클라이언트 요청 처리
    // UDP 모드에서는 센서 데이터가 수신될 때마다 deviceSettings.serverIp로 푸시됩니다.
    if (strcmp(deviceSettings.protocol, "TCP") == 0) 
    {
        EthernetClient client = modbusTCPServer->available();
        if (client) 
        {
            Serial.println("Client Connected!");
            byte request[12];
            int index = 0;
            unsigned long startTime = millis();
            // 0.5초 이내에 요청 데이터를 수신
            while (millis() - startTime < 500) 
            {
                if (client.available()) 
                {
                    request[index++] = client.read();
                    if (index >= 12) break;
                }
            }
            Serial.print("Arduino: Received Modbus TCP Request: ");
            printHex(request, index);
            // 요청 패킷 검증
            if (!validateModbusTCPRequest(request, index)) 
            {
                client.write(request, index);
                Serial.println("Request validation failed. Discarding request.");
                client.stop();
                return;
            }
            
            // 요청이 유효하면 Response 생성 후 전송
            sendModbusTCPResponse(client, request);
        } 
    }
}

//--------------------------------------------------
// [1] DAQ보드 관련 함수 구현
//--------------------------------------------------

// 타이머 콜백: 0.5초마다 DAQ 보드에 Modbus 요청 flag 설정 (DAQ와 통신)
bool requestModbusData(repeating_timer_t *t) 
{
    requestFlag = true;
    return true;
}

// DAQ 보드에 Modbus 요청 패킷 생성 및 전송 (DAQ와 통신)
void sendModbusRequest() 
{
    clearSerial1Buffer();
    unsigned char request[8] = 
    {
        SLAVE_ADDRESS, FUNCTION_CODE,
        (REGISTER_START >> 8) & 0xFF, REGISTER_START & 0xFF,
        (REGISTER_COUNT >> 8) & 0xFF, REGISTER_COUNT & 0xFF,
        0x00, 0x00   // CRC 자리 (후에 채움)
    };

    uint16_t crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    Serial.print("Serial1🚀 Send Data: ");
    printHex(request, 8);

    for (int i = 0; i < 8; i++) 
    {
        Serial1.write(request[i]);
    }
}

// DAQ 보드에서 Modbus 응답 수신 및 저장 (최신 센서 데이터 업데이트) (DAQ와 통신)
void receiveModbusResponse() 
{
    
    unsigned char response[RESPONSE_MAX_LEN] = {0};
    int index = 0;
    unsigned long startTime = millis();

    if (index >= RESPONSE_MAX_LEN)
    {
        index = 0;
    }

    // 최대 0.5초 동안 응답 수신
    while (millis() - startTime < 500) 
    {
        if (Serial1.available()) 
        {
            unsigned char byteReceived = Serial1.read();
            response[index++] = byteReceived;

            if (index >= RESPONSE_MAX_LEN) 
            {
                Serial.println("❌ Response Overflow! Clearing Buffer.");
                return;
            }

            // 초기 4바이트 검사 (예제에서는 0x01, 0x03, RECEIVED_DATA_BYTE, 0x33 확인)
            if (index == 2 && (response[0] != SLAVE_ADDRESS || response[1] != FUNCTION_CODE)) 
            {
                Serial.println("❌ Invalid Start Sequence! Discarding Response...");
                return;
            }

            // 예상 길이 도달 (ByteCount + 5)
            if (index >= 5 && index >= response[2] + 5) break;
        }
    }

    // CRC 검증
    if (!validateModbusResponse(response, index)) 
    {
        Serial.println("CRC Error: Response Invalid!");
        return;
    }
    receivedDataByte = response[2];

    Serial.print("Check Ok! DAQ Data: ");
    printHex(response, index);

    // 최신 센서 데이터 업데이트
    memcpy(latestSensorData, response + 3, receivedDataByte);
    // UDP 모드인 경우, 센서 데이터를 deviceSettings.serverIp로 푸시 전송
    if (strcmp(deviceSettings.protocol, "UDP") == 0) {
        sendUDPSensorData();
    }
}

// Modbus 응답의 CRC 검증 (DAQ와 통신)
bool validateModbusResponse(unsigned char *response, int length) 
{
    if (length < 5) return false;
    uint16_t receivedCRC = (response[length - 1] << 8) | response[length - 2];
    uint16_t calculatedCRC = calculateCRC(response, length - 2);
    
    Serial.print("Serial1 CRC Check: Received = 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(", Calculated = 0x");
    Serial.println(calculatedCRC, HEX);
    
    return (receivedCRC == calculatedCRC);
}

// Modbus CRC16 계산 (LSB 우선) (DAQ와 통신)
uint16_t calculateCRC(unsigned char *data, unsigned char length) 
{
    uint16_t crc = 0xFFFF;
    for (unsigned char i = 0; i < length; i++) 
    {
        crc ^= data[i];
        for (unsigned char j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

// Serial1 버퍼 비우기 (DAQ와 통신)
void clearSerial1Buffer() 
{
  while (Serial1.available())  Serial1.read();
  Serial.println("Serial1 Buffer Cleared!");
}

//--------------------------------------------------
// [2] 네트워크 설정 (EEPROM, Serial 명령) 관련 함수 구현
//--------------------------------------------------

// EEPROM에서 설정 값 불러오기
void loadSettingsFromEEPROM() 
{
    EEPROM.get(0, deviceSettings);
    Serial.println("EEPROM에서 데이터를 불러왔습니다.");
}

// EEPROM에 설정 값 저장
void saveSettingsToEEPROM()
{
    EEPROM.put(0, deviceSettings);
    EEPROM.commit();
    Serial.println("EEPROM (Flash) data save!");
}

// Serial로 받은 명령 파싱하여 설정 저장 ("WRITE" 명령 사용)
void parseAndSaveSettings(String command) 
{
    int macIndex      = command.indexOf("MAC=") + 4;
    int ipIndex       = command.indexOf("IP=");
    int subnetIndex   = command.indexOf("Subnet=");
    int gatewayIndex  = command.indexOf("Gateway=");
    int dnsIndex      = command.indexOf("DNS=");
    int serverIpIndex = command.indexOf("ServerIP=");
    int protocolIndex = command.indexOf("Protocol=");
    int portIndex     = command.indexOf("Port=");
    int slaveIndex    = command.indexOf("Slave=");
    int readAddrIndex = command.indexOf("ReadAddr=");
    int blockIndex    = command.indexOf("Block=");

    if (macIndex < 4 || ipIndex < 0 || subnetIndex < 0 || gatewayIndex < 0 || dnsIndex < 0 ||
        serverIpIndex < 0 || protocolIndex < 0 || portIndex < 0 || slaveIndex < 0 ||
        readAddrIndex < 0 || blockIndex < 0) 
    {
        Serial.println("잘못된 명령 형식입니다.");
        return;
    }

    command.substring(macIndex, ipIndex - 1).toCharArray(deviceSettings.mac, sizeof(deviceSettings.mac));
    command.substring(ipIndex + 3, subnetIndex - 1).toCharArray(deviceSettings.ip, sizeof(deviceSettings.ip));
    command.substring(subnetIndex + 7, gatewayIndex - 1).toCharArray(deviceSettings.subnet, sizeof(deviceSettings.subnet));
    command.substring(gatewayIndex + 8, dnsIndex - 1).toCharArray(deviceSettings.gateway, sizeof(deviceSettings.gateway));
    command.substring(dnsIndex + 4, serverIpIndex - 1).toCharArray(deviceSettings.dns, sizeof(deviceSettings.dns));
    command.substring(serverIpIndex + 9, protocolIndex - 1).toCharArray(deviceSettings.serverIp, sizeof(deviceSettings.serverIp));
    command.substring(protocolIndex + 9, portIndex - 1).toCharArray(deviceSettings.protocol, sizeof(deviceSettings.protocol));
    deviceSettings.port = command.substring(portIndex + 5, slaveIndex - 1).toInt();

    deviceSettings.slaveAddress = command.substring(slaveIndex + 6, readAddrIndex - 1).toInt();
    deviceSettings.readAddress  = command.substring(readAddrIndex + 9, blockIndex - 1).toInt();
    deviceSettings.numOfBlocks  = command.substring(blockIndex + 6).toInt();

    saveSettingsToEEPROM();
}

// 저장된 설정 값 Serial 전송 ("READ_SETTINGS" 명령 사용)
void sendSettings() 
{
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

// 네트워크 설정을 Ethernet에 적용 (EEPROM/Serial로 설정된 값 사용)
void applyNetworkSettings() 
{
    byte mac[6];
    byte ip[4];
    byte subnet[4];
    byte gateway[4];
    byte dns[4];

    SPI.begin();
    Ethernet.init(W5500_CS);

    // MAC 주소 파싱 (형식: "DE:AD:BE:EF:FE:ED")
    sscanf(deviceSettings.mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
            &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    // IP, 서브넷, 게이트웨이, DNS 파싱 (형식: "192.168.1.100")
    sscanf(deviceSettings.ip, "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]);
    sscanf(deviceSettings.subnet, "%hhu.%hhu.%hhu.%hhu", &subnet[0], &subnet[1], &subnet[2], &subnet[3]);
    sscanf(deviceSettings.gateway, "%hhu.%hhu.%hhu.%hhu", &gateway[0], &gateway[1], &gateway[2], &gateway[3]);
    sscanf(deviceSettings.dns, "%hhu.%hhu.%hhu.%hhu", &dns[0], &dns[1], &dns[2], &dns[3]);

    Ethernet.begin(mac, ip, dns, gateway, subnet);

    Serial.print("Network Configuration Applied: ");
    Serial.print("MAC=");
    Serial.print(deviceSettings.mac);
    Serial.print("; IP=");
    Serial.print(Ethernet.localIP());
    Serial.print("; Subnet=");
    Serial.print(Ethernet.subnetMask());
    Serial.print("; Gateway=");
    Serial.print(Ethernet.gatewayIP());
    Serial.print("; DNS=");
    Serial.println(Ethernet.dnsServerIP());
}

//--------------------------------------------------
// [3] Ethernet ModbusTCP, UDP 서버 관련 함수 구현
//--------------------------------------------------

// 시작 시 프로토콜에 따라 TCP 또는 UDP 초기화
void startModbusServer() 
{
    int serverPort = (deviceSettings.port == 0) ? 503 : deviceSettings.port;
    if (strcmp(deviceSettings.protocol, "UDP") == 0)
    {
        udp.begin(serverPort);
        udpActive = true;
        Serial.print("Modbus UDP Mode: UDP 전송 포트 ");
        Serial.println(serverPort);
        if (modbusTCPServer != nullptr) 
        {
            delete modbusTCPServer;
            modbusTCPServer = nullptr;
        }
    } else 
    {
        modbusTCPServer = new EthernetServer(serverPort);
        modbusTCPServer->begin();
        Serial.print("Modbus TCP Server Started on port ");
        Serial.println(serverPort);

        if (udpActive)
        {
            udp.stop();
            udpActive = false;
            Serial.println("Modbus UDP Socket Closed");
        }
    }
}


// 요청 패킷 검증 함수 request: 수신된 패킷 배열, len: 패킷 길이
bool validateModbusTCPRequest(byte *request, int len) 
{
    if (len < 8) 
    {
        Serial.println("Too Short length.");
        return false;
    }
    
    // 고정된 값 검증 (바이트 0,1,2,3,4,7,8,9,10,11)
    if (request[0] != 0x42 || request[1] != 0x44) 
    {
        Serial.println("Invalid Protocol ID.");
        return false;
    }

    if (request[6] != (unsigned char)deviceSettings.slaveAddress)
    {
        Serial.println("Invalid Unit ID (byte6");
        return false;
    }
    if (request[7] != 0x03) 
    {
        Serial.println("Invalid Function Code (byte 7).");
        return false;
    }

    return true;
}
// Ethernet 클라이언트로 센서 데이터가 포함된 ModbusTCP 응답 전송
void sendModbusTCPResponse(EthernetClient &client, byte *request) 
{

    //요청된 레지스터 개수 가져오기
    uint16_t requestedRegisters = (request[10] << 8) | request[11]; // 2바이트 결합
    int byteCount = requestedRegisters * 2;  // 요청된 레지스터 개수 * 2바이트
    int responseLength = 6 + 1 + 1 + 1 + byteCount;  // 헤더(6) + Unit ID(1) + Function(1) + 데이터(byteCount)(1) + Bytecount

    // 동적 배열 할당
    byte *response = new byte[responseLength];

    // [1] Modbus TCP 헤더
    response[0] = 0x42;  // Protocol ID ("BD")
    response[1] = 0x44;
    response[2] = (responseLength - 4) >> 8; // Length 상위 바이트
    response[3] = (responseLength - 4) & 0xFF; // Length 하위 바이트
    response[4] = request[4]; // Transaction ID
    response[5] = request[5]; // Transaction ID

    // [2] Modbus 응답
    response[6] = (unsigned char)deviceSettings.slaveAddress; // Unit ID
    response[7] = 0x03; // Function Code (Read Input Registers)
    response[8] = byteCount; // 바이트 개수 설정

    // [3] 센서 데이터 삽입 (최대 RECEIVED_DATA_BYTE까지)
    for (int i = 0; i < min(byteCount, receivedDataByte); i++) {
        response[9 + i] = latestSensorData[i]; // 최신 센서 데이터 삽입
    }

    // [4] 부족한 바이트를 0x00으로 채우기 (데이터 개수 차이 보정)
    for (int i = receivedDataByte; i < byteCount; i++) {
        response[9 + i] = 0x00; // 부족한 부분을 0x00으로 패딩
    }

    // [5] 응답 전송
    client.write(response, responseLength);
    
    Serial.print("Aduino: Sent Modbus TCP Response: ");
    printHex(response, responseLength);

    // [6] 메모리 해제
    delete[] response;
}

// UDP 푸시 전용 함수: 센서 데이터를 deviceSettings.serverIp로 전송
void sendUDPSensorData() 
{
    const int byteCount = receivedDataByte;
    const int responseLength = 6 + 1 + 1 + 1 + (byteCount);  // 예: 6+1+1+1+8 = 17 바이트
    byte response[responseLength];

    // [1] MBAP 헤더 구성
    response[0] = 0x42;  // Protocol ID "BD"
    response[1] = 0x44;
    int lenField = responseLength - 4;  // Length 필드: 총 길이 - 4
    response[2] = (lenField >> 8) & 0xFF;
    response[3] = lenField & 0xFF;
    // Transaction ID (푸시용으로 0)
    response[4] = 0x00;
    response[5] = 0x00;

    // [2] Modbus 응답 필드
    response[6] = deviceSettings.slaveAddress; // Unit ID
    response[7] = 0x03;                        // Function Code (Read Input Registers)
    response[8] = byteCount;                   // Byte Count

    // [3] 최신 센서 데이터 삽입
    for (int i = 0; i < byteCount && i < (int)receivedDataByte; i++) {
        response[9 + i] = latestSensorData[i];
    }
    // [4] 부족한 바이트 0x00 패딩 (필요시)
    for (int i = receivedDataByte; i < byteCount; i++) {
        response[9 + i] = 0x00;
    }

    // deviceSettings.serverIp 문자열을 IPAddress로 변환 후 전송
    IPAddress serverIP;
    if (!serverIP.fromString(deviceSettings.serverIp)) {
      Serial.print("Invalid Server IP: ");
      Serial.println(deviceSettings.serverIp);
      return;
    }
    udp.beginPacket(serverIP, deviceSettings.port);
    udp.write(response, responseLength);
    udp.endPacket();
    
    Serial.print("UDP Push: Sent sensor data to ");
    Serial.print(deviceSettings.serverIp);
    Serial.print(":");
    Serial.println(deviceSettings.port);
    Serial.print("Data: ");
    printHex(response, responseLength);
}


//---------------------------------------------------
// [4] 공통 디버깅 출력 용 (0x00 데이터 출력)
//---------------------------------------------------
// 공용 디버깅용 HEX 출력 함수 (byte 단위)
void printHex(const byte *data, int length) 
{
    for (int i = 0; i < length; i++) 
    {
        Serial.print("0x");
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
