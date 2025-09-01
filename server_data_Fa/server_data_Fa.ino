#include <WiFi.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <mcp2515.h>
//현재 확장 PID는 256x5, 일반 PID는 12byte를 소모
#include <TinyGPS++.h>
#include <HardwareSerial.h>
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

double mylat, mylng;

const char* gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

struct can_frame read_can_msg;
struct can_frame write_can_msg;
MCP2515 mcp2515(15);

// Wi-Fi 네트워크 이름 및 비밀번호 설정
const char* ssid = "SSIBSARE";      // Wi-Fi SSID
const char* password = "song1472";  // Wi-Fi 비밀번호

// 서버 IP 및 포트 설정
IPAddress server(13,125,75,159);  // 서버 IP 주소
int port = 4000;                   // 서버 포트 번호

float module[97];               //배터리 98개 셀 전압데이터
float bat_temp[4];              //배터리 4개 파트별 온도
int charge_state = 0;           //배터리 충전상태
char device_num[] = "9314321";  //고유 디바이스 넘버, 등록된 기기하고 연동
int State_of_Charge_BMS = 0;    //배터리 %
int Battery_Power = 50;         //kw, 충전기 충전속도

//데이터 저장용 버퍼 expid 22번 5개, 이외 데이터등...
int expid[5][47];
int c = 0;  //0~7 byte 받는용도

// Wi-Fi 클라이언트 객체 생성
WiFiClient client;

void setup() {
  Serial.begin(115200);                       // 시리얼 통신 시작
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX 2번, 23번 핀으로 시리얼 통신2 시작, GPS용

  mcp2515.reset();  //CAN 통신 모듈
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Wi-Fi 연결 시작
  Serial.print("Wi-Fi connecting...");
  WiFi.begin(ssid, password);

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 6) {  // 최대 6초 대기
    delay(1000);
    Serial.println("Reconnecting to WiFi...");
    retryCount++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi");
    esp_restart();  // 연결 실패 시 강제 재부팅
  } else {
    Serial.println("Connected to WiFi");
  }

  Serial.println("\nWi-Fi connection complete");

  //CanData_cellvolt();  //expid 에 데이터를 저장하도록함
  //data_p();            // expid에서 받아온 원본 pid data를 파싱해서 전역변수에 저장
  //gpsloc();
  //sendDataToServer_MT();  // 모듈별 온도 데이터를 전송
  //sendDataToServer_cellvolt();  //셀 전압전송
  //sendDataToServer_ChargeState();  //배터리 충전상태 전송
  //-> 실기에서 LOOP로 일정 시간마다 반복해서 데이터를 전달할 함수
  // 현재 LOOP문은 시연을 위해 Serial모니터를 사용해 특정 시나리오를 위해 데이터를 보내도록 개조되어있습니다.
}

void loop() {

  if (Serial.available()) {
    Serial.println("Serial com active");
    Serial.println("-------------OPTIONS-------------");
    Serial.println("1 : Send car BAT TMEP");
    Serial.println("2 : Send car Cell Volt");
    Serial.println("3 : Send car ChargeState");
    Serial.println("4 : data test");

    int value = Serial.parseInt();
    if (value == 1) {
      CanData_cellvolt();
      data_p();
      Serial.println("action 1 : Send car BAT TMEP");
      sendDataToServer_MT();
    } else if (value == 2) {
      CanData_cellvolt();
      data_p();
      Serial.println("action 2 : Send car Cell Volt");
      sendDataToServer_cellvolt();
    } else if (value == 3) {
      CanData_cellvolt();
      data_p();
      Serial.println("action 3 : Send car ChargeState");
      sendDataToServer_ChargeState();
    } else if (value == 4){
      Serial.println("action 4 : Data test");
      CanData_cellvolt();
      data_p();
    }
  }
  //CanData_cellvolt();  //expid 에 데이터를 저장하도록함
  //data_p();            // expid에서 받아온 원본 pid data를 파싱해서 전역변수에 저장
  //gpsloc(); //GPS 모듈을 활용하여 현재 위치 판별
  //sendDataToServer_MT();  // 모듈별 온도 데이터를 전송
  //sendDataToServer_cellvolt();  //셀 전압전송
  //sendDataToServer_ChargeState();  //배터리 충전상태 전송

  // 데이터를 추가적으로 보낼 경우 loop에서 호출 가능
}

void CanData_cellvolt() {
  for (int requestNum = 1; requestNum <= 5; requestNum++) {
    Serial.print("exPID");
    Serial.print(requestNum);
    Serial.println(" request");

    write_can_msg.can_id = 2028;  // ECU에 DATA 요청
    write_can_msg.can_dlc = 8;    // 바이트 길이수
    write_can_msg.data[0] = 6;    // 길이
    write_can_msg.data[1] = 65;   // 서비스 번호
    write_can_msg.data[2] = 22;   // PID (확장 PID)
    write_can_msg.data[3] = 1;    // 기본 데이터
    write_can_msg.data[4] = requestNum;
    write_can_msg.data[5] = 0;
    write_can_msg.data[6] = 0;
    write_can_msg.data[7] = 0;

    mcp2515.sendMessage(&write_can_msg);
    delay(100);

    int byteIndex = 0;  // `expid` 배열에 저장할 위치 초기화
    bool receivingMultiFrame = true;
    unsigned long startTime = millis();

    while (receivingMultiFrame) {
      if (mcp2515.readMessage(&read_can_msg) == MCP2515::ERROR_OK) {
        int frameType = read_can_msg.data[0];  // 프레임 타입 구분을 위한 첫 바이트

        // 첫 프레임인 경우 byteIndex 초기화
        if (frameType == 0x10) {
          byteIndex = 0;
        } else if (frameType < 0x20 || frameType > 0x2F) {
          // 유효하지 않은 프레임은 무시
          continue;
        }

        // 프레임 데이터를 expid 배열에 저장
        for (int i = 0; i <= 7 && byteIndex < 48; i++) {
          expid[requestNum - 1][byteIndex++] = read_can_msg.data[i];
        }

        // 모든 데이터를 수신했으면 루프 탈출
        if (byteIndex >= 47) {
          receivingMultiFrame = false;
        }
      } else if (millis() - startTime > 5000) {  // 5초 타임아웃
        Serial.print(requestNum);
        Serial.println("no expid complete");
        receivingMultiFrame = false;
      }

      delay(10);
    }

    // 수신한 expid 배열 출력
    Serial.print(requestNum);
    Serial.print("no data: ");
    for (int i = 0; i < 47; i++) {
      Serial.print(expid[requestNum - 1][i]);
      Serial.print(" ");
    }
    Serial.println();
    delay(100);
  }
}



void data_p() {  //expid에서 받아온 원본 pid data를 파싱해서 전역변수에 저장

  bat_temp[0] = expid[0][23];
  bat_temp[1] = expid[0][25];
  bat_temp[2] = expid[0][26];
  bat_temp[3] = expid[0][27];  //배터리 모듈 4개의 온도

  if (expid[0][15] == 128) charge_state = 1;
  else charge_state = 0;  //충전기에 물려있는지 아닌지 판별, 1이면 충전 0이면 충전 x

  State_of_Charge_BMS = expid[0][10] / 2;  // 배터리가 충전되어있는 %


  for (int i = 0; i <= 5; i++) {
    module[i] = (expid[1][10 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[6 + i] = (expid[1][17 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[13 + i] = (expid[1][25 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[20 + i] = (expid[1][33 + i]) / 50.0;
  }
  for (int i = 0; i <= 4; i++) {
    module[27 + i] = (expid[1][41 + i]) / 50.0;
  }

  for (int i = 0; i <= 5; i++) {
    module[32 + i] = (expid[2][10 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[38 + i] = (expid[2][17 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[45 + i] = (expid[2][25 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[52 + i] = (expid[2][33 + i]) / 50.0;
  }
  for (int i = 0; i <= 4; i++) {
    module[59 + i] = (expid[2][41 + i]) / 50.0;
  }

  for (int i = 0; i <= 5; i++) {
    module[64 + i] = (expid[3][10 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[70 + i] = (expid[3][17 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[77 + i] = (expid[3][25 + i]) / 50.0;
  }
  for (int i = 0; i <= 6; i++) {
    module[84 + i] = (expid[3][33 + i]) / 50.0;
  }
  for (int i = 0; i <= 4; i++) {
    module[91 + i] = (expid[3][41 + i]) / 50.0;
  }
  module[96] = (expid[4][44]) / 50.0;
  module[97] = (expid[4][45]) / 50.0;


  Serial.print("bat charge state :");
  Serial.print(State_of_Charge_BMS);
  Serial.println("%");
  Serial.println("bat temp by module: ");
  Serial.println(bat_temp[0]);
  Serial.println(bat_temp[1]);
  Serial.println(bat_temp[2]);
  Serial.println(bat_temp[3]);

  Serial.print("charging?");
  Serial.println(charge_state);


  Serial.println("bat cell voltage: ");
  for (int i = 0; i < 98; i++) {
    Serial.print(i + 1);
    Serial.print("cell volt:");
    Serial.println(module[i]);
  }
}

void gpsloc() {  //GPS 위도, 경도를 추출하는
  unsigned long start = millis();

  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      Serial.print("LAT: ");
      mylat = (gps.location.lat());
      Serial.println(mylat, 6);
      Serial.print("LONG: ");
      mylng = (gps.location.lng());
      Serial.println(mylng, 6);
      Serial.print("SPEED (km/h) = ");
      Serial.println(gps.speed.kmph());
      Serial.print("ALT (min)= ");
      Serial.println(gps.altitude.meters());
      Serial.print("HDOP = ");
      Serial.println(gps.hdop.value() / 100.0);
      Serial.print("Satellites = ");
      Serial.println(gps.satellites.value());
      Serial.print("Time in UTC: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      Serial.println("");
    }
  }
}


void sendDataToServer_MT() {  // 서버로 JSON 데이터 전송 함수, 4개 모듈별 온도
  for (int i = 1; i < 2; i++) {
    // JSON 객체 생성
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["device_number"] = "888777";  //디바이스의 고유넘버, 이것과 아이디를 연동하여 사용한다.
    jsonDoc["module_number"] = i;
    jsonDoc["module_temp"] = bat_temp[i - 1];

    // JSON 데이터를 문자열로 변환
    char jsonBuffer[200];
    serializeJson(jsonDoc, jsonBuffer);

    // 서버에 연결
    if (client.connect(server, port)) {
      Serial.println("server connection complete");

      // HTTP POST 요청 전송
      client.println("POST /cars/batteryTemp HTTP/1.1");
      client.println("Host: 13.125.75.159:4000");
      client.println("Content-Type: application/json");
      client.print("Content-Length: ");
      client.println(strlen(jsonBuffer));
      client.println();
      client.println(jsonBuffer);

      // 서버 응답 읽기
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          Serial.print(c);
        }
      }

      // 연결 종료
      client.stop();
      Serial.println("연결 종료");
    } else {
      Serial.println("서버 연결 실패");
    }
  }
}

void sendDataToServer_cellvolt() {  // 서버로 JSON 데이터 전송 함수
  for (int n = 9; n < 10; n++) {
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["device_number"] = "888777";

    if (n == 0) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_00"] = module[0];
      jsonDoc["cell_01"] = module[1];
      jsonDoc["cell_02"] = module[2];
      jsonDoc["cell_03"] = module[3];
      jsonDoc["cell_04"] = module[4];
      jsonDoc["cell_05"] = module[5];
      jsonDoc["cell_06"] = module[6];
      jsonDoc["cell_07"] = module[7];
      jsonDoc["cell_08"] = module[8];
      jsonDoc["cell_09"] = module[9];
    }
    if (n == 1) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_10"] = module[10];
      jsonDoc["cell_11"] = module[11];
      jsonDoc["cell_12"] = module[12];
      jsonDoc["cell_13"] = module[13];
      jsonDoc["cell_14"] = module[14];
      jsonDoc["cell_15"] = module[15];
      jsonDoc["cell_16"] = module[16];
      jsonDoc["cell_17"] = module[17];
      jsonDoc["cell_18"] = module[18];
      jsonDoc["cell_19"] = module[19];
    }
    if (n == 2) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_20"] = module[20];
      jsonDoc["cell_21"] = module[21];
      jsonDoc["cell_22"] = module[22];
      jsonDoc["cell_23"] = module[23];
      jsonDoc["cell_24"] = module[24];
      jsonDoc["cell_25"] = module[25];
      jsonDoc["cell_26"] = module[26];
      jsonDoc["cell_27"] = module[27];
      jsonDoc["cell_28"] = module[28];
      jsonDoc["cell_29"] = module[29];
    }
    if (n == 3) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_30"] = module[30];
      jsonDoc["cell_31"] = module[31];
      jsonDoc["cell_32"] = module[32];
      jsonDoc["cell_33"] = module[33];
      jsonDoc["cell_34"] = module[34];
      jsonDoc["cell_35"] = module[35];
      jsonDoc["cell_36"] = module[36];
      jsonDoc["cell_37"] = module[37];
      jsonDoc["cell_38"] = module[38];
      jsonDoc["cell_39"] = module[39];
    }
    if (n == 4) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_40"] = module[40];
      jsonDoc["cell_41"] = module[41];
      jsonDoc["cell_42"] = module[42];
      jsonDoc["cell_43"] = module[43];
      jsonDoc["cell_44"] = module[44];
      jsonDoc["cell_45"] = module[45];
      jsonDoc["cell_46"] = module[46];
      jsonDoc["cell_47"] = module[47];
      jsonDoc["cell_48"] = module[48];
      jsonDoc["cell_49"] = module[49];
    }
    if (n == 5) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_50"] = module[50];
      jsonDoc["cell_51"] = module[51];
      jsonDoc["cell_52"] = module[52];
      jsonDoc["cell_53"] = module[53];
      jsonDoc["cell_54"] = module[54];
      jsonDoc["cell_55"] = module[55];
      jsonDoc["cell_56"] = module[56];
      jsonDoc["cell_57"] = module[57];
      jsonDoc["cell_58"] = module[58];
      jsonDoc["cell_59"] = module[59];
    }
    if (n == 6) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_60"] = module[60];
      jsonDoc["cell_61"] = module[61];
      jsonDoc["cell_62"] = module[62];
      jsonDoc["cell_63"] = module[63];
      jsonDoc["cell_64"] = module[64];
      jsonDoc["cell_65"] = module[65];
      jsonDoc["cell_66"] = module[66];
      jsonDoc["cell_67"] = module[67];
      jsonDoc["cell_68"] = module[68];
      jsonDoc["cell_69"] = module[69];
    }
    if (n == 7) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_70"] = module[70];
      jsonDoc["cell_71"] = module[71];
      jsonDoc["cell_72"] = module[72];
      jsonDoc["cell_73"] = module[73];
      jsonDoc["cell_74"] = module[74];
      jsonDoc["cell_75"] = module[75];
      jsonDoc["cell_76"] = module[76];
      jsonDoc["cell_77"] = module[77];
      jsonDoc["cell_78"] = module[78];
      jsonDoc["cell_79"] = module[79];
    }
    if (n == 8) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_80"] = module[80];
      jsonDoc["cell_81"] = module[81];
      jsonDoc["cell_82"] = module[82];
      jsonDoc["cell_83"] = module[83];
      jsonDoc["cell_84"] = module[84];
      jsonDoc["cell_85"] = module[85];
      jsonDoc["cell_86"] = module[86];
      jsonDoc["cell_87"] = module[87];
      jsonDoc["cell_88"] = module[88];
      jsonDoc["cell_89"] = module[89];
    }
    if (n == 9) {
      jsonDoc["ten_num"] = n;
      jsonDoc["cell_90"] = module[90];
      jsonDoc["cell_91"] = module[91];
      jsonDoc["cell_92"] = module[92];
      jsonDoc["cell_93"] = module[93];
      jsonDoc["cell_94"] = module[94];
      jsonDoc["cell_95"] = module[95];
      jsonDoc["cell_96"] = module[96];
      jsonDoc["cell_97"] = module[97];
    }



    // JSON 데이터를 문자열로 변환
    char jsonBuffer[200];
    serializeJson(jsonDoc, jsonBuffer);

    // 서버에 연결
    if (client.connect(server, port)) {
      Serial.println("server conn");

      // HTTP POST 요청 전송
      client.println("POST /cars/cellVoltage HTTP/1.1");
      client.println("Host: 3.39.234.31:4000");
      client.println("Content-Type: application/json");
      client.print("Content-Length: ");
      client.println(strlen(jsonBuffer));
      client.println();
      client.println(jsonBuffer);

      // 서버 응답 읽기
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          Serial.print(c);
        }
      }

      // 연결 종료
      client.stop();
      Serial.println("end");
    } else {
      Serial.println("cell data server con fail");
    }
  }
}



void sendDataToServer_ChargeState() {  // 서버로 JSON 데이터 전송 함수, 4개 모듈별 온도

  mylng = 126.864887;
  mylat = 37.600545;

  StaticJsonDocument<200>
    jsonDoc;
  jsonDoc["device_number"] = "888777";  //디바이스의 고유넘버, 이것과 아이디를 연동하여 사용한다.
  jsonDoc["charging_percent"] = State_of_Charge_BMS;
  jsonDoc["battery_power"] = Battery_Power;  //kw수(충전기, 방전속도)
  jsonDoc["charging"] = charge_state;
  jsonDoc["longitude"] = mylng;
  jsonDoc["latitude"] = mylat;


  // JSON 데이터를 문자열로 변환
  char jsonBuffer[200];
  serializeJson(jsonDoc, jsonBuffer);

  // 서버에 연결
  if (client.connect(server, port)) {
    Serial.println("server con complete");

    // HTTP POST 요청 전송
    client.println("POST /cars/batteryStatus HTTP/1.1");
    client.println("Host: 3.39.234.31:4000");
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(strlen(jsonBuffer));
    client.println();
    client.println(jsonBuffer);

    // 서버 응답 읽기
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.print(c);
      }
    }

    // 연결 종료
    client.stop();
    Serial.println("end");
  } else {
    Serial.println("charge state server con fail");
  }
}