#include <SPI.h>
#include <mcp2515.h>

struct can_frame read_can_msg;
struct can_frame write_can_msg;
MCP2515 mcp2515(10);

//아래변수들은 시뮬레이션을 위한 변수들
int charge_state = 0, cell_1 = 150, cell_2 = 150, battemp = 20;


void setup() {
  Serial.begin(9600);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {

  if (Serial.available()) {  //시리얼 모니터를 통해 차량의 상황을 시뮬레이션
    Serial.println("Serial com active, CarSim controler");
    Serial.println("-------------OPTIONS-------------");
    Serial.println("1 : Situation BAT TMEP_HIGH");
    Serial.println("2 : Situation BAT TMEP CRIT");
    Serial.println("3 : Situation Cell balance critical");
    Serial.println("4 : Situation ChargeState_ON");
    Serial.println("5 : Situation ChargeState OFF");
    Serial.println("6 : Reset");

    int value = Serial.parseInt();
    if (value == 1) {
      Serial.println("1 : Situation BAT TMEP_HIGH");  //배터리 모듈의 온도가 높아진 상황을 가정함
      battemp = 40;

      Serial.println("Update complete");

    } else if (value == 2) {
      Serial.println("2 : Situation BAT TMEP_CIRT");  //배터리 모듈의 온도가 높아진 상황을 가정함
      battemp = 60;

      Serial.println("Update complete");

    } 
    
    else if (value == 3) {
      Serial.println("3 : Situation Cell balance critical");  // 셀 밸런스가 틀어진 상황을 가정함 (1~2개 셀의 volt가 엇나간 경우)
      cell_1 = 124;
      cell_2 = 122;

      Serial.println("Update complete");
    } else if (value == 4) {
      Serial.println("4 : Situation ChargeState_ON");  // 충전중인 상황을 가정함
      charge_state = 128;

      Serial.println("Update complete");

    } else if (value == 5) {
      Serial.println("5 : Situation ChargeState OFF");  // 충전중에 충전기가 분리된 상황을 가정함
      charge_state = 0;

      Serial.println("Update complete");

    } else if (value == 5) {
      Serial.println("5 : Reset");  // 실험종료, 모든 변수를 리셋
      charge_state = 0;
      cell_1 = 150;
      cell_2 = 150;
      battemp = 20;
      Serial.println("Update complete");
    }
  }


  if (mcp2515.readMessage(&read_can_msg) == MCP2515::ERROR_OK) {
    memset(&write_can_msg, 0, sizeof(write_can_msg));
    int potent_rpm = analogRead(A0);
    unsigned char p_id = read_can_msg.data[2];


    Serial.print("요청된 PID: ");
    Serial.println((int)read_can_msg.data[2], HEX);  //22인경우 확장 PID
    int a = read_can_msg.data[4];
    Serial.println((int)read_can_msg.data[4], HEX);  //여기로 확장 PID 구분





    switch (p_id) {
      case 0:                         //PID 0번이면 가용 가능한 목차 전송
        write_can_msg.can_id = 2024;  //ECU ID넘버
        write_can_msg.can_dlc = 8;    //바이트 길이수
        write_can_msg.data[0] = 6;    //길이
        write_can_msg.data[1] = 65;   // 서비스 번호
        write_can_msg.data[2] = 0;    //pid
        write_can_msg.data[3] = 255;  //데이터(속도 RPM등)
        write_can_msg.data[4] = 255;
        write_can_msg.data[5] = 255;
        write_can_msg.data[6] = 255;
        write_can_msg.data[7] = 170;

        mcp2515.sendMessage(&write_can_msg);
        Serial.println("사용가능한 리스트요청받음");
        break;

      case 12:
        write_can_msg.can_id = 2024;
        write_can_msg.can_dlc = 8;
        write_can_msg.data[0] = 4;
        write_can_msg.data[1] = 65;
        write_can_msg.data[2] = 12;
        write_can_msg.data[3] = (long)(potent_rpm * 4) / 256;
        write_can_msg.data[4] = (long)(potent_rpm * 4) % 256;
        write_can_msg.data[5] = 170;
        write_can_msg.data[6] = 170;
        write_can_msg.data[7] = 170;

        mcp2515.sendMessage(&write_can_msg);
        Serial.println("rpm 값 전송");
        break;


      default:
        break;
    }

    if ((22, HEX) == ((int)read_can_msg.data[2], HEX)) {  // 22인경우는 확장 PID, 배터리 관련 데이터 전달
      int a = read_can_msg.data[4];

      if (a == 3) {  //03는 셀별 전압
        delay(10);

        write_can_msg.can_id = 2024;  //ECU ID넘버
        write_can_msg.can_dlc = 8;    //바이트 길이수
        write_can_msg.data[0] = 16;   //0x10 (256*1)
        //첫 번째 바이트: 0x1X 형태로, 여기서 X는 데이터의 총 길이의 첫 두 자리(상위 4비트)입니다. 예를 들어, 총 데이터 길이가 16바이트라면 첫 번째 바이트는 0x10이 됩니다.
        write_can_msg.data[1] = 0;  //두번째 바이트는 0이므로 총 데이터 길이는 256 바이트 -> 데이터 프레임을 32번 보내야됨
        // 첫 프레임(First Frame)에서 두 번째 바이트는 전체 데이터의 총 길이의 하위 8비트를 나타냅니다.
        write_can_msg.data[2] = 0;    // 더미
        write_can_msg.data[3] = 240;  // 더미
        write_can_msg.data[4] = 230;  // 더미
        write_can_msg.data[5] = 222;  //a
        write_can_msg.data[6] = 210;  //b
        write_can_msg.data[7] = 101;  //c

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 33;  //21 첫번째 프레임 의미 14바이트 전송
        //첫 번째 바이트: 0x2N 형태로, 여기서 N은 순차적인 프레임 번호(1~F)입니다. 첫 후속 프레임은 0x21, 그 다음은 0x22, 이런 식으로 증가
        //이후 바이트들은 그냥 데이터
        write_can_msg.data[1] = 10;   //d
        write_can_msg.data[2] = 151;  //e cell33
        write_can_msg.data[3] = 152;  //f cell34
        write_can_msg.data[4] = 153;  //g cell35
        write_can_msg.data[5] = 150;  //h cell36
        write_can_msg.data[6] = 154;  //i cell37
        write_can_msg.data[7] = 155;  //j cell38

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 34;      //두번째 프레임 22바이트 전송
        write_can_msg.data[1] = 150;     //k cell39
        write_can_msg.data[2] = 152;     //l cell40
        write_can_msg.data[3] = 155;     //m cell41
        write_can_msg.data[4] = 155;  //n	cell42
        write_can_msg.data[5] = 154;     //o cell43
        write_can_msg.data[6] = 153;     //p cell44
        write_can_msg.data[7] = 150;  //q cell45

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 35;   //세번째 프레임 30바이트 전송
        write_can_msg.data[1] = 154;  //r cell46
        write_can_msg.data[2] = 153;  //s cell47
        write_can_msg.data[3] = 156;  //t cell48
        write_can_msg.data[4] = 155;  //u cell49
        write_can_msg.data[5] = 153;  //v cell50
        write_can_msg.data[6] = 154;  //w cell51
        write_can_msg.data[7] = 152;  //x cell52

        mcp2515.sendMessage(&write_can_msg);
        delay(100);


        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 36;   //네번째 프레임 38바이트 전송
        write_can_msg.data[1] = 152;  //y cell53
        write_can_msg.data[2] = 151;  //z cell54
        write_can_msg.data[3] = 153;  //aa cell55
        write_can_msg.data[4] = 155;  //ab cell56
        write_can_msg.data[5] = 155;  //ac cell57
        write_can_msg.data[6] = 153;  //ad cell58
        write_can_msg.data[7] = 152;  //ae cell59

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        write_can_msg.data[0] = 37;   //다섯번째 프레임 38바이트 전송
        write_can_msg.data[1] = 153;  //af cell60
        write_can_msg.data[2] = 154;  //ag cell61
        write_can_msg.data[3] = 153;  //ah cell62
        write_can_msg.data[4] = 152;  //ai cell63
        write_can_msg.data[5] = 152;  //aj cell64
        write_can_msg.data[6] = 255;  //ak
        write_can_msg.data[7] = 170;  //al

        mcp2515.sendMessage(&write_can_msg);

        Serial.println("셀 별 전압 전송완료");
      }

      if (a == 4) {  //03는 셀별 전압

        delay(10);

        write_can_msg.can_id = 2024;  //ECU ID넘버
        write_can_msg.can_dlc = 8;    //바이트 길이수
        write_can_msg.data[0] = 16;   //0x10 (256*1)
        //첫 번째 바이트: 0x1X 형태로, 여기서 X는 데이터의 총 길이의 첫 두 자리(상위 4비트)입니다. 예를 들어, 총 데이터 길이가 16바이트라면 첫 번째 바이트는 0x10이 됩니다.
        write_can_msg.data[1] = 0;  //두번째 바이트는 0이므로 총 데이터 길이는 256 바이트 -> 데이터 프레임을 32번 보내야됨
        // 첫 프레임(First Frame)에서 두 번째 바이트는 전체 데이터의 총 길이의 하위 8비트를 나타냅니다.
        write_can_msg.data[2] = 0;    // 더미
        write_can_msg.data[3] = 240;  // 더미
        write_can_msg.data[4] = 230;  // 더미
        write_can_msg.data[5] = 222;  //a
        write_can_msg.data[6] = 210;  //b
        write_can_msg.data[7] = 102;  //c

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 33;  //21 첫번째 프레임 의미 14바이트 전송
        //첫 번째 바이트: 0x2N 형태로, 여기서 N은 순차적인 프레임 번호(1~F)입니다. 첫 후속 프레임은 0x21, 그 다음은 0x22, 이런 식으로 증가
        //이후 바이트들은 그냥 데이터
        write_can_msg.data[1] = 10;   //d
        write_can_msg.data[2] = 156;  //e cell65
        write_can_msg.data[3] = 155;  //f cell66
        write_can_msg.data[4] = 152;  //g cell67
        write_can_msg.data[5] = 151;  //h cell68
        write_can_msg.data[6] = 153;  //i cell69
        write_can_msg.data[7] = 151;  //j cell70

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 34;   //두번째 프레임 22바이트 전송
        write_can_msg.data[1] = 153;  //k cell71
        write_can_msg.data[2] = 152;  //l cell72
        write_can_msg.data[3] = 155;  //m cell73
        write_can_msg.data[4] = 154;  //n	cell74
        write_can_msg.data[5] = 153;  //o cell75
        write_can_msg.data[6] = 151;  //p cell76
        write_can_msg.data[7] = 152;  //q cell77

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 35;   //세번째 프레임 30바이트 전송
        write_can_msg.data[1] = 152;  //r cell78
        write_can_msg.data[2] = 155;  //s cell79
        write_can_msg.data[3] = 155;  //t cell80
        write_can_msg.data[4] = 154;  //u cell81
        write_can_msg.data[5] = 153;  //v cell82
        write_can_msg.data[6] = 151;  //w cell83
        write_can_msg.data[7] = 152;  //x cell84

        mcp2515.sendMessage(&write_can_msg);
        delay(100);


        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 36;   //네번째 프레임 38바이트 전송
        write_can_msg.data[1] = 155;  //y cell85
        write_can_msg.data[2] = 154;  //z cell86
        write_can_msg.data[3] = 154;  //aa cell87
        write_can_msg.data[4] = 151;  //ab cell88
        write_can_msg.data[5] = 154;  //ac cell89
        write_can_msg.data[6] = 153;  //ad cell90
        write_can_msg.data[7] = 155;  //ae cell91

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        write_can_msg.data[0] = 37;   //다섯번째 프레임 38바이트 전송
        write_can_msg.data[1] = 153;  //af cell92
        write_can_msg.data[2] = 153;  //ag cell93
        write_can_msg.data[3] = cell_1;  //ah cell94
        write_can_msg.data[4] = 154;  //ai cell95
        write_can_msg.data[5] = cell_2;  //aj cell96
        write_can_msg.data[6] = 255;  //ak
        write_can_msg.data[7] = 170;  //al

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        Serial.println("셀 별 전압 전송완료");
      }

      if (a == 5) {  //03는 셀별 전압
        delay(10);

        write_can_msg.can_id = 2024;  //ECU ID넘버
        write_can_msg.can_dlc = 8;    //바이트 길이수
        write_can_msg.data[0] = 16;   //0x10 (256*1)
        //첫 번째 바이트: 0x1X 형태로, 여기서 X는 데이터의 총 길이의 첫 두 자리(상위 4비트)입니다. 예를 들어, 총 데이터 길이가 16바이트라면 첫 번째 바이트는 0x10이 됩니다.
        write_can_msg.data[1] = 0;  //두번째 바이트는 0이므로 총 데이터 길이는 256 바이트 -> 데이터 프레임을 32번 보내야됨
        // 첫 프레임(First Frame)에서 두 번째 바이트는 전체 데이터의 총 길이의 하위 8비트를 나타냅니다.
        write_can_msg.data[2] = 0;    // 더미
        write_can_msg.data[3] = 240;  // 더미
        write_can_msg.data[4] = 230;  // 더미
        write_can_msg.data[5] = 222;  //a
        write_can_msg.data[6] = 210;  //b
        write_can_msg.data[7] = 103;  //c

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 33;  //21 첫번째 프레임 의미 14바이트 전송
        //첫 번째 바이트: 0x2N 형태로, 여기서 N은 순차적인 프레임 번호(1~F)입니다. 첫 후속 프레임은 0x21, 그 다음은 0x22, 이런 식으로 증가
        //이후 바이트들은 그냥 데이터
        write_can_msg.data[1] = 10;   //d
        write_can_msg.data[2] = 20;   //e
        write_can_msg.data[3] = 30;   //f
        write_can_msg.data[4] = 2;    //g
        write_can_msg.data[5] = 2;    //h
        write_can_msg.data[6] = 1;    //i
        write_can_msg.data[7] = 128;  //j

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 34;   //두번째 프레임 22바이트 전송
        write_can_msg.data[1] = 2;    //k
        write_can_msg.data[2] = 2;    //l
        write_can_msg.data[3] = 255;  //m
        write_can_msg.data[4] = 40;   //n
        write_can_msg.data[5] = 50;   //o
        write_can_msg.data[6] = 200;  //p
        write_can_msg.data[7] = 20;   //q

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 35;   //세번째 프레임 30바이트 전송
        write_can_msg.data[1] = -20;  //r
        write_can_msg.data[2] = 10;   //s
        write_can_msg.data[3] = 20;   //t
        write_can_msg.data[4] = 255;  //u
        write_can_msg.data[5] = 255;  //v
        write_can_msg.data[6] = 255;  //w
        write_can_msg.data[7] = 170;  //x

        mcp2515.sendMessage(&write_can_msg);
        delay(100);


        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 36;   //네번째 프레임 38바이트 전송
        write_can_msg.data[1] = 20;   //y
        write_can_msg.data[2] = 0;    //z
        write_can_msg.data[3] = 222;  //aa
        write_can_msg.data[4] = 255;  //ab
        write_can_msg.data[5] = 255;  //ac
        write_can_msg.data[6] = 255;  //ad
        write_can_msg.data[7] = 170;  //ae

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        write_can_msg.data[0] = 37;   //다섯번째 프레임 38바이트 전송
        write_can_msg.data[1] = 20;   //af
        write_can_msg.data[2] = 0;    //ag
        write_can_msg.data[3] = 222;  //ah
        write_can_msg.data[4] = 150;  //ai cell97
        write_can_msg.data[5] = 150;  //aj cell98
        write_can_msg.data[6] = 255;  //ak
        write_can_msg.data[7] = 170;  //al

        mcp2515.sendMessage(&write_can_msg);

        Serial.println("셀 별 전압 전송완료");
      }

      else if (2 == a) {  //02는 셀별 전압

        delay(10);

        write_can_msg.can_id = 2024;  //ECU ID넘버
        write_can_msg.can_dlc = 8;    //바이트 길이수
        write_can_msg.data[0] = 16;   //0x10 (256*1)
        //첫 번째 바이트: 0x1X 형태로, 여기서 X는 데이터의 총 길이의 첫 두 자리(상위 4비트)입니다. 예를 들어, 총 데이터 길이가 16바이트라면 첫 번째 바이트는 0x10이 됩니다.
        write_can_msg.data[1] = 0;  //두번째 바이트는 0이므로 총 데이터 길이는 256 바이트 -> 데이터 프레임을 32번 보내야됨
        // 첫 프레임(First Frame)에서 두 번째 바이트는 전체 데이터의 총 길이의 하위 8비트를 나타냅니다.
        write_can_msg.data[2] = 0;    // 더미
        write_can_msg.data[3] = 240;  // 더미
        write_can_msg.data[4] = 230;  // 더미
        write_can_msg.data[5] = 222;  //a
        write_can_msg.data[6] = 210;  //b
        write_can_msg.data[7] = 104;  //c

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 33;  //21 첫번째 프레임 의미 14바이트 전송
        //첫 번째 바이트: 0x2N 형태로, 여기서 N은 순차적인 프레임 번호(1~F)입니다. 첫 후속 프레임은 0x21, 그 다음은 0x22, 이런 식으로 증가
        //이후 바이트들은 그냥 데이터
        write_can_msg.data[1] = 10;   //d
        write_can_msg.data[2] = 150;  //e cell00 계산식) e/50, 값은 2.8~4.2
        write_can_msg.data[3] = 154;  //f cell01
        write_can_msg.data[4] = 153;  //g cell02
        write_can_msg.data[5] = 154;  //h cell03
        write_can_msg.data[6] = 154;  //i cell04
        write_can_msg.data[7] = 153;  //j cell05

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 34;   //두번째 프레임 22바이트 전송
        write_can_msg.data[1] = 154;  //k cell06
        write_can_msg.data[2] = 153;  //l cell07
        write_can_msg.data[3] = 152;  //m cell08
        write_can_msg.data[4] = 153;  //n	cell09
        write_can_msg.data[5] = 154;  //o cell10
        write_can_msg.data[6] = 154;  //p cell11
        write_can_msg.data[7] = 153;  //q cell12

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 35;   //세번째 프레임 30바이트 전송
        write_can_msg.data[1] = 152;  //r cell13
        write_can_msg.data[2] = 153;  //s cell14
        write_can_msg.data[3] = 154;  //t cell15
        write_can_msg.data[4] = 155;  //u cell16
        write_can_msg.data[5] = 153;  //v cell17
        write_can_msg.data[6] = 150;  //w cell18
        write_can_msg.data[7] = 152;  //x cell19

        mcp2515.sendMessage(&write_can_msg);
        delay(100);


        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 36;   //네번째 프레임 38바이트 전송
        write_can_msg.data[1] = 152;  //y cell20
        write_can_msg.data[2] = 153;  //z cell21
        write_can_msg.data[3] = 155;  //aa cell22
        write_can_msg.data[4] = 152;  //ab cell23
        write_can_msg.data[5] = 153;  //ac cell24
        write_can_msg.data[6] = 154;  //ad cell25
        write_can_msg.data[7] = 153;  //ae cell26

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        write_can_msg.data[0] = 37;   //다섯번째 프레임 38바이트 전송
        write_can_msg.data[1] = 155;  //af cell27
        write_can_msg.data[2] = 155;  //ag cell28
        write_can_msg.data[3] = 154;  //ah cell29
        write_can_msg.data[4] = 152;  //ai cell30
        write_can_msg.data[5] = 153;  //aj cell31
        write_can_msg.data[6] = 255;  //ak
        write_can_msg.data[7] = 170;  //al

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        Serial.println("셀 별 전압 전송완료");
      }

      else if (1 == a) {  //확장 PID 구분자가 1인경우
        delay(10);


        write_can_msg.can_id = 2024;  //ECU ID넘버
        write_can_msg.can_dlc = 8;    //바이트 길이수
        write_can_msg.data[0] = 16;   //0x10 (256*1)
        //첫 번째 바이트: 0x1X 형태로, 여기서 X는 데이터의 총 길이의 첫 두 자리(상위 4비트)입니다. 예를 들어, 총 데이터 길이가 16바이트라면 첫 번째 바이트는 0x10이 됩니다.
        write_can_msg.data[1] = 0;  //두번째 바이트는 0이므로 총 데이터 길이는 256 바이트 -> 데이터 프레임을 32번 보내야됨
        // 첫 프레임(First Frame)에서 두 번째 바이트는 전체 데이터의 총 길이의 하위 8비트를 나타냅니다.
        write_can_msg.data[2] = 0;    // 더미
        write_can_msg.data[3] = 240;  // 더미
        write_can_msg.data[4] = 230;  // 더미
        write_can_msg.data[5] = 222;  //a
        write_can_msg.data[6] = 210;  //b
        write_can_msg.data[7] = 105;  //c

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 33;  //21 첫번째 프레임 의미 14바이트 전송
        //첫 번째 바이트: 0x2N 형태로, 여기서 N은 순차적인 프레임 번호(1~F)입니다. 첫 후속 프레임은 0x21, 그 다음은 0x22, 이런 식으로 증가
        //이후 바이트들은 그냥 데이터
        write_can_msg.data[1] = 10;            //d
        write_can_msg.data[2] = 50;            //e -> e/2 해서 % 로 나누면 현재 배터리 잔량 % State_of_Charge_BMS
        write_can_msg.data[3] = 30;            //f
        write_can_msg.data[4] = 2;             //g
        write_can_msg.data[5] = 2;             //h
        write_can_msg.data[6] = 1;             // i
        write_can_msg.data[7] = charge_state;  //j -> 충전중인지 아닌지 판별 1000 0000 <- {J:7} 128이면 충전중 charge_state

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 34;       //두번째 프레임 22바이트 전송
        write_can_msg.data[1] = 2;        //k
        write_can_msg.data[2] = 2;        //l
        write_can_msg.data[3] = 255;      //m
        write_can_msg.data[4] = 40;       //n
        write_can_msg.data[5] = 50;       //o
        write_can_msg.data[6] = 200;      //p
        write_can_msg.data[7] = battemp;  //q Signed(Q) bat moudle 01

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 35;   //세번째 프레임 30바이트 전송
        write_can_msg.data[1] = 20;   //r Signed(r) bat moudle 02
        write_can_msg.data[2] = 10;   //s Signed(s) bat moudle 03
        write_can_msg.data[3] = 20;   //t Signed(t) bat moudle 04
        write_can_msg.data[4] = 255;  //u
        write_can_msg.data[5] = 255;  //v
        write_can_msg.data[6] = 255;  //w
        write_can_msg.data[7] = 170;  //x

        mcp2515.sendMessage(&write_can_msg);
        delay(100);


        //write_can_msg.can_id  = 2024; //ECU ID넘버
        //write_can_msg.can_dlc = 8; //바이트 길이수
        write_can_msg.data[0] = 36;   //네번째 프레임 38바이트 전송
        write_can_msg.data[1] = 20;   //y
        write_can_msg.data[2] = 0;    //z
        write_can_msg.data[3] = 222;  //aa
        write_can_msg.data[4] = 255;  //ab
        write_can_msg.data[5] = 255;  //ac
        write_can_msg.data[6] = 255;  //ad
        write_can_msg.data[7] = 170;  //ae

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        write_can_msg.data[0] = 37;   //다섯번째 프레임 38바이트 전송
        write_can_msg.data[1] = 20;   //af
        write_can_msg.data[2] = 0;    //ag
        write_can_msg.data[3] = 222;  //ah
        write_can_msg.data[4] = 255;  //ai
        write_can_msg.data[5] = 255;  //aj
        write_can_msg.data[6] = 255;  //ak
        write_can_msg.data[7] = 170;  //al

        mcp2515.sendMessage(&write_can_msg);
        delay(100);

        Serial.println("01 번 PID 전송 완료");
      }
    }
  }
}