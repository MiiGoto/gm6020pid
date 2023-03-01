#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

int led = 13;
FlexCAN CANTransmitter(1000000);
static CAN_message_t msg1;
static CAN_message_t rxmsg;
int flag = 0;
static int fireflag = 0, prefireflag = 0;
int count = 0;
void setup() {
  CANTransmitter.begin();
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
}

static int yawraw, yaw, pitchraw, pitch;
static unsigned long testch[6];

void loop() {
  int val = 1050;
  static int data[18];                       //入力データが入る？
  static int dataNumber = 0;                 //入力データの数(Serial1.available()の返値)
  static unsigned long lastConnectTime = 0;  //直前の通信の時間?
  static int preyaw = 5250, prepitch = 6985;
  int yawin = 4000;
  float yawout, pitchout;
  int limitSpeed = 0, u[4] = {};
  //0以下は0に100以上は100に
  //走行用モータ制限速度:-16,384～0～16,384(0xC000～0x4000)
  limitSpeed = 16384;
  //0~100を0~16384に変換

  //Serial.print("Sending: ");
  int roll = 0;
  testch[4] = (data[5] & 0xC0) >> 6;
  testch[5] = (data[5] & 0x30) >> 4;
  if (testch[4] == 3) roll = 0;
  else if (testch[4] == 1) roll = 1;
  else if (testch[4] == 2) roll = -1;

  if (Serial1.available() > 0) {
    for (int dataNum = Serial1.available(); dataNum > 0; dataNum--) {
      if (dataNumber < 0) {
        Serial1.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = Serial1.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      } else if (dataNumber == 18) {
        testch[0] = (((data[1] & 0x07) << 8) | data[0]);                           //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));                    //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6));  //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));                    //ch3(364～1024～1684)
        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);                           //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3));                    //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6));  //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1));                    //ch3(364～1024～1684)
            if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
              dataNumber = -i;
              break;
            }
          }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        } else {
          dataNumber = 0;
        }
      }
    }
  }
  digitalWrite(led, !digitalRead(led));
  for (int i = 0; i < 4; i++) u[i] = 0;


  msg1.id = 0x1FF;
  msg1.len = 8;
  yaw = yawraw;
  prepitch = pitch;
  preyaw = yaw;
  Serial.print(yawraw);
  Serial.print(",");
  Serial.println(/*yawraw*/ yawin);
  yawout = yawPID(yawin, yaw);
  //Serial.println(yawout);
  u[0] = yawout;
  for (int i = 0; i < msg1.len; i++) {
    u[i] = max(-16384, min(16384, u[i]));
    msg1.buf[i * 2] = u[i] >> 8;
    msg1.buf[i * 2 + 1] = u[i] & 0x00FF;
  }
  delay(50);
}

void timerInt() {
  CANTransmitter.write(msg1);
  count++;
  while (CANTransmitter.read(rxmsg)) {
    if (rxmsg.id == 0x205) {
      yawraw = rxmsg.buf[0] * 256 + rxmsg.buf[1];
    }
    if (rxmsg.id == 0x206) {
      pitchraw = rxmsg.buf[0] * 256 + rxmsg.buf[1];
    }
  }
}

int pitchPID(int pitchCommand, int pitchValue) {
  const float pgain[3] = { 50, 0, 0 };
  static float pIe, prepPe;
  float pPe = pitchCommand - pitchValue;
  pIe += pPe;
  float pDe = pPe - prepPe;
  prepPe = pPe;
  return pgain[0] * pPe + pgain[1] * pIe + pgain[2] * pDe;
}

int yawPID(int yawCommand, int yawValue) {
  const float ygain[3] = { 9, 0, 0 };
  static float yIe, preyPe;
  float yPe = yawCommand - yawValue;
  if(yPe>4096)yPe=yPe-8192;
  if(yPe<-4096)yPe=yPe+8192;
  yIe += yPe;
  float yDe = yPe - preyPe;
  preyPe = yPe;
  return ygain[0] * yPe + ygain[1] * yIe + ygain[2] * yDe;
}
