/*
   Development: Mediaflow

  Arduino IDE 1.0.5 이상에서 사용을 권장합니다.
  
  temperature humidity sensor(AM2305 Digital temperature and humidity sensor)
  http://mediaflow.cafe24.com/product/detail.html?product_no=334&cate_no=93&display_group=1
  library : https://github.com/adafruit/DHT-sensor-library
  ex) c:\Arduino\libraries\libraryfile
  
  co2 sensor(T6613, Single channel)
  http://makezone.co.kr/product/detail.html?product_no=339&cate_no=93&display_group=
  
  light sensor(BH1750)
  http://makezone.co.kr/product/detail.html?product_no=231&cate_no=88&display_group=
  sample code : http://www.dfrobot.com/image/data/SEN0097/BH1750_Sample.rar
  
  relay(HR-CR3A13-DC12H)
  http://makezone.co.kr/product/detail.html?product_no=340&cate_no=94&display_group=1

  
*/



#include <MediafarmLib.h>

#include "DHT.h" //dht library
#include <Wire.h> //BH1750 IIC Mode 
#include <math.h> //for light sensor


// dht libraries
DHT dht1(DHTPIN1, DHTTYPE); 
DHT dht2(DHTPIN2, DHTTYPE); 
DHT dht3(DHTPIN3, DHTTYPE); 
DHT dht4(DHTPIN4, DHTTYPE); 

char receiveDatas[SERIAL_READ_BYTE]; // receive datas

byte buff[I2C_BUS]; //light sensor value array

// pin definition
int digitalPins[DIGITAL_PIN_COUNT] = {11,10,9,8};
//int analogPins[ANALOG_PIN_COUNT] = {A0,A1,A2,A3,A4,A5};
int analogPins[ANALOG_PIN_COUNT] = {14,15,16,17,18,19};

// response length
byte RespLen = 0;

// values reset
byte temper[DIGITAL_PIN] = {0,0,0,0};
byte humi[DIGITAL_PIN] = {0,0,0,0};
int analog = 0;
byte anal[SENSOR_BYTE]={0,0};
byte light_conv[SENSOR_BYTE]={0,0};

MediafarmLib mediafarmLib;

void setup(){
  mediafarmLib.setArduinoConfig();
  Serial.begin(BAUD_RATE1);

  pinMode(ENABLE_PIN, OUTPUT);
  pinSetting();
  
  if(mediafarmLib.arduino_mode == SENSOR_MODE)
    Wire.begin();

}

void loop() {
//  if(mediafarmLib.arduino_mode == SENSOR_MODE)
      //주기적으로 센서값 갱신
      updateSensorData();
}


//시리얼 이벤트시 발동
void serialEvent()
{
  if (Serial.available()) 
  {
    //시리얼 정보를 byte 배열로 저장
    Serial.readBytes(receiveDatas, SERIAL_READ_BYTE);

    RespLen = 0;
   //전달 받은 프로토콜에서 센서 갯수 정보를 취득
    char sensor[receiveDatas[DATA_LENGTH]];
    int sizearray = PROTOCOL_HEADER+receiveDatas[DATA_LENGTH]*SENSOR_BYTE;

    byte last[sizearray];

    int crc_length = ((int)receiveDatas[DATA_LENGTH])+PROTOCOL_HEADER+CRC;      
    boolean flag = validationCheck(receiveDatas, crc_length); //유효성 검사

    if(flag)
    {
//      Serial.print("receiveDatas[MSG_MODE]: ");
//            Serial.println(receiveDatas[MSG_MODE]);

        //sensor
        //딥스위치와 입력신호 유효성 검사
        if((receiveDatas[MSG_MODE] == SENSOR_MODE)&&(receiveDatas[REQ_TYPE] == PROTOCOL_RESP_SENSOR))
          sensorWork(sensor, last, sizearray);
        
        //control
        if((receiveDatas[MSG_MODE] == CONTROL_MODE)&&(receiveDatas[REQ_TYPE] == PROTOCOL_RESP_CONTROL))
        {
//          Serial.println("(receiveDatas[MSG_MODE] == CONTROL_MODE)&&(receiveDatas[REQ_TYPE] == PROTOCOL_RESP_CONTROL)1!!!");
          controlWork(sensor);
        }
      }
    }
  }



// Sensor 동작
void sensorWork(char _sensor[], byte _send[], int _sizearray)
{
  //_sensor : sensor information
  //_send : response data array
  //_sizearray: _send array size
  
  //센서 타입 스캔후 각 센서당 data byte size 할당
  for(int i=0;i<receiveDatas[DATA_LENGTH];i++){    
    _sensor[i]=receiveDatas[PROTOCOL_HEADER+i];
    //프로토콜 데이터 부분에서 센서 요청 정보를 추출
    //sensor length assignment(byte)
    if(_sensor[i] == ANALOG)//analog
      RespLen += ANALOG_BYTE;
    
    if(_sensor[i] == DIGITAL_BUS)//digital bus
      RespLen += DIGITAL_BUS_BYTE;
    
    if(_sensor[i] == DIGITAL_IO)//digital I/O(not use)
      RespLen += DIGITAL_IO_BYTE;
    
    if(_sensor[i] == I2C_BUS)//I2C
      RespLen += I2C_BYTE;
  }

  
  byte count = 0;
  
//Protocol Header를 전송할 배열에 입력
  _send[MSG_ADDR] = mediafarmLib.arduino_id;
  _send[MSG_MODE] = mediafarmLib.arduino_mode;
  _send[REQ_TYPE] = mediafarmLib.response_sensor;
  _send[DATA_LENGTH] = RespLen;

//센서값읋 Protocol에 맞게 변환 int형 데이더를 byte형으로 변환
  for(int i=0;i<receiveDatas[DATA_LENGTH];i++){
    byte a= PROTOCOL_HEADER+(i*SENSOR_BYTE);
    
    if(_sensor[i] == PROTOCOL_ANALOG)//analog
    {
      _send[a]=anal[0];
      _send[a+1]=anal[1];
    }

    else if(_sensor[i] == PROTOCOL_DIGITAL_BUS)//digital bus
    { 
      _send[a]=temper[count];
      _send[a+1]=humi[count];
      count++;
    }
    else if(_sensor[i] == PROTOCOL_I2C)//I2C 
    { 
      _send[a]=light_conv[0];
      _send[a+1]=light_conv[1];
    }
  }

  _send[PROTOCOL_HEADER+RespLen] = mediafarmLib.getCrcValue(_send, PROTOCOL_HEADER+RespLen);
  _send[PROTOCOL_HEADER+RespLen+SEPARATOR]='_';
  
  
  send485(_send,_sizearray+SEND_CRC);
}



//컨트롤 동작
void controlWork(char _sensor[])
{
//  
//  Serial.println("controlWork!!!!");
  //프로토콜 데이터 부분에서 컨트롤 요청 정보를 추출
  for(int i=0;i<receiveDatas[DATA_LENGTH];i++){
    _sensor[i]=receiveDatas[PROTOCOL_HEADER+i];
  }


  // relay command
  //아날로그핀과 디지털핀의 구분을 위해 사용
  for(int i=0;i < receiveDatas[DATA_LENGTH];i++){
    int ioValue = CONTROL_LOW;
    if(bitRead(_sensor[i],READ_CONTROL_INFORMATION) == CONTROL_HIGH)
      ioValue = CONTROL_HIGH;
    if(i < DECIDE_DIGITAL_PIN)
      digitalWrite(digitalPins[i],ioValue);    
    else if(i > DECIDE_ANALOG_PIN)
      //아날로그핀의 디지털핀화
      digitalWrite(analogPins[i-DECIDE_DIGITAL_PIN],ioValue);
    else
      digitalWrite(ENABLE_PIN,LOW);
  }


  // write 메세지의 충돌 방지
  if((receiveDatas[PROTOCOL_FIRST_DATA]==CONTROL_LOW)||(receiveDatas[PROTOCOL_FIRST_DATA]==CONTROL_HIGH)){
    byte success[] = {receiveDatas[MSG_ADDR], receiveDatas[MSG_MODE], PROTOCOL_RESP_CONTROL,NULL, '_'};

    success[SEND_SUCCESS] = mediafarmLib.getCrcValue(success, PROTOCOL_RESP_CONTROL);
//    Serial.print("send485!!!!");
    send485(success,SEND_SUCCESS+SEND_CRC);
  }
}

//아두이노 모드에 따라 I/O pinmode 설정
void pinSetting(){
  if(mediafarmLib.arduino_mode == CONTROL_MODE){
    for(int i=0;i<DIGITAL_PIN_COUNT;i++)
      pinMode(digitalPins[i],OUTPUT);    

    for(int i=0;i<ANALOG_PIN_COUNT;i++)
      pinMode(analogPins[i],OUTPUT);
  }
  else if(mediafarmLib.arduino_mode == SENSOR_MODE){
    dht1.begin();
    dht2.begin();
    dht3.begin();
    dht4.begin();
  }  
}



// 주기적으로 센서 값 업데이트
void updateSensorData()
{
  dhtSensor();

  analog = analogRead(analogPins[0]);

  // 아날로그 값을 가져오기 위한 수식
  int analog_map = map(analog, CO2_ANALOG_MIN, CO2_ANALOG_MAX, CO2_PPM_MIN, CO2_PPM_MAX);
  int analog_conv = constrain(analog_map, CO2_PPM_MIN, CO2_PPM_MAX);
  
  if(mediafarmLib.arduino_mode == SENSOR_MODE){
    lightInit();
    delay(LIGHT_SENSOR_DELAY1);

    if(LIGHT_SENSOR_READ==lightRead())
    {
      uint16_t light = ((buff[0]<<SHIFT8)|buff[1])/1.2;
      if(light > ONE_BYTE_SIZE){
        light_conv[0] = buff[0];
        light_conv[1] = buff[1];
      }
      else
      {
        light_conv[0] = NULL;
        light_conv[1] = buff[1];
      }
    }
    delay(LIGHT_SENSOR_DELAY2);

    if(analog_conv > ONE_BYTE_SIZE){
      anal[0] = (byte)((analog_conv>>8)&0xFF);
      anal[1] = (byte)(analog_conv&0xFF);
    }
    else
    {
      anal[0] = NULL;
      anal[1] = (byte)(analog_conv);
    }
  }
}


// receive 메세지의 유효성 체크
boolean validationCheck(char checkArray[], int lastlength){

  if(receiveDatas[MSG_ADDR] == mediafarmLib.arduino_id){
    if(receiveDatas[MSG_MODE] == mediafarmLib.arduino_mode){
      byte receive_check = 0;

      for(int i = 0;i<lastlength-1;i++)
        receive_check += checkArray[i];  

      if(receive_check==checkArray[lastlength-1])
        return true;
    } 
    else{
      byte fail[] = {mediafarmLib.arduino_id,mediafarmLib.arduino_mode,PROTOCOL_RESPONSE_ERROR,PROTOCOL_ERROR_NONRESPONSE,'_'};
      fail[SEND_FAIL] = mediafarmLib.getCrcValue(fail, SEND_FAIL);

      send485(fail,SEND_FAIL+SEND_CRC);
      return false;  
    }
  } 
  else{
    return false;       
  }
}


 //I2C Read
int lightRead() 
{
  int i=0;
  Wire.beginTransmission( mediafarmLib.LIGHT_SENSOR_ADDRESS);
  Wire.requestFrom(mediafarmLib.LIGHT_SENSOR_ADDRESS, LIGHT_SENSOR_READ);
  while(Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();  
  return i;
}

// I2C Init
void lightInit() 
{
  Wire.beginTransmission(mediafarmLib.LIGHT_SENSOR_ADDRESS);
  Wire.write(mediafarmLib.LIGHT_SENSOR_RESOLUTION);//1lx resolution 120ms
  Wire.endTransmission();
}

// 온습도 센서 값 업데이트
void dhtSensor(){
  //read all digital port dht senser
  if(mediafarmLib.arduino_mode == CONTROL_MODE){
    return;
  }
  temper[0] = dht1.readTemperature();
  humi[0] = dht1.readHumidity();
  temper[1] = dht2.readTemperature();
  humi[1] = dht2.readHumidity();
  temper[2] = dht3.readTemperature();
  humi[2] = dht3.readHumidity();
  temper[3] = dht4.readTemperature();
  humi[3] = dht4.readHumidity();
}


//RS485 메세지 전송
void send485(byte _send[],int arraysize){
    digitalWrite(ENABLE_PIN,HIGH);
    delay(SEND_DELAY);
    Serial.write(_send,arraysize);  
    delay(SEND_DELAY);
    digitalWrite(ENABLE_PIN,LOW);
  }
