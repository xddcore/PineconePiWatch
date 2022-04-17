
#define DebugPrintln(message) Serial.println(message)
#define DebugPrint(message) Serial.print(message)


/**************************TFT屏幕部分******************************/
#include <SPI.h>
#include <TFT_eSPI.h>
void tft_init(void); //tft初始化函数
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
void tft_init()//屏幕初始化
{
  
 tft.init();
 tft.setSwapBytes(true);
 tft.invertDisplay( false );
 tft.fillScreen(TFT_BLACK);
 tft.setCursor(0, 0, 2);
 tft.setTextColor(TFT_WHITE, TFT_BLACK);
 tft.print("TFT Init...[");
 tft.setTextColor(TFT_BLUE, TFT_BLACK);
 tft.print("X");
 tft.setTextColor(TFT_WHITE, TFT_BLACK);
 tft.println("]");
}

#include <JPEGDecoder.h>
void drawSdJpeg(const char *filename, int xpos, int ypos) {
  uint32_t readTime = millis();
  // Open the named file (the Jpeg decoder library will close it)
  File jpegFile = SD.open( filename, FILE_READ);  // or, file handle reference for SD library
  if ( !jpegFile ) {
    DebugPrint("ERROR: File \"");
    DebugPrint(filename);
    DebugPrintln ("\" not found!");
    return;
  }
  DebugPrintln("===========================");
  DebugPrint("Drawing file: "); DebugPrintln(filename);
  DebugPrintln("===========================");
  // Use one of the following methods to initialise the decoder:
  boolean decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,
  //boolean decoded = JpegDec.decodeSdFile(filename);  // or pass the filename (String or character array)
  //SD_read_Time(millis() - readTime);
  if (decoded) {
    // print information about the image to the serial port
    //jpegInfo();
    // render the image onto the screen at given coordinates
    jpegRender(xpos, ypos);
  }
  else {
    DebugPrintln("Jpeg file format not supported!");
  }
}
 
//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void jpegRender(int xpos, int ypos) {
  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();
  //jpegInfo(); // Print information from the JPEG file (could comment this line out)
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;
  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);
  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = (mcu_w<(max_x % mcu_w)?mcu_w:(max_x % mcu_w));
  uint32_t min_h = (mcu_h<(max_y % mcu_h)?mcu_h:(max_y % mcu_h));
  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;
  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;
  // Fetch data from the file, decode and display
  while (JpegDec.read()) {    // While there is more data in the file
    pImg = JpegDec.pImage ;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)
    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;
    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }
    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;
    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ( (mcu_y + win_h) >= tft.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }
  tft.setSwapBytes(swapBytes);
  //showTime(millis() - drawTime); //将图片显示到屏幕所用的时间(ms)
}
/**************************TFT屏幕部分******************************/
/**************************SD卡部分******************************/
#include <SD.h>
#include <FS.h>
SPIClass sdSPI(VSPI);
#define SD_MISO     19
#define SD_MOSI     23
#define SD_SCLK     18
#define SD_CS       5
void TFCard_Init(void);
void TFCard_Init()
{
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  delay(100);
  if (!SD.begin(SD_CS, sdSPI))
  {
    //tft.println("存储卡挂载失败");
    tft.println("TF Card Init...[ ]");
    return;
  }
  uint8_t cardType = SD.cardType();
 
  if (cardType == CARD_NONE)
  {
    //tft.println("未连接存储卡");
    tft.println("TF Card Init...[ ]");
    return;
  }
  else if (cardType == CARD_MMC)
  {
    //tft.println("挂载了MMC卡");
  }
  else if (cardType == CARD_SD)
  {
    //tft.println("挂载了SDSC卡");
  }
  else if (cardType == CARD_SDHC)
  {
    //tft.println("挂载了SDHC卡");
  }
  else
  {
    //tft.println("挂载了未知存储卡");
  }
  //char sd_size_string[30];
  //sprintf(sd_size_string,"TF Card Init ...[X] = %ld MB",SD.cardSize()/(1024*1024));
 tft.setTextColor(TFT_WHITE, TFT_BLACK);
 tft.print("TF Card Init...[");
 tft.setTextColor(TFT_BLUE, TFT_BLACK);
 tft.print("X");
 tft.setTextColor(TFT_WHITE, TFT_BLACK);
 tft.println("]"); 
}
/**************************SD卡部分******************************/
/**************************按钮部分******************************/
byte UserButton_Press_Down_flag = 0; //按钮按下标志位
byte UserButton_Left_Press_Down_flag = 0; //按钮上拨标志位
byte UserButton_Right_Press_Down_flag = 0; //按钮下拨标志位
void UserButton_Init()
{
  pinMode(33, INPUT|PULLUP ); // Press Down
  pinMode(32, INPUT|PULLUP ); // Left Press Down
  pinMode(34, INPUT|PULLUP ); // Right Press Down
  attachInterrupt(33, UserButton_Press_Down, FALLING); // negedge
  attachInterrupt(32, UserButton_Left_Press_Down, FALLING); // negedge
  attachInterrupt(34, UserButton_Right_Press_Down, FALLING); // negedge
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Button Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print("X");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
}
void UserButton_Press_Down()
{
  UserButton_Press_Down_flag = 1;
}
void UserButton_Left_Press_Down()
{
  UserButton_Left_Press_Down_flag = 1;
}
void UserButton_Right_Press_Down()
{
  UserButton_Right_Press_Down_flag = 1;
}
/**************************按钮部分******************************/
/**************************LED部分******************************/
//上绿下蓝
#define LED_Green 26
#define LED_Blue 27
void LED_Init()
{
  pinMode(LED_Green, OUTPUT); // Blue LED
  pinMode(LED_Blue, OUTPUT); // Green LED
  digitalWrite(LED_Green, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_Blue, HIGH);    // turn the LED off by making the voltage LOW
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("LED Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print("X");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
}
/**************************LED部分******************************/
/**************************震动马达部分******************************/
//上绿下蓝
#define shake_motor 12
void shake_motor_Init()
{
  pinMode(shake_motor, OUTPUT); // shake_motor
  digitalWrite(shake_motor, LOW);    // turn the shake_motor off by making the voltage LOW
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Shake Motor Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print("X");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
}
/**************************震动马达部分******************************/

/**************************陀螺仪+加速度计部分+LOG打印******************************/
#include <Wire.h>
#include "JY901.h"
#include <HardwareSerial.h> 

HardwareSerial Serial_2(2);//串口2
int GY_60_available = 0;//0无效 1有效
void GY_60_Init()
{
  Serial.begin(115200);
  Serial_2.begin(9600);
  delay(50);
  GY_60_Event();
  if(GY_60_available == 1)
  {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Sensor1 Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print("X");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
  }
  else if(GY_60_available == 0)
  {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Sensor1 Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print(" ");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
  }
}
void GY_60_Event()//解析GY60的数据
{
  while (Serial_2.available()) 
  {
    GY_60_available = 1;
    JY901.CopeSerialData(Serial_2.read()); //Call JY901 data cope function
  }
}

/**************************陀螺仪+加速度计部分+LOG打印******************************/
/**************************心率 血压(软串口连接STC  )******************************/
#include <Wire.h>
#define Systolic_bit 0x01
#define Diastolic_bit 0x02
#define heart_bit 0x03
#define temp_h 0x04
#define temp_l 0x05
#define wave_h 0x11
#define wave_l 0x12

#define sensor_status 0x07
#define tft_bit 0x08
#define sensor_bit 0x09
#define stc_sleep_bit 0x10

#define enable_co 0x01
#define disable_co 0x00

uint8_t error;
uint8_t i2cAddress = 0x2d;//2d

unsigned char sensor_data[8];
  float temp = 0.0;//体温
  int heart = 0;//心率
  int Systolic = 0;//收缩压
  int Diastolic = 0;//舒张压
  unsigned char i = 0;
void sensor2_init()
{
  Wire.begin();
  Wire.beginTransmission(i2cAddress);
  //Wire.write(0x09);
  //Wire.write(0x01);
  error = Wire.endTransmission();
  //Wire.requestFrom(i2cAddress, 1);
  //if(Wire.available() >= 1)
  //{
   //status = Wire.read(); 
   //Serial.println(status, HEX);
  //}
  Serial.println(error);
  if(error == 0)//收到了ACK数据
  {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Sensor2 Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print("X");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
  return;  
  }
  else
  {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Sensor2 Init...[");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print(" ");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("]");
  return;
  }    
  
  /*
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    */
}
void tft_switch(unsigned char onoff)
{
  if(onoff == 1)
  {
    Wire.beginTransmission(i2cAddress);
    Wire.write(tft_bit);
    Wire.write(enable_co);
    error = Wire.endTransmission();
  }
  else
  {
    Wire.beginTransmission(i2cAddress);
    Wire.write(tft_bit);
    Wire.write(disable_co);
    error = Wire.endTransmission();
  }
}
void sensor_switch(unsigned char onoff)
{
  if(onoff == 1)
  {
    Wire.beginTransmission(i2cAddress);
    Wire.write(sensor_bit);//0x55
    Wire.write(enable_co);
    error = Wire.endTransmission();
  }
  else
  {
    Wire.beginTransmission(i2cAddress);
    Wire.write(sensor_bit);
    Wire.write(disable_co);
    error = Wire.endTransmission();
  }
}
void stc_switch(unsigned char onoff)
{
  if(onoff == 1)
  {
    Wire.beginTransmission(i2cAddress);
    Wire.write(stc_sleep_bit);
    Wire.write(enable_co);
    error = Wire.endTransmission();
  }
  else
  {
    /*先发一个数据唤醒下*/
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x55);
    error = Wire.endTransmission();
    /*然后在把控制位关掉*/
    Wire.beginTransmission(i2cAddress);
    Wire.write(stc_sleep_bit);
    Wire.write(disable_co);
    error = Wire.endTransmission();
  }
}
void sensor2_handle()
{
    for(i=0x01;i<=0x05;i++)
    {
      Wire.beginTransmission(i2cAddress);
      Wire.write(i);
      Wire.endTransmission();
      Wire.requestFrom(i2cAddress, 1);
      if(Wire.available() >= 1)
      {
        sensor_data[i] = Wire.read(); 
        //Serial.print(i);
        //Serial.print("|");
        //Serial.println(sensor_data[i], HEX);
      }
    }
    
    temp = (sensor_data[0x04]*256+sensor_data[0x05])/256;
    if(sensor_data[0x01]<250)
    {
      Systolic = sensor_data[0x01];
    }
    if(sensor_data[0x02]<250)
    {
      Diastolic = sensor_data[0x02];
    }
    if(sensor_data[0x03]<250)
    {
      heart = sensor_data[0x03];
    }
    Serial.print("heart:");
    Serial.print(heart);
    Serial.print("|");
    Serial.print("temp:");
    Serial.println(temp);
    //Serial.printf("Systolic:%d|Diastolic:%d|heart:%d|temp:%f.\r\n",Systolic,Diastolic,heart,temp);
}
/**************************心率 血压******************************/
/**************************WIFI与NTP时间******************************/
#include <WiFi.h>
#include <PubSubClient.h> //MQTT库函数
#include "time.h"
 
const char* ssid       = "TP-LINK_18A1";
const char* password   = "dl742163";

const char* mqtt_server = "47.242.51.78";//mqtt服务器IP地址
const int mqttPort = 1883;//mqtt服务器端口地址

WiFiClient espClient;
PubSubClient client(espClient);

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;
int wifi_tick = 0;
int request_status = 0;//服务器请求设备发送自己的数据
void wifi_init()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      wifi_tick++;
      if(wifi_tick == 10)//尝试10次重连，无法连接则跳过
      {
      return;
      }
      //WiFi.begin(ssid, password);
  }
  Serial.println("WIFI CONNECTED");
  
   //WIFI路由器连接成功后,连接MQTT服务器
  client.setServer(mqtt_server, mqttPort);
  client.setCallback(callback);
  while (!client.connected()) {
    if (client.connect("PineconePiWatch")) {
      //成功连接MQTT服务器
    } else {
      //不成功连接MQTT服务器则每隔2秒重试一次
      delay(2000);
    }
  }
 
  client.publish("PineconePi/Watch", "PineconePi Watch Online!");//设备上线
  client.subscribe("PineconePi/Watch");//订阅mqtt服务器PineconePi/Watch主题
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //printLocalTime();
 
  //disconnect WiFi as it's no longer needed
  //WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);

}
void callback(char* topic, byte* payload, unsigned int length) {//接收mqtt返回的数据
  //mqtt指令表
  //a:开机 b:关机 
  //c:上一界面 d:下一个界面
 char command = (char)payload[0];
 switch(command)//根据网页上发来的命令做相应处理
 {
  case 'a':break;
  case 'b':break;

  case 'c':break;
  case 'd':break;

  case 'z':request_status = 1;break;
  default:break;
 }
}
void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  //Serial.println(&timeinfo, "%A, %Y-%m-%d %H:%M:%S");
  char ntp_time_string[50];
  sprintf(ntp_time_string,"%d:%d:%d",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
  //打印存储卡信息
  tft.println(ntp_time_string); 
}
/**************************WIFI与NTP时间******************************/
/**************************显示开机LOGO******************************/
void show_logo()
{
  tft.fillScreen(TFT_BLACK);
  digitalWrite(shake_motor, HIGH);    // turn the shake_motor on by making the voltage HIGH
  drawSdJpeg("/sys/logo.jpg", 0, 0);     // This draws a jpeg pulled off the SD Card
  digitalWrite(LED_Green, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_Blue, LOW);    // turn the LED off by making the voltage LOW
}
/**************************显示开机LOGO******************************/
/**************************ULP部分******************************/
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "UlpDebug.h" //ULP调试器输出
//#include "UlpDebug.h"

// 慢速内存变量分配
enum {
  //SLOW_BLINK_STATE,     // Blink状态变量 
  SLOW_PROG_ADDR        // 程序开始地址
};

void ULP_Init(uint32_t us) {
  ulp_set_wakeup_period(0, us);  // 设置ULP激活间隔 30秒一次 
  
  memset(RTC_SLOW_MEM, 0, 8192);  // 慢速内存初始化
  //RTC_SLOW_MEM[SLOW_BLINK_STATE] = 0;  //状态变量 Blink状态初始化 0亮 1灭

  // 接口闪烁 (specify by +14)
  //RTC6 -> GPIO25
  int pin_blink_bit = RTCIO_GPIO25_CHANNEL + 14;//RTC6 +14 = 20 计算寄存器位
  const gpio_num_t pin_blink = GPIO_NUM_25;
  Serial.printf("rtc_bit = %d",pin_blink_bit);
  // 将25号管脚PIN初始化为RTC GPIO
  rtc_gpio_init(pin_blink);
  rtc_gpio_set_direction(pin_blink, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(pin_blink, 0);
  
  // ULP程序 输出56ms的高电平然后输出低电平暂停，然后等待下一次唤醒
  const ulp_insn_t  ulp_prog[] = {    
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit, pin_blink_bit, 1), // pin_blink_bit = 1 输出高电平
    //IP5306 延迟 56 ms要求高电平持续时间大于50ms 7*8 = 56 需要7个I_DELAY
    I_DELAY(65535),                         //8Mhz 65535/8x106 == 8ms
    I_DELAY(65535),
    I_DELAY(65535),
    I_DELAY(65535),
    I_DELAY(65535),
    I_DELAY(65535),
    I_DELAY(65535),
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit, pin_blink_bit, 0),// pin_blink_bit = 0 输出低电平
    I_HALT()                               //暂停ULP等待30s下次定时器唤醒
  };
  esp_err_t ulp_run_status;
  size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(0, ulp_prog, &size);
  ulp_run_status = ulp_run(0);
  if(ulp_run_status == ESP_OK)
  {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("ULP Init...[");
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.print("X");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("]");    
  }
  else
  {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("ULP Init...[");
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.print(" ");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("]");
  }
}
/**************************ULP部分******************************/

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// define two tasks for Blink & AnalogRead
void TaskUI( void *pvParameters );
void TaskLED(void *pvParameters);
//freertos || ucos ucosii
xQueueHandle xqueue0;  //创建的队列0句柄
// the setup function runs once when you press reset or power the board
void setup() {
  tft_init();//初始化TFT屏幕
  delay(300);
  LED_Init(); //LED初始化
  delay(300);
  TFCard_Init();//初始化TF卡
  delay(300);
  UserButton_Init();//按钮初始化
  delay(300);
  GY_60_Init();//GY60+LOG初始化
  delay(300);
  sensor2_init();//CoProcess初始化
  delay(300);
  //ULP_Init(30000000);//ULP处理器初始化，唤醒周期30s
  shake_motor_Init();
  delay(300);
  show_logo();//开机LOGO
  wifi_init();//链接WIFI
  
  //delay(3000);
  digitalWrite(shake_motor, LOW);    // 关闭震动马达
  //ulpDump();//串口打印ULP的汇编程序(ps:不能与ulp_init放在同一函数里面打印)
  //ulp程序数组的下标(地址) + 实际地址
  //利用数组来写地址的时候，实际上是通过下标
  tft_switch(1);
  sensor_switch(1);
  stc_switch(1);
  xqueue0 = xQueueCreate(10,sizeof(int));//创建的队列0
  
  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskUI
    ,  "TaskUI"   // A name just for humans
    ,  8192  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1);

      // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskLED
    ,  "TaskLED"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);
}

void loop()//main
{
  // Empty. Things are done in Tasks.
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void TaskUI(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(50);  //这里就是用于取数据阻塞了，我觉得本来这种收发就不可能同步，但是应该接收来满足发送，接收速度大于发送速度才行
  int status_ui = 1;
  while(1)
  {
  xStatus = xQueueReceive( xqueue0, &status_ui, xTicksToWait );  
  if(status_ui == 1)
  drawSdJpeg("/sys/status.jpg", 0, 0);//状态
  else if(status_ui == 2)
  drawSdJpeg("/sys/health.jpg", 0, 0);//健康
  else if(status_ui == 3)
  drawSdJpeg("/sys/sport.jpg", 0, 0);//运动
  else if(status_ui == 4)
  drawSdJpeg("/sys/weather.jpg", 0, 0);//天气
  else if(status_ui == 5)
  drawSdJpeg("/sys/notice.jpg", 0, 0);//通知
  else if(status_ui == 6)
  drawSdJpeg("/sys/more.jpg", 0, 0);//更多
  else if(status_ui == 7)
  {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  printLocalTime();
  //tft.println("Body Test");//更多
  tft.print("Systolic:");
  tft.println(Systolic);//更多
  tft.print("Diastolic:");
  tft.println(Diastolic);//更多
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("H:");//更多
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(heart);//更多
  tft.print("|");//更多
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.print("T:");//更多
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println(temp);//更多
  tft.print("Angle:");tft.println("  Gyro:");
  tft.print((float)JY901.stcAngle.Angle[0]/32768*180);tft.print("   ");tft.println((float)JY901.stcGyro.w[0]/32768*2000);
  tft.print((float)JY901.stcAngle.Angle[1]/32768*180);tft.print("   ");tft.println((float)JY901.stcGyro.w[1]/32768*2000);
  tft.print((float)JY901.stcAngle.Angle[2]/32768*180);tft.print("   ");tft.println((float)JY901.stcGyro.w[2]/32768*2000);
  }
  else if(status_ui == 8)
  {
    tft_switch(0);
    sensor_switch(0);
    stc_switch(0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0);  
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);
    esp_deep_sleep_start();//开始休眠
  }
  }
}

void TaskLED(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);   // 阻止任务的时间，直到队列有空闲空间 ，应该是如果发送需要阻滞等待（比如队列满了）或者别的情况需要用到的
  int status_ui = 1;
  BaseType_t xStatus;
  char Data_String[200];//温湿度和设备状态字符数组
  float AngleX=0.0,AngleY=0.0,AngleZ=0.0;
  float GyroX=0.0,GyroY=0.0,GyroZ=0.0;

  while(1) // A Task shall never return or exit.
  {    
    GY_60_Event();//解析GY60的数据
    sensor2_handle();
    digitalWrite(LED_Green, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_Blue, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    digitalWrite(LED_Green, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(LED_Blue, LOW);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    /***********************************mqtt****************************************/
    client.loop();//mqtt服务器心跳包
    if(request_status == 1)//request_status == 0
    {
    AngleX = (float)JY901.stcAngle.Angle[0]/32768*180;
    GyroX = (float)JY901.stcGyro.w[0]/32768*2000;

    AngleY = (float)JY901.stcAngle.Angle[1]/32768*180;
    GyroY = (float)JY901.stcGyro.w[1]/32768*2000;
    
    AngleZ = (float)JY901.stcAngle.Angle[2]/32768*180;
    GyroZ = (float)JY901.stcGyro.w[2]/32768*2000;
    
    sprintf(Data_String,"T:%f|H:%d|Systolic:%d|Diastolic:%d|Angle-X:%f|Angle-Y:%f|Angle-Z:%f|Gyro-X:%f|Gyro-Y:%f|Gyro-Z:%f",temp,heart,Systolic,Diastolic,AngleX,AngleY,AngleZ,GyroX,GyroY,GyroZ);//拼接设备状态字符串
    client.publish("PineconePi/Watch", Data_String);//将温湿度数据发送给mqtt服务器
    request_status = 0;//清除状态位
    }
    
    /********************按钮消抖触发************************/
    if(UserButton_Press_Down_flag == 1)//按钮按下
    {
      vTaskDelay(10);
      if((UserButton_Press_Down_flag == 1)&&(digitalRead(33)==LOW))
      {
        attachInterrupt(33, UserButton_Press_Down, RISING); // negedge
        UserButton_Press_Down_flag = 0;
      }
      if((UserButton_Press_Down_flag == 1)&&(digitalRead(33)==HIGH))
      {
        attachInterrupt(33, UserButton_Press_Down, FALLING); // negedge
        Serial.print("freeretos:用户按钮按下");//用户按钮按下
        UserButton_Press_Down_flag = 0;
      }
    } 
    if(UserButton_Left_Press_Down_flag == 1)//上拨
    {
      vTaskDelay(10);
      if((UserButton_Left_Press_Down_flag == 1)&&(digitalRead(32)==LOW))
      {
        attachInterrupt(32, UserButton_Left_Press_Down, RISING); // negedge
        UserButton_Left_Press_Down_flag = 0;
      }
      if((UserButton_Left_Press_Down_flag == 1)&&(digitalRead(32)==HIGH))
      {
        attachInterrupt(32, UserButton_Left_Press_Down, FALLING); // negedge
        Serial.print("freeretos:用户按钮上拨");//用户按钮上拨
        UserButton_Left_Press_Down_flag = 0;
        status_ui--;
        
        if(status_ui<1)status_ui=7;
        xStatus = xQueueSendToFront( xqueue0, &status_ui, xTicksToWait );
      }
    } 
    if(UserButton_Right_Press_Down_flag == 1)//下拨
    {
      vTaskDelay(10);
      if((UserButton_Right_Press_Down_flag == 1)&&(digitalRead(34)==LOW))
      {
        attachInterrupt(34, UserButton_Right_Press_Down, RISING); // negedge
        UserButton_Right_Press_Down_flag = 0;
      }
      if((UserButton_Right_Press_Down_flag == 1)&&(digitalRead(34)==HIGH))
      {
        attachInterrupt(34, UserButton_Right_Press_Down, FALLING); // negedge
        Serial.print("freeretos:用户按钮下拨");//按钮按下
        UserButton_Right_Press_Down_flag = 0;
        status_ui++;
        
        if(status_ui>7)status_ui=1;
        xStatus = xQueueSendToFront( xqueue0, &status_ui, xTicksToWait );
      }
    }     


    
  }
}
