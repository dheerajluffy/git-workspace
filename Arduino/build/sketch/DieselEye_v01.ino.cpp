#include <Arduino.h>
#line 1 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
#include <SPI.h>
#include <stdint.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>
#include <mcp2515.h>
//#include <SoftwareSerial.h>
//SoftwareSerial Serial(PIN_PC7, PIN_PC6); // RX, TX
#define PIN_TDC1000_OSC_ENABLE    (A0)
//#define PIN_TDC1000_RESET     (PIN_PB0)
#define NUM_STOPS (1)

int dt = 200;
float CLK_FREQ = 8 * pow(10, 6); //CLOCK frequency (default 8MHz)
float CLK_PERIOD = 1 / CLK_FREQ;
uint32_t SPIfrequency = 20000000;

uint16_t readCONFIG1 ;
uint16_t readCONFIG2 ;
uint16_t readINT_STATUS ;
uint16_t readINT_MASK ;
uint16_t val;
uint16_t val1;
uint16_t val2;
uint16_t val3;
uint16_t val4;
uint16_t val5;
uint16_t val6;
uint16_t val7;
uint16_t val8;
uint16_t val9;

//**********TDC7200 REGISTER ADDRESS LIST**********

byte CONFIG1 = 0x00;
byte CONFIG2 = 0x01;
byte INT_STATUS = 0x02;
byte INT_MASK = 0x03;
byte COARSE_CNTR_OVF_H = 0x04;
byte COARSE_CNTR_OVF_L = 0x05;
byte CLOCK_CNTR_OVF_H = 0x06;
byte CLOCK_CNTR_OVF_L = 0x07;
byte CLOCK_CNTR_STOP_MASK_H = 0x08;
byte CLOCK_CNTR_STOP_MASK_L = 0x09;

#define TDC7200_REG_ADR_CALIBRATION1           (0x1Bu)
#define TDC7200_REG_ADR_CALIBRATION2           (0x1Cu)
#define TDC7200_REG_ADR_TIME1                  (0x10u)
#define TDC7200_REG_ADR_TIMEX(num)             (TDC7200_REG_ADR_TIME1+2*((num)-1))
#define TDC7200_REG_ADR_CLOCK_COUNTX(num)      (TDC7200_REG_ADR_CLOCK_COUNT1+2*((num)-1))
#define TDC7200_SPI_REG_ADDR_MASK (0x1Fu)
#define TDC7200_SPI_REG_READ      (0x00u)
#define TDC7200_REG_ADR_CLOCK_COUNT1           (0x11u)

byte TIME1 = 0x10;
byte CLOCK_COUNT1 = 0x11;
byte TIME2 = 0x12;
byte CALIBRATION1 = 0x1B;
byte CALIBRATION2 = 0x1C;
//**********END OF TDC7200 REGISTER ADDRESS LIST*********

int selectTDC7200 = 9;//48;
int enableTDC7200 = 5;//49;
//int TDC7200clock = 11;

/////////////////////////TDC1000 REGISTER ADDRESS LIST//////////////////////////////
#define CONFIG_0      0x00
#define CONFIG_1      0x01
#define CONFIG_2      0x02
#define CONFIG_3      0x03
#define CONFIG_4      0X04
#define TOF_1         0X05
#define TOF_0         0X06
#define ERROR_FLAGS   0X07
#define TIMEOUT       0X08
#define CLOCK_RATE    0x09
////////////////////////END OF TDC1000 REGISTER ADDRESS LIST//////////////////////////


/////////////////////////////////////////////////////////////////////
#define TDC1000_SPI_CLK_MAX                               (int32_t(20000000))
#define TDC1000_SPI_REG_ADDR_MASK                         (0x1Fu)
#define TDC1000_SPI_REG_READ                              (0x00u)
#define TDC1000_SPI_REG_WRITE                             (0x40u)
///////////////////////////////////////////////////////////////////

#define PIN_TDC1000_RESET     (10)
#define PIN_TDC1000_ENABLE    (6)
#define PIN_TDC1000_CHSEL     (A1)
#define PIN_TDC1000_SPI_CS    (8)
//#define PIN_VDD1              (PIN_PD1)
#define PIN_TDC1000_ERRB      (4)

MPU6050 mpu6050(Wire);
MCP2515 mcp2515(7);
struct can_frame canMsg1;

#line 98 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void setup();
#line 162 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void begin();
#line 174 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void loop();
#line 188 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void measure( int calibPeriod );
#line 198 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void calculateTof();
#line 235 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void tdc1000configRead();
#line 267 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void tdc7200configRead();
#line 300 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
char * double2s(double f, unsigned int digits);
#line 326 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void TDC7200Write(byte address, byte data);
#line 338 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
byte TDC7200Read(byte address);
#line 350 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
uint8_t TDC1000Read(const uint8_t addr);
#line 364 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void TDC1000Write(const uint8_t addr, const uint8_t val);
#line 375 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
uint32_t spiReadReg24(const uint8_t addr);
#line 395 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
static void ui64toa(uint64_t v, char * buf, uint8_t base);
#line 414 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void i2cRead();
#line 98 "c:\\Users\\KURO\\Documents\\GitHub\\workspace\\Arduino\\DieselEye_v01\\DieselEye_v01.ino"
void setup() {
  Serial.begin(9600);
  Serial.println("----------------------Program Start-------------------------,");
  SPI.begin();

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  ////////////////////tdc7200 pin direction///////////
  pinMode(selectTDC7200, OUTPUT);
  pinMode(enableTDC7200, OUTPUT);
  ////////////////////////////////////////////////////

  ///////////////////TDC1000 PIN DIRECTION////////////
  pinMode(PIN_TDC1000_ENABLE, OUTPUT);
  //  pinMode(PIN_VDD1, OUTPUT);
  pinMode(PIN_TDC1000_RESET, OUTPUT);
  pinMode(PIN_TDC1000_ERRB, INPUT_PULLUP);
  pinMode(PIN_TDC1000_SPI_CS, OUTPUT);
  pinMode(PIN_TDC1000_OSC_ENABLE, OUTPUT);
  /////////////////////////////////////////////////////

  //////////////////////////initializing pins//////////
  //  digitalWrite(PIN_VDD1, HIGH);
  digitalWrite(PIN_TDC1000_ENABLE, HIGH);
  digitalWrite(PIN_TDC1000_RESET, HIGH);
  _delay_ms(12);
  digitalWrite(PIN_TDC1000_RESET, LOW);
  digitalWrite(PIN_TDC1000_CHSEL, LOW);
  digitalWrite(PIN_TDC1000_OSC_ENABLE, HIGH);
  /////////////////////////////////////////////////////

  digitalWrite(selectTDC7200, HIGH); //Select is Active low, set to high @ initialise
  digitalWrite(enableTDC7200, LOW); //Enable is Active high, set to low @ initialise
  _delay_ms(20);
  digitalWrite(enableTDC7200, HIGH);
  delay(dt);

  //////////////////////initialize tdc1000/////////////
  TDC1000Write(CONFIG_0, 0X44);
  TDC1000Write(CONFIG_1, 0X41); //41
  TDC1000Write(CONFIG_2, 0X0);
  TDC1000Write(CONFIG_3, 0XC);
  TDC1000Write(CONFIG_4, 0X5F);
  TDC1000Write(TOF_1, 0X40);
  TDC1000Write(TOF_0, 0X1E);
  TDC1000Write(ERROR_FLAGS, 0X00);
  TDC1000Write(TIMEOUT, 0X23);
  TDC1000Write(CLOCK_RATE, 0X1);

  val = TDC1000Read(CONFIG_0);
  val1 = TDC1000Read(CONFIG_1);
  val2 = TDC1000Read(CONFIG_2);
  val3 = TDC1000Read(CONFIG_3);
  val4 = TDC1000Read(CONFIG_4);
  val5 = TDC1000Read(TOF_1);
  val6 = TDC1000Read(TOF_0);
  val7 = TDC1000Read(ERROR_FLAGS);
  val8 = TDC1000Read(TIMEOUT);
  val9 = TDC1000Read(CLOCK_RATE);
  /////////////////////////////////////////////////////

}

void begin() {
  TDC7200Write(CONFIG1, 0x03); //03
  TDC7200Write(CONFIG2, 0x00); //00
  TDC7200Write(INT_STATUS, 0x00);
  TDC7200Write(INT_MASK, 0x00);

  readCONFIG1 = TDC7200Read(CONFIG1);
  readCONFIG2 = TDC7200Read(CONFIG2);
  readINT_STATUS = TDC7200Read(INT_STATUS);
  readINT_MASK = TDC7200Read(INT_MASK);
}

void loop() {
  begin(); //Start new measurement by writing to tdc7200 spi registers
  SPI.beginTransaction(SPISettings(SPIfrequency, MSBFIRST, SPI_MODE3));
  //tdc1000configRead(); //uncomment to print reg config of tdc1000
  //tdc7200configRead(); //uncomment to print reg config of tdc7200
 // measure(10); //uncoment to print Tof parameters
  mpu6050.update();
  calculateTof();
  //i2cRead();
  Serial.print("angleX : ");
  Serial.println(mpu6050.getAngleX());

}

void measure( int calibPeriod ) {
  float cal2 = (spiReadReg24(0X1Cu));
  float cal1 = (spiReadReg24(0X1Bu));
  double time1 = spiReadReg24(0X10);
  double time2 = spiReadReg24(0X12);
  double clkcount = spiReadReg24(0X11);

  String payload = ("c1:" + String(cal1) + "," + "c2:" + String(cal2) + "," + "t1:" + String(time1) + "," + "t2:" + String(time2) + "," + "clkc:" + String(clkcount) + ",.");
  Serial.println(payload);
}
void calculateTof() {
  double clkperiod = 125 * pow(10, -9);
  double calcount = 0;
  double normlsb = 0;
  double TOF1 = 0;
  uint64_t cal1 = (spiReadReg24(0X1Bu));
  uint64_t cal2 = (spiReadReg24(0X1Cu));
  uint64_t clkcount = (spiReadReg24(0X11));
  uint64_t time1 = spiReadReg24(0X10);
  uint64_t time2 = spiReadReg24(0X12);
  calcount = (cal2 - cal1) / (9);
  normlsb = (clkperiod) / (calcount);
  TOF1 = (normlsb * (time1 - time2)) + (clkcount * clkperiod);
  //  float height = ((a * 1480) / 2) * 1000;
  //Serial.print("height =");
  //Serial.println(double2s(height,8));
  String S1 = double2s(TOF1, 16);
  //Serial.println(S1);
  float a = (S1.substring(0, 3)).toFloat();
  float height = ((a * 1300) / 2) * 0.1;
  Serial.println(String(height) + "MM");

  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = TOF1;
  canMsg1.data[1] = height;
  canMsg1.data[2] = mpu6050.getAngleX();
  canMsg1.data[3] = mpu6050.getAngleY();
  canMsg1.data[4] = mpu6050.getAngleZ();
  canMsg1.data[5] = mpu6050.getAccX();
  canMsg1.data[6] = mpu6050.getAccX();
  canMsg1.data[7] = mpu6050.getAccX();
  mcp2515.sendMessage(&canMsg1);

}


void tdc1000configRead(){
   Serial.print("TDC1000 CONFIG 0 = ");
   Serial.println(val, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 CONFIG 1 = ");
   Serial.println(val1, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 CONFIG 2 = ");
   Serial.println(val2, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 CONFIG 3 = ");
   Serial.println(val3, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 CONFIG 4 = ");
   Serial.println(val4, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 TOF1 = ");
   Serial.println(val5, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 TOF0 = ");
   Serial.println(val6, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 ERROR FLAG = ");
   Serial.println(val7, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 TIMEOUT = ");
   Serial.println(val8, HEX);
   Serial.print("\n");
   Serial.print("TDC1000 CLOCK RATE = ");
   Serial.println(val9, HEX);
   Serial.print("\n");
}
void tdc7200configRead(){
   Serial.print("TDC7200 CONFIG1= ");
   Serial.println(readCONFIG1, HEX);
   Serial.print("\n");
   Serial.print("TDC7200 CONFIG2= ");
   Serial.println(readCONFIG2, HEX);
   Serial.print("\n");
   Serial.print("TDC7200 INT_STATUS= ");
   Serial.println(readINT_STATUS, HEX);
   Serial.print("\n");
   Serial.print("TDC7200 INT_MASK= ");
   Serial.println(readINT_MASK, HEX);
   Serial.print("\n");
   Serial.print("TDC7200 TIME1= ");
   Serial.println(spiReadReg24(0X10), HEX);
   Serial.print("\n");
   Serial.print("TDC7200 COUNT1= ");
   Serial.println(spiReadReg24(0X11), HEX);
   Serial.print("\n");
   Serial.print("TDC7200 TIME2= ");
   Serial.println(spiReadReg24(0X12), HEX);
   Serial.print("\n");
   Serial.print("TDC7200 COUNT2= ");
   Serial.println(spiReadReg24(0X13), HEX);
   Serial.print("\n");
   Serial.print("TDC7200 CALIB1= ");
   Serial.println(spiReadReg24(0X1Bu), DEC);
   Serial.print("\n");
   Serial.print("TDC7200 CALIB2= ");
   Serial.println(spiReadReg24(0X1Cu), DEC);
   Serial.print("\n");

}
char * double2s(double f, unsigned int digits)
{
  int index = 0;
  static char s[16];                    // buffer to build string representation

  // max digits
  if (digits > 6) digits = 6;
  long multiplier = pow(10, digits);     // fix int => long

  int exponent = int(log10(f));
  float g = f / pow(10, exponent);
  if ((g < 1.0) && (g != 0.0))
  {
    g *= 10;
    exponent--;
  }

  long whole = long(g);                     // single digit
  long part = long((g - whole) * multiplier); // # digits
  char format[16];
  sprintf(format, "%%ld.%%0%dld E%%+d", digits);
  sprintf(&s[index], format, whole, part, exponent);

  return s;
}

void TDC7200Write(byte address, byte data) {
  SPI.beginTransaction(SPISettings(SPIfrequency, MSBFIRST, SPI_MODE3));
  digitalWrite(selectTDC7200, LOW);//between selectTDC1000 OR selectTDC7200
  delayMicroseconds(100);
  address |= 0x40;
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(selectTDC7200, HIGH);
  SPI.endTransaction();
};


byte TDC7200Read(byte address) {
  SPI.beginTransaction(SPISettings(SPIfrequency, MSBFIRST, SPI_MODE3));
  digitalWrite(selectTDC7200, LOW);
  delayMicroseconds(100);
  //SPI.transfer(address);
  SPI.transfer(address);
  byte inByte = SPI.transfer(0x00);
  digitalWrite(selectTDC7200, HIGH);
  SPI.endTransaction();
  return inByte;
};

uint8_t TDC1000Read(const uint8_t addr)
{
  SPI.beginTransaction(SPISettings(TDC1000_SPI_CLK_MAX, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_TDC1000_SPI_CS, LOW);

  SPI.transfer((addr & TDC1000_SPI_REG_ADDR_MASK) | TDC1000_SPI_REG_READ);
  uint8_t val = SPI.transfer(0u);

  digitalWrite(PIN_TDC1000_SPI_CS, HIGH);
  SPI.endTransaction();

  return val;
}

void TDC1000Write(const uint8_t addr, const uint8_t val)
{
  SPI.beginTransaction(SPISettings(TDC1000_SPI_CLK_MAX, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_TDC1000_SPI_CS, LOW);

  (void)SPI.transfer16((((addr & TDC1000_SPI_REG_ADDR_MASK) | TDC1000_SPI_REG_WRITE) << 8) | val);

  digitalWrite(PIN_TDC1000_SPI_CS, HIGH);
  SPI.endTransaction();
}

uint32_t spiReadReg24(const uint8_t addr)
{
  SPI.beginTransaction(SPISettings(SPIfrequency, MSBFIRST, SPI_MODE3));
  digitalWrite(selectTDC7200, LOW);

  SPI.transfer((addr & TDC7200_SPI_REG_ADDR_MASK) | TDC7200_SPI_REG_READ);
  uint32_t val;
  val = SPI.transfer(0u);
  val <<= 8;
  val |= SPI.transfer(0u);
  val <<= 8;
  val |= SPI.transfer(0u);

  digitalWrite(selectTDC7200, HIGH);
  SPI.endTransaction();

  return val;
}


static void ui64toa(uint64_t v, char * buf, uint8_t base)
{
  int idx = 0;
  uint64_t w = 0;
  while (v > 0)
  {
    w = v / base;
    buf[idx++] = (v - w * base) + '0';
    v = w;
  }
  buf[idx] = 0;
  // reverse char array
  for (int i = 0, j = idx - 1; i < idx / 2; i++, j--)
  {
    char c = buf[i];
    buf[i] = buf[j];
    buf[j] = c;
  }
}
void i2cRead(){
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}

