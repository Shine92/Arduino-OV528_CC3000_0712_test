//  File SerialCamera_DemoCode_CJ-OV528.ino
//  8/8/2013 Jack Shao
//  Demo code for using seeeduino or Arduino board to cature jpg format
//  picture from seeed serial camera and save it into sd card. Push the
//  button to take the a picture .

//  For more details about the product please check http://www.seeedstudio.com/depot/

#include <arduino.h>
#include <SoftwareSerial.h>

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"


#define PIC_PKT_LEN    128        //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0
#define CAM_SERIAL     softSerial

#define PIC_FMT        PIC_FMT_OCIF

// Define CC3000 chip pins
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "ASUS"
#define WLAN_PASS       "00000000"


// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
// received before closing the connection.  If you know the server
// you're accessing is quick to respond, you can reduce this value.


#define WEBSITE      "192.168.43.161"
#define WEBPAGE2      "/serverTest/getData.php"
#define BUFFER_SIZE 256
uint32_t ip;

SoftwareSerial softSerial(2, 4);  //rx,tx (11-13 is used by sd shield)

const byte cameraAddr = (CAM_ADDR << 5);  // addr

unsigned long picTotalLen = 0;            // picture length
int picNameNum = 0;
char buf[BUFFER_SIZE+1];
String str = "";    
String strCmd = "";


/*********************************************************************/
void setup()
{
  Serial.begin(115200);
  CAM_SERIAL.begin(9600);       //cant be faster than 9600, maybe difference with diff board.
  
  initialize();
  
  Serial.println(F("Hello, CC3000!\n")); 
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  } 
  // Optional SSID scan
  // listSSIDResults();
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
  Serial.println(F("Connected!"));
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  
  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  ip = 0xC0A82BA1;                               //IP轉十六進位
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }
  cc3000.printIPdotsRev(ip);

}
/*********************************************************************/
void loop()
{
  TCPcheck();
}
/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if (!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
/*********************************************************************/
// 將rx過來的資料清空
void clearRxBuf()
{
  while (CAM_SERIAL.available())
  {
    CAM_SERIAL.read();
  }
}
/*********************************************************************/
void sendCmd(char cmd[], int cmd_len)
{
  for (char i = 0; i < cmd_len; i++) CAM_SERIAL.write(cmd[i]);
}
/*********************************************************************/
int readBytes(char *dest, int len, unsigned int timeout)
{
  int read_len = 0;
  unsigned long t = millis();
  while (read_len < len)
  {
    while (CAM_SERIAL.available() < 1)
    {
      if ((millis() - t) > timeout)
      {
        return read_len;
      }
    }
    *(dest + read_len) = CAM_SERIAL.read();
    Serial.print(*(dest + read_len), HEX);
    Serial.print(" ");
    read_len++;
  }
  Serial.println();
  return read_len;
}
/*********************************************************************/
// arduino给摄像机上电后,arduino连续发送同步请求(AA 0D 00 00 00 00),直到摄像机应 答(AA 0E 0D XX 00 00)。通常arduino发送 5 次以内,摄像机会成功应答。
// 1.cmd  aa 0d 00 00 00 00 同步請求
// 2.resp aa 0e 0d xx 00 00 回應1
// 3.resp aa 0d 00 00 00 00 回應2
// 4.cmd  aa 0e 0d 00 00 00 同步確認
void initialize()
{
  char cmd[] = {0xaa, 0x0d | cameraAddr, 0x00, 0x00, 0x00, 0x00} ;
  unsigned char resp[6];

  Serial.print("\ninitializing camera...");

  while (1)
  {
    // 1.cmd
    sendCmd(cmd, 6);
    // 2.resp
    if (readBytes((char *)resp, 6, 1000) != 6)
    {
      Serial.print(".");
      continue;
    }
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
    {
      // 3.resp
      if (readBytes((char *)resp, 6, 500) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
    }
  }
  cmd[1] = 0x0e | cameraAddr;
  cmd[2] = 0x0d;
  // 4.cmd
  sendCmd(cmd, 6);
  Serial.println("\nCamera initialization done.");
}
/*********************************************************************/
// 拍照前初始化
// 1. cmd  aa 01 00 07 xx 07 JPEG預覽，VGA格式
// 2. resp aa 0e 01 xx 00 00 回應ok
void preCapture()
{
  Serial.println("preCapture....");
  Serial.println("init");
  char cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x01, 0x03 };
  unsigned char resp[6];

  while (1)
  {
    clearRxBuf();
    // 1.
    sendCmd(cmd, 6);
    // 2.
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) break;
  }
  Serial.println("set package size");
  cmd[1] = 0x06 | cameraAddr;
  cmd[2] = 0x08;
  cmd[3] = PIC_PKT_LEN & 0xff;
  cmd[4] = (PIC_PKT_LEN >> 8) & 0xff;
  cmd[5] = 0;

  while (1)
  {
    clearRxBuf();
    // 1.1
    sendCmd(cmd, 6);
    // 1.2
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
  }
}
/*********************************************************************/
// 拍照
// 1.1 cmd  aa 06 08 00 02 00 設定packet length
// 1.2 resp aa 0e 06 xx 00 00 回應ok
// 2.1 cmd  aa 05 00 00 00 00 快照 壓縮圖像
// 2.2 resp aa 0e 05 xx 00 00 回應ok
// 3.1 cmd  aa 04 01 00 00 00 獲取圖像 快照圖像
// 3.2 resp aa 0e 04 xx 00 00 回應ok
// 3.3 resp aa 0a 01 ~~ ~~ ~~ 數據傳送請求 快照圖像，回應packet size
void Capture()
{
  Serial.println("Capture....");

  //  cmd[1] = 0x05 | cameraAddr;
  //  cmd[2] = 0;
  //  cmd[3] = 0;
  //  cmd[4] = 0;
  //  cmd[5] = 0;
  //  while (1)
  //  {
  //    clearRxBuf();
  //    // 2.1
  //    sendCmd(cmd, 6);
  //    // 2.2
  //    if (readBytes((char *)resp, 6, 100) != 6) continue;
  //    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
  //  }
  char cmd[] = { 0xaa, 0x04 | cameraAddr, 0x05, 0, 0, 0 };
  unsigned char resp[6];

  while (1)
  {
    clearRxBuf();
    // 3.1
    sendCmd(cmd, 6);
    // 3.2
    if (readBytes((char *)resp, 6, 100) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
    {
      // 3.3
      if (readBytes((char *)resp, 6, 1000) != 6)
      {
        continue;
      }
      if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x05)
      {
        picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
        //Serial.print("picTotalLen:");
        //Serial.println(picTotalLen);
        break;
      }
    }
  }
}
/*********************************************************************/
// 取得圖像數據
// 1. cmd  aa 0e 0i 00 00 00 回應相機數據傳送的請求
// 2. resp
// (1..looping..2)
// 3. cmd  aa 0e 00 00 F0 F0 結束
/*
  size_t File::write(const uint8_t *buf, size_t size) {
  size_t t;
  if (!_file) {
    setWriteError();
    return 0;
  }
  _file->clearWriteError();
  t = _file->write(buf, size);
  if (_file->getWriteError()) {
    setWriteError();
    return 0;
  }
  return t;
  }
  int16_t Adafruit_CC3000_Client::write(const void *buf, uint16_t len, uint32_t flags)
  {
  return send(_socket, buf, len, flags);
  }
*/
void sendData()
{
  unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
  if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

  char cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
  unsigned char pkt[PIC_PKT_LEN];

  Serial.print("Storing ");
  Serial.print(picTotalLen, DEC);
  Serial.print(" byte image.\n");

  strCmd = String("/serverTest/connectSQL.php?XD28=") + picTotalLen;
  strCmd.toCharArray(buf, BUFFER_SIZE);
  Serial.println(buf);

  if (www.connected()) {
    www.fastrprint(F("GET "));
    www.fastrprint(buf);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();


  for (unsigned int i = 0; i < pktCnt; i++)
  {
    cmd[4] = i & 0xff;
    cmd[5] = (i >> 8) & 0xff;

    int retry_cnt = 0;
retry:
    Serial.print(".");
    delay(10);
    clearRxBuf();
    // 1.
    sendCmd(cmd, 6);
    // 2.
    uint16_t cnt = readBytes((char *)pkt, PIC_PKT_LEN, 200);

    // 檢查封包是否完整
    // 校验码:错误校验码,低字节等于该包数据除去校验码的累加和,高字节等于 0。校验码低字节=累加(字节(0)~字节(N-2);校验码高字节=0。
    unsigned char sum = 0;
    for (int y = 0; y < cnt - 2; y++)
    {
      sum += pkt[y];
    }
    if (sum != pkt[cnt - 2])
    {
      if (++retry_cnt < 100) goto retry;
      else break;
    }

    // 寫檔，packet format如下:
    // [0][1]    ︱ [2][3]  ︱ [4]...[cnt-3] ︱ [cnt-2][cnt-1]
    // packet id ︱ 資料長度 ︱     資料      ︱     驗証碼
    // myFile.write((const uint8_t *)&pkt[4], cnt-6);
    //      client.write((const uint8_t *)&pkt[4], cnt-6);
    //if (cnt != PIC_PKT_LEN) break;
  }

  cmd[4] = 0xf0;
  cmd[5] = 0xf0;
  // 3.
  sendCmd(cmd, 6);
  Serial.println(F("-------------------------------------"));

}
/**************************************************************************/
/*!
   TCP檢查 讀＠指令拍照
*/
/**************************************************************************/
void TCPcheck(void)
{
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
   
  if (www.connected()) {

    www.fastrprint(F("GET "));
    www.fastrprint(WEBPAGE2);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();    
  } else {
    Serial.println(F("Connection failed"));
    return;
  }
  Serial.println(F("-------------------------------------"));
  /**************************************************************************/
  /*!
     讀＠指令 + 拍照
  */
  /**************************************************************************/
  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis();   
  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      str = str + c;
      Serial.print(c);
      lastRead = millis();
    }
  }
  picture();      /*拍照判斷*/
  www.close();
  Serial.println(F("-------------------------------------"));
  return;
}
/**************************************************************************/
/*!
    拍照 判斷
*/
/**************************************************************************/
void picture(void)
{     
      int n =0;
      Serial.println("WAITING ......");
      delay(5000);                        //等候5秒
      if(str.endsWith("@")){               //如果有收到＠
      Serial.println("taking....");
      delay(1000);        
      if(n == 0) preCapture();
      Capture();
      Serial.print("Saving picture...");
      sendData();                    

      Serial.print("\r\nDone");
      Serial.println(n);
      n++;
      } 
return;  
}

