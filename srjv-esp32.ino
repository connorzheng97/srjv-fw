#include <Arduino.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include <rom/gpio.h>
#include <soc/gpio_struct.h>
#include <esp32-hal-spi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <Update.h>
#include <webpages.h>

#define FIRMWARE_VERSION "1.10 Release"

#define LED_PIN 38
#define BTN_PIN 39

#define SENS_CTL_PIN 21

#define IO_OE_PIN 1

#define D0 16
#define D1 15
#define D2 14
#define D3 9
#define D4 8
#define D5 7
#define D6 6
#define D7 5
#define WE_PIN 4
#define OE_PIN 3
#define CE_PIN 2
#define RY_BY_PIN 40

#define SCK_PIN 12
#define MOSI_PIN 13
#define MISO_PIN 11
#define SS_PIN 10
#define SR_OE_PIN 41
#define SPI_SPEED_MHZ 27

#define STATE_UNKNOWN 0
#define STATE_ERASED 1
#define STATE_PROGRAMMING 2
#define STATE_PROGRAMMED 3
#define STATE_ERASING 4

#define WIFI_ALWAYS_ON 0
#define WIFI_OFF_BLE_WAKE 1
#define WIFI_OFF_BLE_OFF 2

typedef struct {
  uint32_t addr;
  uint8_t data;
} pgmData_t;

// class instances
// global variables

Preferences prefs;
AsyncWebServer server(80);
QueueHandle_t blinkerQueue;
TaskHandle_t blinkerHandle;
QueueHandle_t programBufferQueue;
TaskHandle_t programmerHandle;
TaskHandle_t housekeepingHandle;
const uint8_t BLINK_ON = 2;
const uint8_t BLINK_OFF = 3;
const uint8_t BLINK_FAST = 1;
const uint8_t BLINK_NORMAL = 0;
uint8_t machine_state = STATE_UNKNOWN;
char ap_name[33];
char ap_pass[33];
uint8_t ap_chan = 11;
uint8_t ap_hidden = 0;
char rom_id[0x22];
uint8_t wifi_pwr_mode = WIFI_ALWAYS_ON;
bool wifi_started = false;
bool in_progress = false;
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
SemaphoreHandle_t eraseSemaphore;
SemaphoreHandle_t flashExitBusySemaphore;
SemaphoreHandle_t programFinishSemaphore;
SemaphoreHandle_t restartSemaphore;
SPIClass *hspi = NULL;
SPISettings spi_settings = SPISettings(SPI_SPEED_MHZ * 1000000, SPI_MSBFIRST, SPI_MODE0);
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
uint8_t otaDone = 0;

// functions declarations

void blinkerTask(void *pvParameters);
void programmerTask(void *pvParameters);
void housekeepingTask(void *pvParameters);
void notFound(AsyncWebServerRequest *request);
bool eraseChip();
void IRAM_ATTR cmdCycle(uint32_t cmd_addr, uint8_t cmd_data);
uint8_t readCycle(uint32_t addr);
void IRAM_ATTR setAddr(uint32_t addr);
void IRAM_ATTR setData(uint8_t data);
uint8_t getData();
bool takeControl();
bool releaseControl();
bool beforeProgramming();
void afterProgramming();
void IRAM_ATTR programCycle(uint32_t addr, uint8_t data);
// functions defination

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void handleUpdateEnd(AsyncWebServerRequest *request) {
  
  if (Update.hasError()) {
    request->send(502, "text/plain", Update.errorString());
  } else {
    request->redirect("/");
    xSemaphoreGive(restartSemaphore);
  }
}

void handleUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    xQueueSendToBack(blinkerQueue, (void *)&BLINK_ON, 0);
    size_t fsize = UPDATE_SIZE_UNKNOWN;
    if (request->hasParam("size")) {
      fsize = request->getParam("size")->value().toInt();
    }
    log_i("OTA Update Start: %s", filename.c_str());
    if (!Update.begin(fsize)) {
      otaDone = 0;
      log_e("%s", Update.errorString());
      xQueueSendToBack(blinkerQueue, (void *)&BLINK_FAST, 0);
    }
  }

  if (Update.write(data, len) != len) {
    log_e("%s", Update.errorString());
  } else {
    otaDone = 100 * Update.progress() / Update.size();
    Serial.print(".");
  }
  
  if (final) {
    if (Update.end(true)) {
      log_i("Update Success: %u bytes, Rebooting...", index+len);
      xQueueSendToBack(blinkerQueue, (void *)&BLINK_NORMAL, 0);
    } else {
      log_e("%s", Update.errorString());
      xQueueSendToBack(blinkerQueue, (void *)&BLINK_FAST, 0);
      otaDone = 0;
    }
  }
}

void state_change(uint8_t new_state) {
  prefs.begin("cfg");
  prefs.putUChar("state", new_state);
  prefs.end();
  machine_state = new_state;
  log_i("State changed to %d", new_state);
}

String processor(const String &var) {
  //log_w(var);
  if (var == "ap_name") {
    return ap_name;
  } else if (var == "status") {
    if (machine_state == STATE_ERASING) {
      return "Erasing";
    } else if (machine_state == STATE_ERASED) {
      return "Erased";
    } else if (machine_state == STATE_PROGRAMMED) {
      return "Programmed";
    } else if (machine_state == STATE_PROGRAMMING) {
      return "Programming";
    } else {
      return "Unknown";
    }
  } else if (var == "busy") {
    if (in_progress) {
      return "true";
    } else {
      return "";
    }
  } else if (var == "rom_id") {
    return rom_id;
  } else if (var == "erase_disabled") {
    if (machine_state == STATE_ERASING || machine_state == STATE_ERASED) {
      return "disabled";
    } else {
      return "";
    }
  } else if (var == "program_disabled") {
    if (machine_state == STATE_PROGRAMMED || machine_state == STATE_PROGRAMMING) {
      return "disabled";
    } else {
      return "";
    }
  } else if (var == "fw_ver") {
    return FIRMWARE_VERSION;
  }
   else if (var == "wifi_pwr_mode") {
    if (wifi_pwr_mode == 0){
      return "On";
    }
    else if (wifi_pwr_mode == 1){
      return "Off with Bluetooth Wakeup";
    }
    else if (wifi_pwr_mode == 2){
      return "All Off (Only Button Wakeup)";
    }
    else{
      return String(wifi_pwr_mode);
    }
    
  }
   else if (var == "ap_chan") {
    return String(ap_chan);
  }
   else if (var == "ap_hidden") {
    if (ap_hidden == 0){
      return "No";
    }
    else if (ap_hidden == 1){
      return "Yes";
    }
    else {
    return String(ap_hidden);
    }
    
  }
  else {
    return String();
  }
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    log_i("Upload Start: %s", filename.c_str());
    beforeProgramming();
  }

  Serial.print("."); //to minimize serial output impact to speed, only print a dot every MTU

  for (size_t i = 0; i < len; i++) {
    pgmData_t pgm_send_data;
    pgm_send_data.addr = (index + i);
    pgm_send_data.data = data[i];
    while (xQueueSendToBack(programBufferQueue, (void *)&pgm_send_data, 1) == errQUEUE_FULL) {};
  }

  if (final) {
    Serial.println();
    log_i("UploadEnd: %s, %u B", filename.c_str(), index + len);
    xSemaphoreGiveFromISR(programFinishSemaphore, NULL);
  }
  vTaskDelay(1);
}

void ARDUINO_ISR_ATTR onTimer() {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void ARDUINO_ISR_ATTR onFlashExitBusy() {
  xSemaphoreGiveFromISR(flashExitBusySemaphore, NULL);
}

bool eraseChip() {
if (!in_progress) {
    log_e("Does not have control of interface!");
    return false;
  }
  state_change(STATE_ERASING);
  //xSemaphoreTake(flashExitBusySemaphore, 0);
  xQueueSendToBack(blinkerQueue, (void *)&BLINK_ON, 0);
  cmdCycle(0x000, 0xF0); //reset command
  log_i("Send chip erase command");
  cmdCycle(0xAAA, 0xAA);
  cmdCycle(0x555, 0x55);
  cmdCycle(0xAAA, 0x80);
  cmdCycle(0xAAA, 0xAA);
  cmdCycle(0x555, 0x55);
  cmdCycle(0xAAA, 0x10);
  vTaskDelay(2);
  int counter = 0;
  while (digitalRead(RY_BY_PIN) == LOW) {
    vTaskDelay(1000);
    counter++;
    log_i("Erasing Chip, %d seconds elapsed...", counter);
    if (counter >= 180) {
      log_e("Erase chip timeout failed!");
      xQueueSendToBack(blinkerQueue, (void *)&BLINK_FAST, 0);
      return false;
    }
  }
  log_i("Chip erase success!");
  //cmdCycle(0x000, 0xF0);

  prefs.begin("cfg");
  prefs.putString("rom_id", "");
  prefs.getString("rom_id", rom_id, (size_t)33);
  prefs.end();
  state_change(STATE_ERASED);
  xQueueSendToBack(blinkerQueue, (void *)&BLINK_NORMAL, 0);
  return true;
}

void IRAM_ATTR cmdCycle(uint32_t cmd_addr, uint8_t cmd_data) {
  //for (int i=0;i<20;i++){__asm__ __volatile__ ("NOP");}// more delay
  setAddr(cmd_addr);
  setData(cmd_data);
  for (int i = 0; i < 2; i++) { __asm__ __volatile__("NOP"); }
  /*timerAlarm(timer, 2, false, 1); //use timer to delay
  while(xSemaphoreTake(timerSemaphore, 0) == pdFALSE){}*/
  //GPIO.out_w1tc = ((uint32_t)1 << CE_PIN); //set CE low
  GPIO.out_w1tc = ((uint32_t)1 << WE_PIN); // set WE low
  /*timerAlarm(timer, 2, false, 1);//use timer to delay
  while(xSemaphoreTake(timerSemaphore, 0) == pdFALSE){}*/
  for (int i = 0; i < 3; i++) { __asm__ __volatile__("NOP"); } //use nop to delay
  //for (int i=0;i<20;i++){__asm__ __volatile__ ("NOP");}// more delay
  GPIO.out_w1ts = ((uint32_t)1 << WE_PIN); //set WE high
  //GPIO.out_w1ts = ((uint32_t)1 << CE_PIN); //set CE high
  //for (int i=0;i<3;i++){__asm__ __volatile__ ("NOP");} //use nop to delay
  //for (int i=0;i<20;i++){__asm__ __volatile__ ("NOP");} // more delay
  /*timerAlarm(timer, 2, false, 1); //use timer to delay
  while(xSemaphoreTake(timerSemaphore, 0) == pdFALSE){}*/
}

uint8_t readCycle(uint32_t addr) {
  setAddr(addr);
  //GPIO.out_w1tc = ((uint32_t)1 << CE_PIN);
  GPIO.out_w1tc = ((uint32_t)1 << OE_PIN);
  __asm__ __volatile__("NOP");
  __asm__ __volatile__("NOP");
  __asm__ __volatile__("NOP");
  __asm__ __volatile__("NOP");
  __asm__ __volatile__("NOP");
  __asm__ __volatile__("NOP");
  uint8_t res = getData();
  GPIO.out_w1ts = ((uint32_t)1 << OE_PIN);
  //GPIO.out_w1ts = ((uint32_t)1 << CE_PIN);
  return res;
}

bool beforeWriting(){
  if (!in_progress) {
    log_e("Does not have control of interface!");
    return false;
  }
  log_i("Setting data pins as OUTPUT and use dedic_gpio");
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  gpio_matrix_out(D0, CORE1_GPIO_OUT0_IDX, false, false);
  gpio_matrix_out(D1, CORE1_GPIO_OUT1_IDX, false, false);
  gpio_matrix_out(D2, CORE1_GPIO_OUT2_IDX, false, false);
  gpio_matrix_out(D3, CORE1_GPIO_OUT3_IDX, false, false);
  gpio_matrix_out(D4, CORE1_GPIO_OUT4_IDX, false, false);
  gpio_matrix_out(D5, CORE1_GPIO_OUT5_IDX, false, false);
  gpio_matrix_out(D6, CORE1_GPIO_OUT6_IDX, false, false);
  gpio_matrix_out(D7, CORE1_GPIO_OUT7_IDX, false, false);
  return true;
}

void beforeReading(){
  log_i("Setting data pins as INPUT");
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
}

bool beforeProgramming() {
  if (!takeControl()) {
    return false;
  }
  beforeWriting();
  state_change(STATE_PROGRAMMING);
  xQueueSendToBack(blinkerQueue, (void *)&BLINK_ON, 0);
  log_i("Before programming setup finished");
  //xSemaphoreTake(flashExitBusySemaphore, 0);
}



void afterProgramming() {
  log_i("Programming finished.");
  beforeReading();
  for (int i = 0; i < 0x20; i++) {
    char var = readCycle(i);
    if (i >= 0x0C && i <= 0x0F) {
      rom_id[i] = 0x20;
    } else {
      rom_id[i] = var;
    }
  }
  log_i("ROM ID: %s", rom_id);
  releaseControl();
  state_change(STATE_PROGRAMMED);
  prefs.begin("cfg");
  prefs.putString("rom_id", rom_id);
  prefs.end();
  xQueueSendToBack(blinkerQueue, (void *)&BLINK_NORMAL, 0);
}

void IRAM_ATTR programCycle(uint32_t addr, uint8_t data) {
  /*if (!in_progress){
    return false;
  }*/
  //xSemaphoreTake(flashExitBusySemaphore, 0);
  //cmdCycle((uint32_t)0x000, (uint8_t)0xF0);
  //program command
  cmdCycle((uint32_t)0xAAA, (uint8_t)0xAA);
  cmdCycle((uint32_t)0x555, (uint8_t)0x55);
  cmdCycle((uint32_t)0xAAA, (uint8_t)0xA0);
  cmdCycle(addr, data);
  for (int i = 0; i < 10; i++) { __asm__ __volatile__("NOP"); }  //programming takes some microseconds so this delay time doesn't matter
  //wait for RYBY to return to high
  taskENTER_CRITICAL(&myMutex);
  while (bitRead(GPIO.in1.val, RY_BY_PIN - 32) == 0) {}  //faster than interrupt+semphr
  taskEXIT_CRITICAL(&myMutex);
  //while(xSemaphoreTake(flashExitBusySemaphore, 0) == pdFALSE){}
  __asm__ __volatile__("NOP");
}

void IRAM_ATTR setAddr(uint32_t addr) {
  // 23 address bits, need to send 24
  uint8_t buf[3]; // implied 32 bit to 8 bit conversion (cut)
  buf[0] = addr;
  buf[1] = addr >> 8;
  buf[2] = addr >> 16;
  //uint32_t buf = ((addr & 0xFF0000) >> 16) | (addr & 0xFF00) | ((addr & 0x00FF) << 16); // use with transferBits
  spiSimpleTransaction(hspi->bus());
  spiWriteNL(hspi->bus(), &buf, 3);
  //spiTransferBitsNL(hspi->bus(), buf, NULL, 24); //too slow, need to implement own method
  spiEndTransaction(hspi->bus());
}

void IRAM_ATTR setData(uint8_t data) {
  // 8 data bits
  dedic_gpio_cpu_ll_write_all(data); //12ns?
  /*digitalWrite(D0, bitRead(data, 0));
  digitalWrite(D1, bitRead(data, 1));
  digitalWrite(D2, bitRead(data, 2));
  digitalWrite(D3, bitRead(data, 3));
  digitalWrite(D4, bitRead(data, 4));
  digitalWrite(D5, bitRead(data, 5));
  digitalWrite(D6, bitRead(data, 6));
  digitalWrite(D7, bitRead(data, 7));*/
}

uint8_t getData() {
  uint8_t res = bitRead(GPIO.in, D7) << 7 | bitRead(GPIO.in, D6) << 6 | bitRead(GPIO.in, D5) << 5 | bitRead(GPIO.in, D4) << 4 | bitRead(GPIO.in, D3) << 3 | bitRead(GPIO.in, D2) << 2 | bitRead(GPIO.in, D1) << 1 | bitRead(GPIO.in, D0);
  //uint8_t res = digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0);
  return res;
}

bool takeControl() {
  if (!in_progress) {
    in_progress = true;
  } else {
    log_e("Busy, cannot operate now");
    return false;
  }

  pinMode(IO_OE_PIN, OUTPUT);
  digitalWrite(IO_OE_PIN, HIGH);
  pinMode(SENS_CTL_PIN, OUTPUT);
  digitalWrite(SENS_CTL_PIN, HIGH);

  beforeReading();

  pinMode(SR_OE_PIN, OUTPUT);
  digitalWrite(SR_OE_PIN, LOW);
  setData(0);
  setAddr(0);
  pinMode(WE_PIN, OUTPUT);
  digitalWrite(WE_PIN, HIGH);
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);
  pinMode(CE_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);
  log_i("Should have control of interface now");
  return true;
}

bool releaseControl() {
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
  pinMode(WE_PIN, INPUT);
  pinMode(OE_PIN, INPUT);
  pinMode(CE_PIN, INPUT);
  pinMode(SR_OE_PIN, INPUT);
  pinMode(IO_OE_PIN, INPUT);
  pinMode(SENS_CTL_PIN, INPUT);
  in_progress = false;
  log_i("Released control of interface");
  return true;
}

void startWifi() {
  if (!wifi_started) {
    wifi_started = true;
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(BTN_PIN, INPUT);
    pinMode(RY_BY_PIN, INPUT);
    releaseControl();
    if (!WiFi.softAP(ap_name, ap_pass, ap_chan, ap_hidden)) {
      log_e("Soft AP creation failed.");
      while (1)
        ;
    }
    server.begin();
    log_w("Wifi and server started");
    xQueueSendToBack(blinkerQueue, (void *)&BLINK_NORMAL, 0);
    BLEDevice::stopAdvertising();
    BLEDevice::deinit(); //force bluetooth pairing request to fail
  }
}

class MyBLECallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    log_w("Got connection on BLE");
    startWifi();
  };
};

void stopWifi() {
  server.end();
  WiFi.mode(WIFI_MODE_STA);
  if (wifi_pwr_mode == WIFI_OFF_BLE_WAKE) {
    BLEDevice::init(ap_name);
    BLEServer *pServer = BLEDevice::createServer();

    BLEHIDDevice *hid = new BLEHIDDevice(pServer);
    hid->manufacturer()->setValue("Connor Zheng");
    hid->pnp(0x02, 0x05AC, 0x820A, 0x0210);
    hid->hidInfo(0x00, 0x01);
    //BLESecurity* pSecurity = new BLESecurity();
    //pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
    hid->startServices();

    //BLEService *pService = pServer->createService("03b80e5a-ede8-4b33-a751-6ce34ec4c700");
    //BLECharacteristic *pCharacteristic = pService->createCharacteristic("7772e5db-3868-4112-a1a9-f2669d106bf3", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    //pCharacteristic->setValue("Hello World");
    //pService->start();
    pServer->setCallbacks(new MyBLECallbacks());
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    //pAdvertising->addServiceUUID("03b80e5a-ede8-4b33-a751-6ce34ec4c700");
    pAdvertising->setAppearance(ESP_BLE_APPEARANCE_HID_KEYBOARD);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    pAdvertising->setMinInterval(0x20);
    pAdvertising->setMaxInterval(0x80);
    BLEDevice::startAdvertising();
  }
  wifi_started = false;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  log_i("SR-JV32 WiFi Version %s starting...", FIRMWARE_VERSION);
  /*
#if CONFIG_PM_ENABLE
  log_w("PM functions enabled");
  // Configure dynamic frequency scaling:
  // maximum and minimum frequencies are set in sdkconfig,
  // automatic light sleep is enabled if tickless idle support is enabled.
  // esp_pm_config_esp32c3_t pm_config = { // old version
  esp_pm_config_t pm_config = {
    .max_freq_mhz = 240,  //  e.g. 80, 160,
    .min_freq_mhz = 40,   // 40
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    .light_sleep_enable = true
#endif
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif  // CONFIG_PM_ENABLE
  esp_pm_config_t pm_config;
  esp_pm_get_configuration((void *)&pm_config);
  log_i("Max Freq: %d, Min Freq: %d, Auto Light Sleep: %d", pm_config.max_freq_mhz, pm_config.min_freq_mhz, pm_config.light_sleep_enable);
  */

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BTN_PIN, INPUT);
  pinMode(RY_BY_PIN, INPUT);
  pinMode(SR_OE_PIN, INPUT);
  pinMode(SENS_CTL_PIN, INPUT);
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
  pinMode(WE_PIN, INPUT);
  pinMode(OE_PIN, INPUT);
  pinMode(CE_PIN, INPUT);
  pinMode(IO_OE_PIN, INPUT);

  hspi = new SPIClass(HSPI);
  hspi->begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  pinMode(hspi->pinSS(), OUTPUT);
  hspi->setHwCs(true);
  hspi->setFrequency(SPI_SPEED_MHZ * 1000000);
  hspi->setBitOrder(MSBFIRST);
  hspi->setDataMode(SPI_MODE0);

  releaseControl();

  /*
  // HW TIMER config
  timer = timerBegin(40000000);
  if (!timer) {
    log_e("Cannot start TIMER");
    xQueueSendToBack(blinkerQueue, (void *)&BLINK_FAST, 0);
  }

  timerSemaphore = xSemaphoreCreateBinary();
  timerAttachInterrupt(timer, &onTimer);*/

  eraseSemaphore = xSemaphoreCreateBinary();
  flashExitBusySemaphore = xSemaphoreCreateBinary();

  restartSemaphore = xSemaphoreCreateBinary();
  programFinishSemaphore = xSemaphoreCreateBinary();

  //attachInterrupt(RY_BY_PIN, onFlashExitBusy, RISING);

  blinkerQueue = xQueueCreate(10, sizeof(uint8_t));
  xTaskCreate(blinkerTask, "Blinker", 2048, (void *)NULL, 10, &blinkerHandle);

  programBufferQueue = xQueueCreate(1500, sizeof(pgmData_t));
  xTaskCreatePinnedToCore(programmerTask, "Programmer", 8192, (void *)NULL, 20, &programmerHandle, 1);

  if (!prefs.begin("cfg")) {
    // first boot
    log_w("First boot, restart prefs");
    prefs.end();
    prefs.begin("cfg");
  }

  if (digitalRead(BTN_PIN) == LOW) {
    log_w("RESET button is pressed!");
    xQueueSendToBack(blinkerQueue, (void *)&BLINK_FAST, 0);
    vTaskDelay(3000);
    if (digitalRead(BTN_PIN) == LOW) {
      //reset still pressed, reset nvs
      log_w("Reset prefs");
      prefs.clear();
      prefs.end();
      log_w("Cleared preferences");
      xQueueSendToBack(blinkerQueue, (void *)&BLINK_ON, 0);
      while (digitalRead(BTN_PIN) == LOW) {
        // when key is still held down after reset, do nothing
        vTaskDelay(100);
      }
      log_w("Restart Now!");
      esp_restart();
    } else {
      // key released before 3 seconds
      log_w("Key released, not resetting");
      xQueueSendToBack(blinkerQueue, (void *)&BLINK_NORMAL, 0);
    }
  }

  if (prefs.isKey("ap_chan")) {
    // if config info is not empty then read config
    machine_state = prefs.getUChar("state");
    log_i("Get Status from nvs: %d", machine_state);
    wifi_pwr_mode = prefs.getUChar("wifi_pwr_mode");
    log_i("Get wireless power mode from nvs: %d", wifi_pwr_mode);
    ap_chan = prefs.getUChar("ap_chan");
    log_i("Get WiFi Channel from nvs: %d", ap_chan);
    ap_hidden = prefs.getUChar("ap_hidden");
    log_i("Get WiFi hidden from nvs: %d", ap_hidden);
    prefs.getString("ap_name", ap_name, (size_t)32);
    log_i("Get SSID from nvs: %s", ap_name);
    prefs.getString("ap_pass", ap_pass, (size_t)32);
    log_i("Get AP password from nvs: %s", ap_pass);
    prefs.getString("rom_id", rom_id, (size_t)33);
    log_i("Get ROM ID from nvs: %s", rom_id);

  } else {
    // system reset or first boot
    log_w("Empty cfg!");
    //get mac address
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    Serial.print("MAC Address: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(mac[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    sprintf(ap_name, "SR-JV80-%02X%02X%02X", mac[3], mac[4], mac[5]);
    // reset config info
    log_w("Setting default AP name: %s", ap_name);
    prefs.putUChar("state", machine_state);
    prefs.putUChar("wifi_pwr_mode", wifi_pwr_mode);
    prefs.putString("ap_name", ap_name);
    prefs.putString("ap_pass", "");
    prefs.putString("rom_id", "");
    prefs.putUChar("ap_chan", ap_chan);
    prefs.putUChar("ap_hidden", ap_hidden);
  }
  prefs.end();
  /*
  for (int i = 0; i < sizeof (ap_name); i++)
  {
    Serial.printf("%02X ", ap_name[i]);
  }
  Serial.println();*/
  

  if (machine_state == STATE_ERASING || machine_state == STATE_PROGRAMMING) {
    state_change(STATE_UNKNOWN);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", config_html, processor);
  });
  server.on("/rom", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", rom_html, processor);
  });
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/");
    xSemaphoreGive(restartSemaphore);
  });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    char status[4];
    sprintf(status, "%d", machine_state);
    request->send(200, "text/text", status);
  });
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("ap_name")) {
      inputMessage = request->getParam("ap_name")->value();
      prefs.begin("cfg");
      inputMessage.toCharArray(ap_name, 32);
      prefs.putString("ap_name", ap_name);
      prefs.end();
    } else if (request->hasParam("ap_pass")) {
      inputMessage = request->getParam("ap_pass")->value();
      prefs.begin("cfg");
      inputMessage.toCharArray(ap_pass, 32);
      prefs.putString("ap_pass", ap_pass);
      prefs.end();
    } else if (request->hasParam("wifi_pwr_mode")) {
      inputMessage = request->getParam("wifi_pwr_mode")->value();
      prefs.begin("cfg");
      wifi_pwr_mode = inputMessage.toInt();
      prefs.putUChar("wifi_pwr_mode", wifi_pwr_mode);
      prefs.end();
    } 
    else if (request->hasParam("ap_chan")) {
      inputMessage = request->getParam("ap_chan")->value();
      prefs.begin("cfg");
      ap_chan = inputMessage.toInt();
      prefs.putUChar("ap_chan", ap_chan);
      prefs.end();
    }
    else if (request->hasParam("ap_hidden")) {
      inputMessage = request->getParam("ap_hidden")->value();
      prefs.begin("cfg");
      ap_hidden = inputMessage.toInt();
      prefs.putUChar("ap_hidden", ap_hidden);
      prefs.end();
    } else {
      inputMessage = "No message sent";
    }
    log_w("%s", inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.on("/erase", HTTP_GET, [](AsyncWebServerRequest *request) {
    xSemaphoreGive(eraseSemaphore);
    request->send(200, "text/text", "OK");
  });
  server.on(
    "/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(200);
    },
    handleUpload);
  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", ota_html);
  });
  server.on(
    "/update", HTTP_POST, handleUpdateEnd, handleUpdate);
  
  server.onFileUpload(handleUpload);
  server.onNotFound(notFound);

  xQueueSendToBack(blinkerQueue, (void *)&BLINK_OFF, 0);
  xTaskCreatePinnedToCore(housekeepingTask, "Housekeeper", 8192, (void *)NULL, 2, &housekeepingHandle, 1);
  if (wifi_pwr_mode == WIFI_ALWAYS_ON) {
    startWifi();
  }

  /*
  Power-on test code here
  */
}



void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
}

void housekeepingTask(void *pvParameters) {
  log_i("Housekeep - task started");
  if (wifi_pwr_mode != WIFI_ALWAYS_ON) {
    stopWifi();
  }
  vTaskDelay(1000);
  while (1) {
    if (xSemaphoreTake(eraseSemaphore, 0) == pdTRUE) {
      takeControl();
      vTaskDelay(20);
      beforeWriting();
      eraseChip();
      releaseControl();
    }
    if (xSemaphoreTake(restartSemaphore, 0) == pdTRUE) {
      vTaskDelay(500);
      ESP.restart();
    }
    if (wifi_pwr_mode >= WIFI_OFF_BLE_WAKE && wifi_started == false) {
      if (digitalRead(BTN_PIN) == LOW) {
        startWifi();
      } else {
        vTaskDelay(500);
        if (wifi_started == false) {
          log_d("Going to light sleep now");
          Serial.flush();
          //esp_sleep_enable_bt_wakeup();
          gpio_wakeup_enable(GPIO_NUM_39, GPIO_INTR_LOW_LEVEL);
          gpio_sleep_set_pull_mode(GPIO_NUM_39, GPIO_PULLUP_ONLY);
          gpio_sleep_set_direction(GPIO_NUM_39, GPIO_MODE_INPUT);
          esp_sleep_enable_gpio_wakeup();
          gpio_hold_en(GPIO_NUM_38);

          if (wifi_pwr_mode == WIFI_OFF_BLE_WAKE) {
            esp_sleep_enable_timer_wakeup(800 * 1000 + (esp_random() >> 12));
          }
          esp_light_sleep_start();

          gpio_hold_dis(GPIO_NUM_38);
          gpio_wakeup_disable(GPIO_NUM_39);
          gpio_intr_disable(GPIO_NUM_39);

          const char *wakeup_reason;
          switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_GPIO:
              wakeup_reason = "pin";
              startWifi();
              break;
            case ESP_SLEEP_WAKEUP_BT:
              wakeup_reason = "BT";
              break;
            case ESP_SLEEP_WAKEUP_TIMER:
              wakeup_reason = "TIMER";
              
              BLEDevice::startAdvertising();
              break;
            default:
              wakeup_reason = "other";
              break;
          }

          log_i("WAKEUP REASON: %d %s", esp_sleep_get_wakeup_cause(), wakeup_reason);
        }
      }
    }
    vTaskDelay(10);
  }
}


void blinkerTask(void *pvParameters) {
  log_i("Blinker - task started");
  uint8_t mode = 0;
  uint8_t buf = 0;
  while (1) {
    if (blinkerQueue != NULL) {

      if (xQueueReceive(blinkerQueue, &buf, 0)) {
        mode = buf;
      }
    }
    switch (mode) {
      case 0:
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(500);
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(500);
        break;
      case 1:
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(50);
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(50);
        break;
      case 2:
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(50);
        break;
      case 3:
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(50);
        break;
      default:
        mode = 0;
    }
  }
}

void programmerTask(void *pvParameters) {
  pgmData_t buf;
  while (1) {
    if (xQueueReceive(programBufferQueue, &buf, 1)) {
      //receive queue and program flash
      programCycle(buf.addr, buf.data);
    } else if (xSemaphoreTake(programFinishSemaphore, 0) == pdTRUE) {
      //programming is finished
      log_i("Wrote %d bytes.", buf.addr + 1);
      afterProgramming();
    } else {
      taskYIELD();
    }
  }
}
