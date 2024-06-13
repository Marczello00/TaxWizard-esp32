#include <FS.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncFsWebServer.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer/src/AsyncJson.h>
#include <base64.h>

#define LED_PIN 65
#define LED_COUNT 1

typedef struct
{
  unsigned short creditCount;
  unsigned short inputPin;
} TransactionData;

typedef struct
{
  unsigned short creditCount;
  unsigned short outputPin;
  bool shouldBeTaxed;
} OutputTransactionData;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

const unsigned short inputPinOfComesteroMoney = 4;
const unsigned short inputPinOfComesteroToken = 5;
const unsigned short inputPinOfNayaxCreditCard = 6;
const unsigned short inputPinOfNayaxPrepaidCard = 7;
const unsigned short outputPinOfCarWashReceiverCH1 = 11;
const unsigned short outputPinOfCarWashReceiverCH2 = 12;
const unsigned short outputPinOfCashRegister = 13;
const unsigned short outputPinOfNayaxSupply = 17;
const unsigned short outputPinOfComesteroSupply = 18;

bool isTaxingEnabled;
String salt = "sól";

const unsigned long inputSignalWidth = 15;
const unsigned long outputSignalWidth = 150;
const unsigned long endOfInputSequenceWidth = 100;

QueueHandle_t TransactionsQueue;

IPAddress myIP;
AsyncFsWebServer server(80, LittleFS, "myServer");
int testInt = 150;
float testFloat = 123.456;

// Functions declarations

void listenToInputTask(void *pvParameters);
void sendOutputDataTask(void *pvParameters);
bool startFilesystem();
void getFsInfo(fsInfo_t *fsInfo);
void handleTaxingRequest(AsyncWebServerRequest *request);
void handleTaxingJsonRequest(AsyncWebServerRequest *request, JsonVariant &requestJson);
void handleTransactionJsonRequest(AsyncWebServerRequest *request, JsonVariant &requestJson);
bool isChecksumValid(String time, String checksum);
bool assignNewTaxing(unsigned short newTaxing);
void initPins();
bool startMoneyProcessingSystem();
void sendTransactionToPin(OutputTransactionData outcomingTransaction);
bool shouldThisPinBeTaxed(unsigned short pin);
unsigned short getOutputPin(unsigned short inputPin);
void logTransaction(OutputTransactionData transaction);
void logTransaction(TransactionData transaction);
bool loadTaxingStatus();
void saveTaxingStatus();

void setup()
{
  Serial.begin(115200);
  delay(10);
  initPins();
  delay(1000);

  /* Start FileSystem */
  if (startFilesystem())
  {
    Serial.println("LittleFS filesystem ready!");
    File config = server.getConfigFile("r");
    if (config)
    {
      DynamicJsonDocument doc(config.size() * 1.33);
      deserializeJson(doc, config);
      testInt = doc["Test int variable"];
      testFloat = doc["Test float variable"];
    }
    Serial.printf("Stored \"testInt\" value: %d\n", testInt);
    Serial.printf("Stored \"testFloat\" value: %3.2f\n", testFloat);
  }
  else
  {
    Serial.println("LittleFS error!");
  }

  // Start WiFi
  myIP = server.startWiFi(15000, "ESP32_AP1234", "123456789");
  WiFi.setSleep(WIFI_PS_NONE);
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected!");
    delay(1000);
    // ESP.restart(); //WORKAROUND! DO NOT USE IT UNLESS YOU HAVE TO!
  }

  // Config server
  server.addOptionBox("Custom options");
  server.addOption("Test int variable", testInt);
  server.addOption("Test float variable", testFloat);
  server.setSetupPageTitle("Simple Async ESP FS WebServer");
  // Enable ACE FS file web editor and add FS info callback fucntion
  server.enableFsCodeEditor();
  server.setFsInfoCallback(getFsInfo);
  // Listening to routes
  server.on("/taxing", HTTP_GET, handleTaxingRequest);
  AsyncCallbackJsonWebHandler *taxingPostHandler = new AsyncCallbackJsonWebHandler("/taxing", handleTaxingJsonRequest);
  server.addHandler(taxingPostHandler);
  AsyncCallbackJsonWebHandler *transactionPostHandler = new AsyncCallbackJsonWebHandler("/transaction", handleTransactionJsonRequest);
  server.addHandler(transactionPostHandler);
  // Start server
  server.init();

  Serial.print(F("Async ESP Web Server started on IP Address: "));
  Serial.println(myIP);
  Serial.println(F(
      "Open /setup page to configure optional parameters.\n"
      "Open /edit page to view, edit or upload example or your custom webserver source files."));

  if(!loadTaxingStatus())
    assignNewTaxing(1);
  startMoneyProcessingSystem();
}

void loop()
{
  vTaskDelete(NULL);
}

/*
 * Task to listen to incoming signals from given input pin, merging them into transaction and sending to queue's back
 * pvParameters is a pointer to the input pin number of given device (int)
 */
void listenToInputTask(void *pvParameters)
{
  unsigned short *pinToReadPtr = (unsigned short *)pvParameters;
  unsigned short inputPin = *pinToReadPtr;

  Serial.print("Nasluchwianie na pinie: ");
  Serial.println(inputPin);

  bool blockInputFlag = true;
  unsigned int creditCount = 0;
  unsigned long currentTime = 0;
  unsigned long lastActivationTime = 0;

  TransactionData incomingTransaction;
  incomingTransaction.inputPin = inputPin;

  for (;;)
  {
    int inputState = digitalRead(inputPin);
    currentTime = millis();

    if (inputState == LOW)
    {
      if (!blockInputFlag && currentTime - lastActivationTime >= inputSignalWidth)
      {
        lastActivationTime = currentTime;
        blockInputFlag = true;
        creditCount++;
      }
      else
      {
        delay(10);
      }
    }
    else
    {
      blockInputFlag = false;
    }
    delay(10);

    if (currentTime - lastActivationTime >= endOfInputSequenceWidth)
      if (creditCount > 0)
      {
        incomingTransaction.creditCount = creditCount;
        if (xQueueSendToBack(TransactionsQueue, &incomingTransaction, (TickType_t)10) == pdPASS)
          logTransaction(incomingTransaction);
        else
          Serial.println("Błąd przy dodawaniu do kolejki!");
        creditCount = 0;
      }
  }
}

/*
 * Task to retrieve transactions from queue and send them to outputs based on their origin and taxing status
 * pvParameters is not used
 */
void sendOutputDataTask(void *pvParameters)
{
  Serial.println("Zadanie wysylania rozpoczete");
  TransactionData transaction;

  for (;;)
  {
    if (xQueueReceive(TransactionsQueue, &transaction, (TickType_t)10) == pdPASS)
    {
      OutputTransactionData outputTransaction;
      outputTransaction.creditCount = transaction.creditCount;
      outputTransaction.outputPin = getOutputPin(transaction.inputPin);
      outputTransaction.shouldBeTaxed = shouldThisPinBeTaxed(transaction.inputPin);
      sendTransactionToPin(outputTransaction);
      logTransaction(outputTransaction);
    }
    else
    {
      delay(1000);
    }
  }
}

void initPins()
{
  // Inputs
  pinMode(inputPinOfComesteroMoney, INPUT_PULLUP);
  pinMode(inputPinOfComesteroToken, INPUT_PULLUP);
  pinMode(inputPinOfNayaxCreditCard, INPUT_PULLUP);
  pinMode(inputPinOfNayaxPrepaidCard, INPUT_PULLUP);
  // Outputs
  pinMode(outputPinOfCarWashReceiverCH1, OUTPUT);
  pinMode(outputPinOfCarWashReceiverCH2, OUTPUT);
  pinMode(outputPinOfCashRegister, OUTPUT);
  // Supply
  pinMode(outputPinOfNayaxSupply, OUTPUT);
  digitalWrite(outputPinOfNayaxSupply, HIGH);
  pinMode(outputPinOfComesteroSupply, OUTPUT);
  digitalWrite(outputPinOfComesteroSupply, HIGH);
}

bool startFilesystem()
{
  if (LittleFS.begin())
  {
    File root = LittleFS.open("/", "r");
    File file = root.openNextFile();
    while (file)
    {
      Serial.printf("FS File: %s, size: %d\n", file.name(), file.size());
      file = root.openNextFile();
    }
    return true;
  }
  else
  {
    Serial.println("ERROR on mounting filesystem. It will be formmatted!");
    LittleFS.format();
    ESP.restart();
  }
  return false;
}

void getFsInfo(fsInfo_t *fsInfo)
{
  fsInfo->totalBytes = LittleFS.totalBytes();
  fsInfo->usedBytes = LittleFS.usedBytes();
  strcpy(fsInfo->fsName, "LittleFS");
}

void handleTaxingRequest(AsyncWebServerRequest *request)
{
  unsigned short responseCode = 200;
  JsonDocument responseDoc;
  JsonObject responseJsonObj = responseDoc.to<JsonObject>();
  String responseBody = "";
  responseJsonObj["taxing"] = isTaxingEnabled;
  serializeJson(responseDoc, responseBody);
  request->send(responseCode, "application/json", responseBody);
}

void handleTaxingJsonRequest(AsyncWebServerRequest *request, JsonVariant &requestJson)
{
  JsonObject requestJsonObj = requestJson.as<JsonObject>();
  unsigned short newTaxing = requestJsonObj["taxing"] | -1;
  String time = requestJsonObj["time"] | " ";
  String checksum = requestJsonObj["checksum"] | " ";
  unsigned short responseCode = 500;
  JsonDocument responseDoc;
  JsonObject responseJsonObj = responseDoc.to<JsonObject>();
  String responseBody = "";
  if (isChecksumValid(time, checksum))
    if (assignNewTaxing(newTaxing))
    {
      responseCode = 200;
      responseJsonObj["taxing"] = isTaxingEnabled;
    }
    else
      responseCode = 400;
  else
    responseCode = 401;
  serializeJson(responseDoc, responseBody);
  request->send(responseCode, "application/json", responseBody);
}

void handleTransactionJsonRequest(AsyncWebServerRequest *request, JsonVariant &requestJson)
{
  JsonObject requestJsonObj = requestJson.as<JsonObject>();
  unsigned short creditCount = requestJsonObj["creditCount"] | 0;
  unsigned short responseCode = 500;
  if (creditCount > 0 && creditCount <= 20)
  {
    TransactionData transaction;
    transaction.creditCount = creditCount;
    transaction.inputPin = inputPinOfNayaxPrepaidCard;
    if (xQueueSendToBack(TransactionsQueue, &transaction, (TickType_t)10) == pdPASS)
    {
      logTransaction(transaction);
      responseCode = 200;
    }
    else
      responseCode = 507;
  }
  else
    responseCode = 400;
  JsonDocument responseDoc;
  JsonObject responseJsonObj = responseDoc.to<JsonObject>();
  String responseBody = "";

  serializeJson(responseDoc, responseBody);
  request->send(responseCode, "application/json", responseBody);
}

bool isChecksumValid(String time, String checksum)
{
  time = time + salt;
  String calculatedChecksum = base64::encode(time);
  if (calculatedChecksum == checksum)
    return true;
  else
    return false;
}

bool assignNewTaxing(unsigned short newTaxing)
{
  if (newTaxing == 1)
    if(isTaxingEnabled)
      return true;
    else
    {
      isTaxingEnabled = true;
      saveTaxingStatus();
      return true;
    }
  else if (newTaxing == 0)
    if(isTaxingEnabled)
    {
      isTaxingEnabled = false;
      saveTaxingStatus();
      return true;
    }
    else
      return true;
  else
    return false;
}

bool startMoneyProcessingSystem()
{
  TransactionsQueue = xQueueCreate(10, sizeof(TransactionData));
  // Receiving data from comestero money channel
  xTaskCreate(listenToInputTask, "Task1", 10000, (void *)&inputPinOfComesteroMoney, 1, &Task1);
  delay(500);
  // Receiving data from comestero token channel
  xTaskCreate(listenToInputTask, "Task2", 10000, (void *)&inputPinOfComesteroToken, 1, &Task2);
  delay(500);
  // Receiving data from nayax credit card channel
  xTaskCreate(listenToInputTask, "Task3", 10000, (void *)&inputPinOfNayaxCreditCard, 1, &Task3);
  delay(500);
  // Receiving data from nayax prepaid card channel
  xTaskCreate(listenToInputTask, "Task4", 10000, (void *)&inputPinOfNayaxPrepaidCard, 1, &Task4);
  delay(500);
  // Sending data to outputs
  xTaskCreate(sendOutputDataTask, "Task5", 10000, NULL, 1, &Task5);
  delay(500);
  // Start the 24V supply to Comestero and Nayax devices
  digitalWrite(outputPinOfNayaxSupply, LOW);
  digitalWrite(outputPinOfComesteroSupply, LOW);
  return true;
}

void sendTransactionToPin(OutputTransactionData outcomingTransaction)
{
  for (int i = 0; i < outcomingTransaction.creditCount; i++)
  {
    digitalWrite(outcomingTransaction.outputPin, HIGH);
    if (outcomingTransaction.shouldBeTaxed)
      digitalWrite(outputPinOfCashRegister, HIGH);
    delay(outputSignalWidth);

    digitalWrite(outcomingTransaction.outputPin, LOW);
    if (outcomingTransaction.shouldBeTaxed)
      digitalWrite(outputPinOfCashRegister, LOW);
    delay(outputSignalWidth);
  }
}

bool shouldThisPinBeTaxed(unsigned short pin)
{
  if (pin == inputPinOfNayaxCreditCard ||
      (pin == inputPinOfComesteroMoney && isTaxingEnabled))
    return true;
  else
    return false;
}

unsigned short getOutputPin(unsigned short inputPin)
{
  if (inputPin == inputPinOfNayaxCreditCard || inputPin == inputPinOfNayaxPrepaidCard)
    return outputPinOfCarWashReceiverCH2;
  else
    return outputPinOfCarWashReceiverCH1;
}

void logTransaction(OutputTransactionData transaction)
{
  Serial.print("Wartosc: ");
  Serial.print(transaction.creditCount);
  Serial.print(" do pinu: ");
  Serial.print(transaction.outputPin);
  transaction.shouldBeTaxed ? Serial.println(" zafiskalizowana") : Serial.println(" niezafiskalizowana");
}

void logTransaction(TransactionData transaction)
{
  Serial.print("Wartosc: ");
  Serial.print(transaction.creditCount);
  Serial.print(" z pinu: ");
  Serial.println(transaction.inputPin);
}

bool loadTaxingStatus()
{
  File taxingFile = LittleFS.open("/config/lastTaxingStatus.json", "r");
  if (!taxingFile)
    return false;
  JsonDocument doc;
  deserializeJson(doc, taxingFile);
  isTaxingEnabled = doc["taxing"];
  taxingFile.close();
  return true;
}

void saveTaxingStatus()
{
  File taxingFile = LittleFS.open("/config/lastTaxingStatus.json", "w");
  JsonDocument doc;
  doc["taxing"] = isTaxingEnabled;
  serializeJson(doc, taxingFile);
  taxingFile.close();
}