#include <FS.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncFsWebServer.h>
//#include <Adafruit_NeoPixel.h> //crash?

#define LED_PIN 65   
#define LED_COUNT 1  
//Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

typedef struct {
  unsigned int creditCount;
  unsigned int inputPin;
} TransactionData;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

const int inputPinOfComesteroMoney = 4;
const int inputPinOfComesteroToken = 5;
const int inputPinOfNayaxCreditCard = 6;
const int inputPinOfNayaxPrepaidCard = 7;
const int outputPinOfCarWashReceiverCH1 = 11;
const int outputPinOfCarWashReceiverCH2 = 12;
const int outputPinOfCashRegister = 13;

bool isTaxingEnabled = false;

const unsigned long inputSignalWidth = 15;
const unsigned long outputSignalWidth = 150;
const unsigned long endOfInputSequenceWidth = 100;

QueueHandle_t TransactionsQueue;

AsyncFsWebServer server(80, LittleFS, "myServer");
int testInt = 150;
float testFloat = 123.456;


//Functions declarations

void listenToInputTask(void* pvParameters);
void sendOutputDataTask(void* pvParameters);
bool startFilesystem();
void getFsInfo(fsInfo_t* fsInfo);
void handleTaxingRequest(AsyncWebServerRequest* request);


void setup() {
  //strip.begin();
  //strip.setPixelColor(0, strip.Color(255,165,0)); //set to orange
  //strip.show();
  Serial.begin(115200);
  delay(10);
  pinMode(inputPinOfComesteroMoney, INPUT_PULLUP);
  pinMode(inputPinOfComesteroToken, INPUT_PULLUP);
  pinMode(inputPinOfNayaxCreditCard, INPUT_PULLUP);
  pinMode(inputPinOfNayaxPrepaidCard, INPUT_PULLUP);
  pinMode(outputPinOfCarWashReceiverCH1, OUTPUT);
  pinMode(outputPinOfCarWashReceiverCH2, OUTPUT);
  pinMode(outputPinOfCashRegister, OUTPUT);

  TransactionsQueue = xQueueCreate(10, sizeof(TransactionData));
  delay(1000);

  /* Start FileSystem */
  if (startFilesystem()) {
    Serial.println("LittleFS filesystem ready!");
    File config = server.getConfigFile("r");
    if (config) {
      DynamicJsonDocument doc(config.size() * 1.33);
      deserializeJson(doc, config);
      testInt = doc["Test int variable"];
      testFloat = doc["Test float variable"];
    }
    Serial.printf("Stored \"testInt\" value: %d\n", testInt);
    Serial.printf("Stored \"testFloat\" value: %3.2f\n", testFloat);
  } else {
    Serial.println("LittleFS error!");
  }

  //Start WiFi
  Serial.println("lol1");
  IPAddress myIP = server.startWiFi(15000, "ESP32_AP1234", "123456789");
  WiFi.setSleep(WIFI_PS_NONE);
  Serial.println("lol2");
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    delay(1000);
    //ESP.restart(); //WORKAROUND! DO NOT USE IT UNLESS YOU HAVE TO!
  }

  //Config server
  server.addOptionBox("Custom options");
  server.addOption("Test int variable", testInt);
  server.addOption("Test float variable", testFloat);
  server.setSetupPageTitle("Simple Async ESP FS WebServer");
  // Enable ACE FS file web editor and add FS info callback fucntion
  server.enableFsCodeEditor();
  server.setFsInfoCallback(getFsInfo);
  //Listening to routes
  server.on("/taxing", HTTP_GET, handleTaxingRequest);

  // Start server
  server.init();
  Serial.print(F("Async ESP Web Server started on IP Address: "));
  Serial.println(myIP);
  Serial.println(F(
    "Open /setup page to configure optional parameters.\n"
    "Open /edit page to view, edit or upload example or your custom webserver source files."));


  //Starting IO tasks
  //Receiving data from comestero money channel
  xTaskCreate(listenToInputTask, "Task1", 10000, (void*)&inputPinOfComesteroMoney, 1, &Task1);
  delay(500);
  //Receiving data from comestero token channel
  xTaskCreate(listenToInputTask, "Task2", 10000, (void*)&inputPinOfComesteroToken, 1, &Task2);
  delay(500);
  //Receiving data from nayax credit card channel
  xTaskCreate(listenToInputTask, "Task3", 10000, (void*)&inputPinOfNayaxCreditCard, 1, &Task3);
  delay(500);
  //Receiving data from nayax prepaid card channel
  xTaskCreate(listenToInputTask, "Task4", 10000, (void*)&inputPinOfNayaxPrepaidCard, 1, &Task4);
  delay(500);
  //Sending data to outputs
  xTaskCreate(sendOutputDataTask, "Task5", 10000, NULL, 1, &Task5);
  delay(500);
  
  //strip.setPixelColor(0, strip.Color(255,165,0)); //set to orange
  //strip.show();
}

void loop() {
  vTaskDelete(NULL);
}


/*
* Task to listen to incoming signals from given input pin, merging them into transaction and sending to queue's back
* pvParameters is a pointer to the input pin number of given device (int)
*/
void listenToInputTask(void* pvParameters) {
  Serial.print("Task input running on core ");
  Serial.println(xPortGetCoreID());

  int* pinToReadPtr = (int*)pvParameters;
  int inputPin = *pinToReadPtr;

  bool blockInputFlag = true;
  unsigned int creditCount = 0;
  unsigned long currentTime = 0;
  unsigned long lastActivationTime = 0;

  TransactionData incomingTransaction;
  incomingTransaction.inputPin = inputPin;

  for (;;) {
    int inputState = digitalRead(inputPin);
    currentTime = millis();

    if (inputState == LOW) {
      if (!blockInputFlag && currentTime - lastActivationTime >= inputSignalWidth) {
        lastActivationTime = currentTime;
        blockInputFlag = true;
        creditCount++;
      } else {
        delay(10);
      }
    } else {
      blockInputFlag = false;
    }

    if (currentTime - lastActivationTime >= endOfInputSequenceWidth)
      if (creditCount > 0) {
        incomingTransaction.creditCount = creditCount;
        if (xQueueSendToBack(TransactionsQueue, &incomingTransaction, (TickType_t)10) == pdPASS) {
          Serial.print("Dodano do kolejki: ");
          Serial.println(String(creditCount) + " z pinu " + String(inputPin));
        } else {
          Serial.println("Błąd przy dodawaniu do kolejki!");
        }
        creditCount = 0;
      }
  }
}

/*
* Task to retrieve transactions from queue and send them to outputs based on their origin and taxing status
* pvParameters is not used
*/
void sendOutputDataTask(void* pvParameters) {
  Serial.print("Task output running on core ");
  Serial.println(xPortGetCoreID());
  TransactionData transaction;
  delay(1000);

  for (;;) {
    if (xQueueReceive(TransactionsQueue, &transaction, (TickType_t)10) == pdPASS) {
      Serial.print("Zdjęto z kolejki: ");
      Serial.print(transaction.creditCount);
      Serial.print(" z pinu: ");
      Serial.print(transaction.inputPin);

      if (isTaxingEnabled && (transaction.inputPin == inputPinOfComesteroMoney || transaction.inputPin == inputPinOfNayaxCreditCard)) {
        for (int i = 0; i < transaction.creditCount; i++) {
          digitalWrite(outputPinOfCarWashReceiverCH1, HIGH);
          digitalWrite(outputPinOfCashRegister, HIGH);
          delay(outputSignalWidth);
          digitalWrite(outputPinOfCarWashReceiverCH1, LOW);
          digitalWrite(outputPinOfCashRegister, LOW);
          delay(outputSignalWidth);
        }
        Serial.println(" i zafiskalizowano");
      } else {
        for (int i = 0; i < transaction.creditCount; i++) {
          digitalWrite(outputPinOfCarWashReceiverCH2, HIGH);
          delay(outputSignalWidth);
          digitalWrite(outputPinOfCarWashReceiverCH2, LOW);
          delay(outputSignalWidth);
        }
        Serial.println(" i niezafiskalizowano");
      }
    } else {
      delay(1000);
    }
  }
}

bool startFilesystem() {
  if (LittleFS.begin()) {
    File root = LittleFS.open("/", "r");
    File file = root.openNextFile();
    while (file) {
      Serial.printf("FS File: %s, size: %d\n", file.name(), file.size());
      file = root.openNextFile();
    }
    return true;
  } else {
    Serial.println("ERROR on mounting filesystem. It will be formmatted!");
    LittleFS.format();
    ESP.restart();
  }
  return false;
}

void getFsInfo(fsInfo_t* fsInfo) {
  fsInfo->totalBytes = LittleFS.totalBytes();
  fsInfo->usedBytes = LittleFS.usedBytes();
  strcpy(fsInfo->fsName, "LittleFS");
}

void handleTaxingRequest(AsyncWebServerRequest* request) {
  static int value = 0;
  String reply = "Taxing is ";
  // http://xxx.xxx.xxx.xxx/taxing?val=1
  if (request->hasParam("val")) {
    value = request->arg("val").toInt();
    if (value) {
      isTaxingEnabled = true;
      reply += "set to ON";
    } else {
      isTaxingEnabled = false;
      reply += "set to OFF";
    }
  } else {
    reply += isTaxingEnabled ? "ON" : "OFF";
  }
  request->send(200, "text/plain", reply);
}