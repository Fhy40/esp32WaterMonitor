#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_GIGA)
#include <WiFi.h>
#elif __has_include(<WiFiNINA.h>) || defined(ARDUINO_NANO_RP2040_CONNECT)
#include <WiFiNINA.h>
#elif __has_include(<WiFi101.h>)
#include <WiFi101.h>
#elif __has_include(<WiFiS3.h>) || defined(ARDUINO_UNOWIFIR4)
#include <WiFiS3.h>
#elif __has_include(<WiFiC3.h>) || defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif __has_include(<WiFi.h>)
#include <WiFi.h>
#endif
#include <time.h>
#include <FirebaseClient.h>
#include <ArduinoJson.h>


#define WIFI_SSID "***********"
#define WIFI_PASSWORD "***********"
const int trigPin = 2;
const int echoPin = 4;
int full_height = 50;
int empty_height = 200;
int delay_time = 60000;
long interval = 60000;
int sample_time = 200;
int distance_sampled = -69;
float percentage_constant = 100.0/(empty_height - full_height);

// The API key can be obtained from Firebase console > Project Overview > Project settings.
#define API_KEY "*********************************"

// User Email and password that already registerd or added in your project.
#define USER_EMAIL "***********@gmail.com"
#define USER_PASSWORD "***********"
#define DATABASE_URL "URL"
// Your collection here
String documentPath = "collection/document/collection";
#define FIREBASE_PROJECT_ID "waterlevel-***********"

void authHandler();

void printResult(AsyncResult &aResult);

void printError(int code, const String &msg);

String getTimestampString(uint64_t sec, uint32_t nano);
String payload;
DefaultNetwork network; // initilize with boolean parameter to enable/disable network reconnection

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);

FirebaseApp app;

#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFiClientSecure.h>
WiFiClientSecure ssl_client;
#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_GIGA) || defined(ARDUINO_PORTENTA_C33) || defined(ARDUINO_NANO_RP2040_CONNECT)
#include <WiFiSSLClient.h>
WiFiSSLClient ssl_client;
#endif

using AsyncClient = AsyncClientClass;

AsyncClient aClient(ssl_client, getNetwork(network));

Firestore::Documents Docs;

AsyncResult aResult_no_callback;

bool taskCompleted = false;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
int cnt = 0;
int first_round = 1;
unsigned long previousMillis = 0;

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT_PULLUP);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
      Serial.print(".");
      delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  delay(5000);
  Serial.println("Initializing app...");
  
  #if defined(ESP32) || defined(ESP8266) || defined(PICO_RP2040)
    ssl_client.setInsecure();
  #if defined(ESP8266)
    ssl_client.setBufferSizes(4096, 1024);
  #endif
  #endif

  initializeApp(aClient, app, getAuth(user_auth), aResult_no_callback);

  authHandler();

  // Binding the FirebaseApp for authentication handler.
  // To unbind, use Docs.resetApp();
  app.getApp<Firestore::Documents>(Docs);

  // In case setting the external async result to the sync task (optional)
  // To unset, use unsetAsyncResult().
  aClient.setAsyncResult(aResult_no_callback);
}

void loop()
{   
  unsigned long currentMillis = millis();
  
  authHandler();
  
  Docs.loop();

  if (app.ready() && first_round > 0)
  {
    set_variables();
  }

  if (app.ready() && currentMillis - previousMillis >= delay_time)
  {
      time_t now = time(nullptr);
      Serial.println(ctime(&now));
      previousMillis = currentMillis;
      int got_percentage = get_percentage();
      
      
      Values::IntegerValue distance(distance_sampled);
      Values::DoubleValue percentage(got_percentage);        
      Values::TimestampValue tsV(getTimestampString(now, 999999999));


      Document<Values::Value> doc("distance", Values::Value(distance));
      doc.add("percentage", Values::Value(percentage));
      doc.add("timestamp", Values::Value(tsV));
      

      Serial.println("Create document... ");
      log_i("Total heap: %u", ESP.getHeapSize());
      log_i("Free heap: %u", ESP.getFreeHeap());
      payload = Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), documentPath, DocumentMask(), doc);

      if (aClient.lastError().code() == 0)
          Serial.println(payload);
      else
          printError(aClient.lastError().code(), aClient.lastError().message());
      
      cnt = cnt + 1;
      if (cnt > 60)
      {
        Serial.println("RE-SYNC TIME");
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        set_variables();
        cnt = 0;
      }
    }
    
}

void authHandler()
{
    // Blocking authentication handler with timeout
    unsigned long ms = millis();
    while (app.isInitialized() && !app.ready() && millis() - ms < 120 * 1000)
    {
        // The JWT token processor required for ServiceAuth and CustomAuth authentications.
        // JWT is a static object of JWTClass and it's not thread safe.
        // In multi-threaded operations (multi-FirebaseApp), you have to define JWTClass for each FirebaseApp,
        // and set it to the FirebaseApp via FirebaseApp::setJWTProcessor(<JWTClass>), before calling initializeApp.
        JWT.loop(app.getAuth());
        Serial.println("AUTH HANDLER IS RUNNING");
        printResult(aResult_no_callback);
    }
}

void printResult(AsyncResult &aResult)
{
    if (aResult.isEvent())
    {
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.appEvent().message().c_str(), aResult.appEvent().code());
    }

    if (aResult.isDebug())
    {
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }

    if (aResult.isError())
    {
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }

    if (aResult.available())
    {
        Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
    }
}

String getTimestampString(uint64_t sec, uint32_t nano)
{
    if (sec > 0x3afff4417f)
        sec = 0x3afff4417f;

    if (nano > 0x3b9ac9ff)
        nano = 0x3b9ac9ff;

    time_t now;
    struct tm ts;
    char buf[80];
    now = sec;
    ts = *localtime(&now);

    String format = "%Y-%m-%dT%H:%M:%S";

    if (nano > 0)
    {
        String fraction = String(double(nano) / 1000000000.0f, 9);
        fraction.remove(0, 1);
        format += fraction;
    }
    format += "Z";

    strftime(buf, sizeof(buf), format.c_str(), &ts);
    return buf;
}

int average_distance(){
  int sum_distance = 0;
  for (int i = 0; i <= 5; i++) {
    sum_distance = sum_distance + get_distance();    
    delay(sample_time);
  }
  
  int average_distance = sum_distance / 5;
  return average_distance;
}

int get_distance(){
  digitalWrite(trigPin, LOW); // Set the trigger pin to low for 2uS 
  delayMicroseconds(2);   
  digitalWrite(trigPin, HIGH); // Send a 10uS high to trigger ranging 
  delayMicroseconds(20);   
  digitalWrite(trigPin, LOW); // Send pin low again 
  int distance = pulseIn(echoPin, HIGH,26000); // Read in times pulse
  distance= distance/58; //Convert the pulse duration to distance
                         //You can add other math functions to calibrate it well
  return distance;   
}

int get_percentage(){
  distance_sampled = average_distance();
  percentage_constant = 100.0/(empty_height - full_height);
  int loss_percentage = (distance_sampled - full_height) * percentage_constant;
  int  current_percentage = 100 - loss_percentage;
  return current_percentage;
}

void set_variables() {
  String variabledocumentPath = "variables/default";
      Serial.println("Get variables... ");
      String payload2 = Docs.get(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), variabledocumentPath, GetDocumentOptions());
      if (aClient.lastError().code() == 0)
        {
            Serial.println(payload2);

            JsonDocument doc;
            deserializeJson(doc, payload2);
            String fields = doc["fields"];
            Serial.println(fields);
            int timer = doc["fields"]["timer"]["integerValue"];
            int high_point = doc["fields"]["high_point"]["integerValue"];
            int low_point = doc["fields"]["low_point"]["integerValue"];
            int rv_sample_time = doc["fields"]["sample_time"]["integerValue"];
            Serial.println(timer);
            Serial.println(high_point);
            Serial.println(low_point);
            Serial.println(rv_sample_time);
            full_height = high_point;
            empty_height = low_point;
            delay_time = timer;
            sample_time = rv_sample_time;
            first_round  = 0;
        }
}

void printError(int code, const String &msg)
{
    Firebase.printf("Error, msg: %s, code: %d\n", msg.c_str(), code);
}



