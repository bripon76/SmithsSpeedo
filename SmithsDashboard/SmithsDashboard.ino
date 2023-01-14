// Import required libraries
#include "WiFi.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "FS.h"
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid     = "SmithsDash";
const char* password = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,void *arg, uint8_t *data, size_t len) {
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open("/log.txt", FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}


void setup(){

// Serial port for debugging purposes
Serial.begin(115200);
initWebSocket();



if (!SPIFFS.begin(true)) {
Serial.println("An Error has occurred while mounting SPIFFS");
return;
}

File root = SPIFFS.open("/");

File filelist = root.openNextFile();

while(filelist){

Serial.print("FILE: ");
Serial.println(filelist.name());
filelist = root.openNextFile();
}




// Connect to Wi-Fi network with SSID and password
Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
WiFi.softAP(ssid, password);

IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);

// Print ESP32 Local IP Address
Serial.println(WiFi.localIP());

// Route for root / web page
server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
});

//Load PNG
server.on("/logo.png", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/logo.png", "image/png");
  });

// Route for Home / web page
server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
});

// Route for download / web page
server.on("/download.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/download.html");
});

// Route for about / web page
server.on("/about.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/about.html");
});
  
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

 // Download Log
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/log.txt", String(), true);
  });

// Remove Log
  server.on("/remove", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS.remove("/log.txt"));
    SPIFFS.remove("/log.txt");
    appendFile(SPIFFS, "/log.txt", "---------------------------------\r\n");
    appendFile(SPIFFS, "/log.txt", "--  SmithsDash v.0.1  LogFile  --\r\n");
    appendFile(SPIFFS, "/log.txt", "--   by Ennar1991 & bripon76   --\r\n");
    appendFile(SPIFFS, "/log.txt", "---------------------------------\r\n");
    appendFile(SPIFFS, "/log.txt", "");
    request->send(SPIFFS, "/download.html");
  });

  // Start server
  server.begin();
}

char serialBuffer[100];
char serialBufferPosition=0;
void loop(){

ws.cleanupClients();

  if (Serial.available()) {
      serialBuffer[serialBufferPosition]=Serial.read();
      if (serialBuffer[serialBufferPosition]=='\n') {
        //do some kind of action
          ws.textAll(String(serialBuffer)); //this is what we send to the browser
          appendFile(SPIFFS, "/log.txt", (serialBuffer));
        serialBufferPosition=0;
        memset(serialBuffer, 0, sizeof(serialBuffer));
      } else{
        serialBufferPosition++;
      }
  }


  } 
