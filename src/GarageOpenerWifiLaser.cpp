/*
 * Garage door opener/monitor firmware for a bare ESP-12E module.
 *
 * A VL53L0X time-of-flight distance sensor measures the gap between a
 * fixed point (e.g. the back wall) and the car bumper to infer whether
 * the garage door is open or closed. Status is published over MQTT and
 * shown on a small authenticated web UI (/gui), which also has a button
 * to trigger the door relay; the relay can also be triggered via an MQTT
 * command. WiFi credentials are provisioned with WiFiManager, and
 * firmware can be updated either over serial or over WiFi (ArduinoOTA).
 *
 * See README.md for hardware details and build/flash instructions.
 */

#include <Arduino.h>

// WIFIManager
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <WiFiClient.h>

// For MQTT
#include <PubSubClient.h>

#include <Wire.h>
#include <VL53L0X.h>


// For the distance sensor
VL53L0X sensor;

/////////////////////////////////////////////////
// MQTT stuff

#define MQTT_KEEPALIVE 10

// MQTT topics
//#define STATUS_TOPIC "/myhouse/garage/GarageOpener_v3/status"
//#define COMMAND_TOPIC "/myhouse/garage/GarageOpener_v3/commands"

#define STATUS_TOPIC "/myhouse/garage/opener/status"
#define COMMAND_TOPIC "/myhouse/garage/opener/commands"

const char* mqtt_server = "192.168.1.4";

#define MQTT_CLIENT_ID "GarageOpener_v3"

// Following two used for MQTT/pubsubclient
WiFiClient espClient;
PubSubClient client(espClient);

// This is set to some distance greater than dist to the door when it's open
// It should not be so high that something under the door could be detected closer (like a car)
// In millimeters
#define DOOR_OPEN_MIN_DIST 400

// Time in ms to wait between distance checks
#define DIST_CHECK_INTERVAL 1000

// Time in ms to periodically resend door/distance status messages (even if nothing changed)
// This time should be greater/or equal to DIST_CHECK_INTERVAL
#define STATUS_RESEND_INTERVAL 60000

// Distance change in percent before it's considered a change
#define DIST_CHANGE_PERCENT 10

// The VL53L0X reports a distance around this value (well beyond any real
// distance in this application) when it can't find a valid target. Treat
// readings at or above this as invalid rather than as a real "far" distance.
#define SENSOR_OUT_OF_RANGE_MM 8000

// Number of consecutive readings (agreeing with each other) required before
// the reported door open/closed state is changed. Filters out single errant
// readings without needing to average/median-filter every sample.
#define DOOR_STATE_CONFIRM_COUNT 3


/////////////////////////////////////////////////
// Laser distance sensor stuff

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE

// Pin 5 goes to SLC
// Pin 4 goes to SDA

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

#define GREEN_LIGHT_PIN 16
#define RED_LIGHT_PIN 14

/////////////////////////////////////////////////
// Other distance checking related

// When at 16.5 inches from front wall looks like there is right at 10 inches clearance from back bumper to garage door
// Will call 16.5 a good middle of the road for the acceptable range
// Will use this information along with a little safety margin to set min and max distances

// Max distance from the wall (min from the garage door)
// This should give about 4.5" from back bumper to garage door
#define GOOD_DIST_MAX 22*25.4

// Min distance from the wall (max from the garage door)
#define GOOD_DIST_MIN 11*25.4

// Am trying 23 inches as the median threshold
#define GOOD_CENTER 16.5*25.4

// How frequently to poll for distance
// This could probably go WAY down fo rthe garage opener
#define POLL_FREQUENCY_MS DIST_CHECK_INTERVAL

/////////////////////////////////////////////////
// Web server

ESP8266WebServer server(80);

/////////////////////////////////////////////////

// WIFIManager
WiFiManager wifiManager;

// The server username/password
const String webUser = "admin";
const String webPass = "admin";

String validLoginPwCookieValue = String("login=") + webUser + ":" + webPass;

//const char* ssid = "homern";
//const char* password = "digitalheerocismystandardpassword";

/////////////////////////////////////////////////
// Function prototypes

//void setupWIfi();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void checkDistance();
bool is_authentified();
void handleStatus();

/////////////////////////////////////////////////
// Pin assignments

/////////////////////////////////////////////////
// Global variables
#define BUTTON 13

// Power / good / error status
#define RED_LED 16

// Status of door (up or down)
#define BLUE_LED 14

// Lit when door opener is being triggered
#define GREEN_LED 12


boolean rebooted = false;
boolean needToPublishWifiConnect = false;
boolean needToPublishMQTTConnect = false;

long lastDistanceCheck=0;
long lastStatusSend=0;
long lastDistance=0;

// The last door state that has been confirmed (via DOOR_STATE_CONFIRM_COUNT
// consecutive agreeing readings) and is safe to report/act on.
bool confirmedDoorOpen = false;
bool pendingDoorOpen = false;
int pendingDoorOpenCount = 0;

/////////////////////////////////////////////////

class DistanceInfo {
  public:
    long distance=0;
    bool doorOpen=false;

    // Returns false if the reading should be discarded (sensor timeout or
    // an out-of-range "no target" reading), in which case distance/doorOpen
    // are not meaningful and should not be used.
    bool calcDistance() {
        int dist = sensor.readRangeSingleMillimeters();
        Serial.print("Distance: ");
        Serial.println(dist);

        bool timeout = sensor.timeoutOccurred();
        if( timeout ) {
            Serial.print(" TIMEOUT");
        }

        bool outOfRange = (dist >= SENSOR_OUT_OF_RANGE_MM);
        if( outOfRange ) {
            Serial.print(" OUT OF RANGE");
        }

        bool valid = !timeout && !outOfRange;

        if( valid ) {
            distance = dist;
        }
        else {
            distance = -1;
        }

        doorOpen = (distance<DOOR_OPEN_MIN_DIST);

        return valid;
    }
};

// Allows something to print up to 15 characters to it ... intended for an ip address
class IpPrinter : public Print {
  private:
    char ipv4[16];
    short count=0;
    
  public:
    IpPrinter() { memset(ipv4, '\0', sizeof(ipv4)); }
    
    virtual size_t write(uint8_t v) {
//      Serial.print("called write: "); Serial.println(v);
      if( count < (sizeof(ipv4)-1) ) {
        ipv4[count++] = v;
      }
      return 3;
    }
    
    virtual size_t write(const uint8_t *buffer, size_t size) {
      if( (count+size) < (sizeof(ipv4)-1) ) {
        strcpy(ipv4+count, (char *)buffer);
        count += size;
      }
      return size;
    }

    char * getIpv4() { return ipv4; };
};

/////////////////////////////////////////////////
// Handlers for web calls

// Does not require authentication (TODO change to require)
void detectDistance() {
  DistanceInfo distanceInfo;

  distanceInfo.calcDistance();
  
  String content = "<html><head>";
  content += "</head><body>";
  content += "<span style='background: grey; display: inline-block; padding: 25px'>";
  content += "Distance: "+String(distanceInfo.distance);
  content += "</span>";

  content += "</body>";
  content += "</html>";
  
  server.send(200, "text/html", content);
}

/*
 * Sets the reboot flag to false
 */
void handleResetRebootFlag() {
    if (!is_authentified()) {
      // Redirect to login url
      String header = "HTTP/1.1 301 OK\r\n";
      header += "Location: /login\r\n";
      header += "Cache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
      return;
  }
  
  rebooted = false;
  handleStatus();
}

// TODO Should we have an endpoint to cause a reboot?

void pushButton() {
  digitalWrite(BUTTON, HIGH); // Relay
  digitalWrite(GREEN_LED, HIGH); // Led that relay is on
  
  delay(1000); // hope this doesn't cause issues ... This is a busy wait
  
  digitalWrite(BUTTON, LOW); // relay
  digitalWrite(GREEN_LED, LOW); // Led that relay is off
}

/*
 * Handles the request to open/close the garge door
 * This looks like a garage door button push ... thus the name
 */
void handleButton() {
  
  if (!is_authentified()) {
      // Redirect to login url
      String header = "HTTP/1.1 301 OK\r\n";
      header += "Location: /login\r\n";
      header += "Cache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
      return;
  }

  pushButton();

  handleStatus();
}

/*
 * The main page for the garage door gui
 */
void handleGui() {
  if (!is_authentified()) {
      // Redirect to login url
      String header = "HTTP/1.1 301 OK\r\n";
      header += "Location: /login\r\n";
      header += "Cache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
      return;
  }

  // Should contain display of garage door status open or closed
  // Should show button to push the garage door button
  // Should constantly poll for the status of the open/close switches

  String content = R"HTML(<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>Garage Door</title>
<style>
  * { box-sizing: border-box; }
  body { margin:0; font-family:-apple-system,'Segoe UI',Roboto,Arial,sans-serif; background:#f2f2f2; display:flex; justify-content:center; align-items:center; min-height:100vh; padding:16px; }
  .card { background:#fff; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,0.15); padding:24px; width:100%; max-width:380px; text-align:center; }
  h1 { font-size:1.3rem; margin:0 0 16px; color:#333; }
  #doorStatus { display:inline-block; padding:10px 20px; border-radius:20px; font-size:1.1rem; font-weight:bold; color:#fff; background:#999; }
  #doorStatus.open { background:#e74c3c; }
  #doorStatus.closed { background:#2ecc71; }
  #distance { display:block; margin-top:12px; color:#666; font-size:0.95rem; min-height:1.2em; }
  #openClose { margin-top:24px; width:100%; padding:16px; font-size:1.1rem; border:none; border-radius:8px; background:#2d7ff9; color:#fff; cursor:pointer; }
  #openClose:active { background:#1a5fd0; }
  #elapsed { display:block; margin-top:16px; color:#bbb; font-size:0.75rem; min-height:1em; }
</style>
<script type='text/javascript'>
var refreshRate=5;
var isRefreshing=false;

function refresh(){
  setTimeout(refresh,1000*refreshRate);
  if(!isRefreshing){checkStatus();}
}

function callServer(url,params,callback){
  var xmlhttp=window.XMLHttpRequest?new XMLHttpRequest():new ActiveXObject('Microsoft.XMLHTTP');
  xmlhttp.onreadystatechange=function(){callback(xmlhttp);};
  xmlhttp.open('POST',url,true);
  xmlhttp.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  xmlhttp.send(params);
}

function pushButton(){
  callServer('/button','',function(xmlhttp){
    if(xmlhttp.readyState==4&&xmlhttp.status==200){
      updateStatus(eval('('+xmlhttp.responseText+')'));
    }
  });
}

function checkStatus(){
  isRefreshing=true;
  var startTime=new Date();
  callServer('/status','',function(xmlhttp){
    if(xmlhttp.readyState==4&&xmlhttp.status==200){
      updateStatus(eval('('+xmlhttp.responseText+')'));
      isRefreshing=false;
      var elapsedEl=document.getElementById('elapsed');
      if(elapsedEl){elapsedEl.textContent='millis: '+(new Date()-startTime);}
    }
  });
}

function updateStatus(result){
  var statusEl=document.getElementById('doorStatus');
  var distanceEl=document.getElementById('distance');
  if(result.response=='success'){
    statusEl.textContent=result.doorStatus;
    statusEl.className=(result.doorStatus=='Door open')?'open':'closed';
    if(distanceEl){distanceEl.textContent='Distance: '+result.distance+' mm';}
  } else {
    statusEl.textContent='Error getting status';
    statusEl.className='';
  }
}

refresh();
</script>
</head><body>
<div class='card'>
<h1>Garage Door</h1>
<span id='doorStatus'>Loading...</span>
<span id='distance'></span>
<input type='button' id='openClose' value='Open / Close Door' onclick='pushButton()'/>
<span id='elapsed'></span>
</div>
</body></html>
)HTML";

  server.send(200, "text/html", content);
}


/*
 * This returns a json message showing the switch stati and the reboot flag
 */
void handleStatus() {
  Serial.println("Request for status");

  if (!is_authentified()) {
    String header = "HTTP/1.1 401 Access Denied\r\nCache-Control: no-cache\r\n\r\n";
    server.sendContent(header);
    return;
  }
  
  DistanceInfo distanceInfo;
  distanceInfo.calcDistance();

  // Report the confirmed/debounced door state (updated by checkDistance()),
  // not this single instantaneous reading, so a momentary errant reading
  // doesn't show up here either.
  String doorStatus = String( (confirmedDoorOpen?"Door open":"Door closed") );

  String content = String( "{ response: 'success', doorStatus: '") + doorStatus + 
    "', rebooted: " + (rebooted?"true":"false") + 
    //", doorOpen: " + (distanceInfo.doorOpen?"true":"false") +
    ", distance: " + (distanceInfo.distance) +
    "}";
  
  server.send(200, "application/json", content);
}

/*
 * 
 */
bool is_authentified(){
  bool authenticated = false;
  
  Serial.println("Enter is_authentified");
  
  if (server.hasHeader("Cookie")){   
  
    Serial.print("Found cookie: ");
    String cookie = server.header("Cookie");
    Serial.println(cookie);
    
    if (cookie.indexOf( validLoginPwCookieValue ) != -1) {
      authenticated = true;
      Serial.println("Authentification by cookie Successful");
    }
    
  }
  
  // Try username/password auth next
  if( !authenticated ) {
    if( server.hasArg("username") && server.hasArg("password") ) {
      String username = server.arg("username");
      String password = server.arg("password");
      Serial.println(username);
      Serial.println(password);

      if( webUser.equals(username) && webPass.equals(password) ) {
        authenticated = true;
      }
      else
        Serial.println("Invalid username/password");
    }
  }

  if( !authenticated )
    Serial.println("Authentification Failed");

  return authenticated;  
}

//login page
void handleLogin(){
  String msg;

  if (!is_authentified()) {
    if (server.hasArg("username") && server.hasArg("password")) {
      msg = "<div class='error'>Wrong username or password. Try again.</div>";
    }
  }
  else {
    // Redirect to garage gui url and set a cookie
      String header = "HTTP/1.1 301 OK\r\n";
      header += "Set-Cookie: " + validLoginPwCookieValue + "\r\n";
      header += "Location: /gui\r\n";
      header += "Cache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
      Serial.println("Log in Successful");
      return;
  }

  String content = R"HTML(<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>Garage Opener Login</title>
<style>
  * { box-sizing: border-box; }
  body { margin:0; font-family:-apple-system,'Segoe UI',Roboto,Arial,sans-serif; background:#f2f2f2; display:flex; justify-content:center; align-items:center; min-height:100vh; padding:16px; }
  .card { background:#fff; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,0.15); padding:24px; width:100%; max-width:320px; }
  h1 { font-size:1.2rem; margin:0 0 16px; text-align:center; color:#333; }
  label { display:block; margin-bottom:4px; font-size:0.9rem; color:#666; }
  input[type=text],input[type=password] { width:100%; padding:10px; margin-bottom:16px; border:1px solid #ccc; border-radius:6px; font-size:1rem; }
  input[type=submit] { width:100%; padding:12px; border:none; border-radius:8px; background:#2d7ff9; color:#fff; font-size:1rem; cursor:pointer; }
  .error { color:#e74c3c; font-size:0.9rem; text-align:center; margin-top:12px; }
</style>
</head><body>
<div class='card'>
<h1>Garage Opener Login</h1>
<form action='/login' method='POST'>
<label>Username</label>
<input type='text' name='username'/>
<label>Password</label>
<input type='password' name='password'/>
<input type='submit' name='login' value='Login'/>
</form>
)HTML" + msg + R"HTML(
</div>
</body></html>
)HTML";

  server.send(200, "text/html", content);
}

// root page can be accessed only if authentification is ok
void handleRoot(){
  Serial.println("Enter handleRoot");
  String header;
  if (!is_authentified()){
    String header = "HTTP/1.1 301 OK\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    server.sendContent(header);
    return;
  }
  
  header = "HTTP/1.1 301 OK\r\nLocation: /gui\r\nCache-Control: no-cache\r\n\r\n";
  server.sendContent(header);
}

// no need authentification
void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

/////////////////////////////////////////////////


void setup()
{
  Serial.begin(115200);
  Wire.begin(4, 5); // Per https://blog.squix.org/2017/04/esp8266-peripherie-cjmcu-530-tof-distance-sensor.html

  Serial.println("Garage opener starting up");
  
  // Relay output (button)
  pinMode(BUTTON, OUTPUT);
  digitalWrite(BUTTON, LOW);

  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);

  
  // TODO Pin setup for status LEDs
//  pinMode(GREEN_LIGHT_PIN, OUTPUT);
//  pinMode(RED_LIGHT_PIN, OUTPUT);
//  
//  digitalWrite(GREEN_LIGHT_PIN, LOW);
//  digitalWrite(RED_LIGHT_PIN, LOW);
 
  // Start up the laser distance sensor
  sensor.init();
  sensor.setTimeout(500);

  
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  // WIFIManager
  if (!wifiManager.autoConnect("GarageOpener", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  Serial.println("Connected to wifi");
  WiFi.printDiag(Serial);

  // Set up the web server
  server.on("/", handleRoot);
  server.on("/login", handleLogin);
  
  /*server.on("/inline", []() {// TOOD Interesting ... review C++ closures again
    server.send(200, "text/plain", "this works without need of authentification");
  });*/

  server.on("/gui", handleGui);

  server.on("/status", handleStatus);
  
  server.on("/button", handleButton);

  server.on("/resetrebootflag", handleResetRebootFlag);

  server.on("/dist", detectDistance);
  
  server.onNotFound(handleNotFound);

  // TODO Figure what this is all about
  //here the list of headers to be recorded
  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  
  //ask server to track these headers
  server.collectHeaders(headerkeys, headerkeyssize );
  
  server.begin();
  Serial.println("HTTP server started");
  
  // TODO Re-enable mqtt setup
  // MQTT setup
  client.setServer(mqtt_server, 1883);
  client.setCallback(callbackMQTT);

  needToPublishWifiConnect = true;

  //////////////////////////////////////////////
  // OTA stuff
  
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("garage-opener_v3-esp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  
  //////////////////////////////////////////////
  digitalWrite(RED_LED, HIGH); // Signal that all is well
  
}

// Last time we polled the distance sensor
long lastPoll=0;

void loop()
{
  // Find equivalent wifimanager way of doing things
  
//  if (WiFi.status() != WL_CONNECTED) {
//    needToPublishWifiConnect = true;
//    Serial.println("Reconnecting");
//    setupWifi(); // TODO Need to do equivalent thing with WifiManager
//    return; // the guy i copied this from said this was was necessary to avoid resets
//  }

  // Handle http stuff
  server.handleClient();
  
  // Handle reconnecting to MQTT if necessary
  if (!client.connected()) {
    needToPublishMQTTConnect = true;
    reconnectMQTT();
  }
  else {
    if( needToPublishWifiConnect ) {
      Serial.println("Publishing wifi connect");
      // Once connected, publish an announcement...
      client.publish( STATUS_TOPIC , "opener:wifi connected");
      IpPrinter ipPrinter;
      WiFi.localIP().printTo(ipPrinter);
      String ip = String("opener:ip:") + String( ipPrinter.getIpv4() );
      client.publish( STATUS_TOPIC, ip.c_str());
      needToPublishWifiConnect = false;
    }

    if( needToPublishMQTTConnect ) {
      Serial.println("Publishing mqtt connect");
      // Once connected, publish an announcement...
      client.publish( STATUS_TOPIC , "opener:mqtt connected");
      needToPublishMQTTConnect = false;
    }
  }

  // MQTT trigger to do it's work
  client.loop();
  
  checkDistance();
  
//  // MQTT trigger to do it's work
  client.loop();

  // OTA
  ArduinoOTA.handle();
  
}

//void setupWifi()
//{
//  WiFi.begin(ssid, password);
//  delay(10);
//  
//  // Wait for connection
//  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print(".");
//    delay(500);
//  }
//
//  Serial.println("\nWifi reconnected");
//  
//  Serial.println("");
//  Serial.print("Connected to ");
//  Serial.println(ssid);
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());
//
//  Serial.print("Connected to ");
//  Serial.println(ssid);
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP()); 
//}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if( strcmp(topic, COMMAND_TOPIC) == 0 ) {
    // Check if it's a push button command
    // button command format: "doorbutton"
    if( length==10 && strncmp( "doorbutton",  (char*)payload, 10)==0 ) {
      pushButton();
    }
  }

  // Might listen for door open/close commands in the future

}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
      
      // ... and resubscribe
      client.subscribe( COMMAND_TOPIC );
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void checkDistance() {
  if( (millis() - lastDistanceCheck) >= DIST_CHECK_INTERVAL ) {
    DistanceInfo distanceInfo;

    if( distanceInfo.calcDistance() ) {
      bool sendMsg = false;

      // Require a few consecutive readings to agree before changing the
      // confirmed door state, so a single errant reading can't flip it.
      if( distanceInfo.doorOpen != confirmedDoorOpen ) {
        if( distanceInfo.doorOpen == pendingDoorOpen ) {
          pendingDoorOpenCount++;
        }
        else {
          pendingDoorOpen = distanceInfo.doorOpen;
          pendingDoorOpenCount = 1;
        }

        if( pendingDoorOpenCount >= DOOR_STATE_CONFIRM_COUNT ) {
          confirmedDoorOpen = pendingDoorOpen;
          pendingDoorOpenCount = 0;
        }
      }
      else {
        pendingDoorOpenCount = 0;
      }

      digitalWrite(BLUE_LED, confirmedDoorOpen ? HIGH : LOW);

      // Send status messages if distance has changed by the defined percent
      if( lastDistance==0 || abs( ((float)(distanceInfo.distance-lastDistance))/lastDistance )*100 >= DIST_CHANGE_PERCENT ) {
        sendMsg = true;
        lastDistance = distanceInfo.distance;
      }

      // Periodically send messages even if percent distance has not changed enough
      else if( (millis() - lastStatusSend) >= STATUS_RESEND_INTERVAL ) {
        sendMsg = true;
      }

      // If we are connected to the mqtt server publish a message
      if( sendMsg && client.connected() ) {
          String msg = String("floordistance:") + String(distanceInfo.distance);
          client.publish( STATUS_TOPIC , msg.c_str());

          String msg2 = String("doorstatus:") + String( confirmedDoorOpen ? "Door open" : "Door closed" );
          client.publish( STATUS_TOPIC , msg2.c_str());

          lastStatusSend = millis();
      }
    }

    lastDistanceCheck = millis();
  }
}


