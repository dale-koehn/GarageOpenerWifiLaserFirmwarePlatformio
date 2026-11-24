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
// Must be defined before including PubSubClient.h to actually take effect.
#define MQTT_KEEPALIVE 10
#include <PubSubClient.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>


// For the distance sensor
VL53L0X sensor;

/////////////////////////////////////////////////
// MQTT stuff

#define STATUS_TOPIC "/myhouse/garage/opener/status"
#define COMMAND_TOPIC "/myhouse/garage/opener/commands"

// Home Assistant topics. State and availability are published retained so
// Home Assistant has a value immediately after it (re)starts.
#define DOOR_TOPIC "/myhouse/garage/opener/door"           // "open" or "closed"
#define DISTANCE_TOPIC "/myhouse/garage/opener/distance"   // raw reading in mm
#define AVAILABILITY_TOPIC "/myhouse/garage/opener/availability" // "online"/"offline"

// MQTT discovery. The unique ids and device identifier below must never
// change, or Home Assistant will treat the unit as a new device.
#define DISCOVERY_PREFIX "homeassistant"
#define HA_DEVICE_ID "garage_opener_v3"

// Initial/default value for config.mqttServer the very first time the
// device boots; editable afterward via the /config page.
#define DEFAULT_MQTT_SERVER "192.168.1.4"

#define MQTT_CLIENT_ID "GarageOpener_v3"

// Following two used for MQTT/pubsubclient
WiFiClient espClient;
PubSubClient client(espClient);

// The six values below control device behavior and are editable at runtime
// via the /config web page; they're stored in flash (see Config/EEPROM
// below) and these #defines only supply the initial/default values the very
// first time the device boots (or after a flash layout change).

// This is set to some distance greater than dist to the door when it's open
// It should not be so high that something under the door could be detected closer (like a car)
// In millimeters
#define DEFAULT_DOOR_OPEN_MIN_DIST 500

// Time in ms to wait between distance checks
#define DEFAULT_DIST_CHECK_INTERVAL 1000

// Time in ms to periodically resend door/distance status messages (even if nothing changed)
// This time should be greater/or equal to DEFAULT_DIST_CHECK_INTERVAL
#define DEFAULT_STATUS_RESEND_INTERVAL 60000

// Distance change in percent before it's considered a change
#define DEFAULT_DIST_CHANGE_PERCENT 10

// The VL53L0X reports a distance around this value (well beyond any real
// distance in this application) when it can't find a valid target. Treat
// readings at or above this as invalid rather than as a real "far" distance.
#define DEFAULT_SENSOR_OUT_OF_RANGE_MM 8000

// Number of consecutive readings (agreeing with each other) required before
// the reported door open/closed state is changed. Filters out single errant
// readings without needing to average/median-filter every sample.
#define DEFAULT_DOOR_STATE_CONFIRM_COUNT 3

/////////////////////////////////////////////////
// Runtime-configurable settings, stored in flash (EEPROM)

// Bump this if the Config struct layout changes, so old/incompatible data
// in flash gets replaced with defaults instead of being misread.
#define CONFIG_MAGIC 0xC0FFEE03

struct Config {
  uint32_t magic;
  long doorOpenMinDist;
  unsigned long distCheckInterval;
  unsigned long statusResendInterval;
  int distChangePercent;
  long sensorOutOfRangeMm;
  int doorStateConfirmCount;
  char mqttServer[64];
};

Config config;

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

void loadConfig() {
  EEPROM.begin(sizeof(Config));
  EEPROM.get(0, config);

  if( config.magic != CONFIG_MAGIC ) {
    Serial.println("No valid config found in flash, writing defaults");
    config.magic = CONFIG_MAGIC;
    config.doorOpenMinDist = DEFAULT_DOOR_OPEN_MIN_DIST;
    config.distCheckInterval = DEFAULT_DIST_CHECK_INTERVAL;
    config.statusResendInterval = DEFAULT_STATUS_RESEND_INTERVAL;
    config.distChangePercent = DEFAULT_DIST_CHANGE_PERCENT;
    config.sensorOutOfRangeMm = DEFAULT_SENSOR_OUT_OF_RANGE_MM;
    config.doorStateConfirmCount = DEFAULT_DOOR_STATE_CONFIRM_COUNT;
    snprintf(config.mqttServer, sizeof(config.mqttServer), "%s", DEFAULT_MQTT_SERVER);
    saveConfig();
  }
}


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

/////////////////////////////////////////////////
// Function prototypes

void callbackMQTT(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void checkDistance();
void publishDiscovery();
void publishHaState();
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


boolean rebooted = true;
boolean needToPublishWifiConnect = false;
boolean needToPublishMQTTConnect = false;

// Set whenever the MQTT connection is (re)established so the current state is
// republished (retained) right away.
boolean needToPublishHaState = true;

// Most recent values, kept so they can be republished after a reconnect.
long lastRawDistance = 0;
bool haStateValid = false;

long lastDistanceCheck=0;
long lastStatusSend=0;
long lastDistance=0;

// The last door state that has been confirmed (via config.doorStateConfirmCount
// consecutive agreeing readings) and is safe to report/act on.
bool confirmedDoorOpen = false;
bool pendingDoorOpen = false;
int pendingDoorOpenCount = 0;

/////////////////////////////////////////////////

class DistanceInfo {
  public:
    long distance=0;
    bool doorOpen=false;

    // Returns false if the reading should be discarded (sensor timeout,
    // meaning no distance could be read at all), in which case
    // distance/doorOpen are not meaningful and should not be used.
    bool calcDistance() {
        int dist = sensor.readRangeSingleMillimeters();
        Serial.print("Distance: ");
        Serial.println(dist);

        bool timeout = sensor.timeoutOccurred();
        if( timeout ) {
            Serial.print(" TIMEOUT");
            return false;
        }

        bool outOfRange = (dist >= config.sensorOutOfRangeMm);
        if( outOfRange ) {
            // No target detected within range. With the sensor aimed at
            // the floor under the door, this happens when the door is
            // down, so it's a legitimate "far" reading (door closed) -
            // not an error to discard, unlike a timeout.
            Serial.print(" OUT OF RANGE");
            distance = 10000;
        }
        else {
            distance = dist;
        }

        doorOpen = (distance<config.doorOpenMinDist);

        return true;
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

// CSS shared by every authenticated page's top navigation bar.
const char* NAV_BAR_CSS =
  ".navbar { display:flex; align-items:center; gap:16px; background:#222; padding:12px 20px; }"
  ".navbar a { color:#ccc; text-decoration:none; font-size:0.95rem; }"
  ".navbar a.active { color:#fff; font-weight:bold; }"
  ".navbar a.logout { margin-left:auto; color:#e74c3c; }"
  ".content { display:flex; justify-content:center; align-items:flex-start; padding:24px 16px; }";

// Renders the top nav bar shown on every authenticated page, highlighting
// whichever page (by id: "gui" or "config") is currently active.
String renderNavBar(const char* activePage) {
  String nav = "<nav class='navbar'>";
  nav += "<a href='/gui' class='";
  nav += (strcmp(activePage, "gui") == 0) ? "active" : "";
  nav += "'>Monitor</a>";
  nav += "<a href='/config' class='";
  nav += (strcmp(activePage, "config") == 0) ? "active" : "";
  nav += "'>Config</a>";
  nav += "<a href='/logout' class='logout'>Logout</a>";
  nav += "</nav>";
  return nav;
}

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
  body { margin:0; font-family:-apple-system,'Segoe UI',Roboto,Arial,sans-serif; background:#f2f2f2; min-height:100vh; }
  )HTML" + String(NAV_BAR_CSS) + R"HTML(
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
)HTML" + renderNavBar("gui") + R"HTML(
<div class='content'>
<div class='card'>
<h1>Garage Door</h1>
<span id='doorStatus'>Loading...</span>
<span id='distance'></span>
<input type='button' id='openClose' value='Open / Close Door' onclick='pushButton()'/>
<span id='elapsed'></span>
</div>
</div>
</body></html>
)HTML";

  server.send(200, "text/html", content);
}

/*
 * Settings page: lets the six behavior-tuning values (normally seeded from
 * the DEFAULT_* defines) be viewed and changed at runtime. Changes are
 * written to flash immediately so they survive a reboot.
 */
void handleConfig() {
  if (!is_authentified()) {
      // Redirect to login url
      String header = "HTTP/1.1 301 OK\r\n";
      header += "Location: /login\r\n";
      header += "Cache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
      return;
  }

  String message;
  if (server.method() == HTTP_POST) {
    snprintf(config.mqttServer, sizeof(config.mqttServer), "%s", server.arg("mqttServer").c_str());
    config.doorOpenMinDist = server.arg("doorOpenMinDist").toInt();
    config.distCheckInterval = server.arg("distCheckInterval").toInt();
    config.statusResendInterval = server.arg("statusResendInterval").toInt();
    config.distChangePercent = server.arg("distChangePercent").toInt();
    config.sensorOutOfRangeMm = server.arg("sensorOutOfRangeMm").toInt();
    config.doorStateConfirmCount = server.arg("doorStateConfirmCount").toInt();
    saveConfig();

    // Point the MQTT client at the (possibly new) server and drop any
    // existing connection so loop()'s reconnect logic picks up the change
    // right away instead of waiting for a reboot.
    client.setServer(config.mqttServer, 1883);
    if( client.connected() ) {
      client.disconnect();
    }

    message = "<div class='success'>Settings saved.</div>";
  }

  String content = R"HTML(<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>Garage Opener Config</title>
<style>
  * { box-sizing: border-box; }
  body { margin:0; font-family:-apple-system,'Segoe UI',Roboto,Arial,sans-serif; background:#f2f2f2; min-height:100vh; }
  )HTML" + String(NAV_BAR_CSS) + R"HTML(
  .card { background:#fff; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,0.15); padding:24px; width:100%; max-width:420px; }
  h1 { font-size:1.2rem; margin:0 0 16px; text-align:center; color:#333; }
  label { display:block; margin-top:14px; margin-bottom:4px; font-size:0.85rem; color:#666; }
  input[type=number],input[type=text] { width:100%; padding:8px; border:1px solid #ccc; border-radius:6px; font-size:1rem; }
  input[type=submit] { margin-top:20px; width:100%; padding:12px; border:none; border-radius:8px; background:#2d7ff9; color:#fff; font-size:1rem; cursor:pointer; }
  .success { color:#2ecc71; font-size:0.9rem; text-align:center; margin-top:12px; }
  .hint { font-size:0.75rem; color:#999; margin-top:2px; }
</style>
</head><body>
)HTML" + renderNavBar("config") + R"HTML(
<div class='content'>
<div class='card'>
<h1>Configuration</h1>
<form action='/config' method='POST'>
<label>MQTT server (IP or hostname)</label>
<input type='text' name='mqttServer' maxlength='63' value=')HTML" + String(config.mqttServer) + R"HTML('/>
<div class='hint'>Broker the device publishes status to and listens for commands on.</div>

<label>Door open threshold (mm)</label>
<input type='number' name='doorOpenMinDist' value=')HTML" + String(config.doorOpenMinDist) + R"HTML('/>
<div class='hint'>Distance below which the door is considered open.</div>

<label>Distance check interval (ms)</label>
<input type='number' name='distCheckInterval' value=')HTML" + String(config.distCheckInterval) + R"HTML('/>
<div class='hint'>How often the sensor is polled.</div>

<label>Status resend interval (ms)</label>
<input type='number' name='statusResendInterval' value=')HTML" + String(config.statusResendInterval) + R"HTML('/>
<div class='hint'>How often status is republished over MQTT even if unchanged.</div>

<label>Distance change threshold (%)</label>
<input type='number' name='distChangePercent' value=')HTML" + String(config.distChangePercent) + R"HTML('/>
<div class='hint'>Minimum distance change to trigger an MQTT update.</div>

<label>Sensor out-of-range threshold (mm)</label>
<input type='number' name='sensorOutOfRangeMm' value=')HTML" + String(config.sensorOutOfRangeMm) + R"HTML('/>
<div class='hint'>Readings at or above this are treated as invalid.</div>

<label>Door state confirm count</label>
<input type='number' name='doorStateConfirmCount' value=')HTML" + String(config.doorStateConfirmCount) + R"HTML('/>
<div class='hint'>Consecutive agreeing readings required before the reported door state changes.</div>

<input type='submit' value='Save'/>
</form>
)HTML" + message + R"HTML(
</div>
</div>
</body></html>
)HTML";

  server.send(200, "text/html", content);
}

/*
 * Clears the auth cookie and sends the browser back to the login page.
 */
void handleLogout() {
  String header = "HTTP/1.1 301 OK\r\n";
  header += "Set-Cookie: login=; Expires=Thu, 01 Jan 1970 00:00:00 GMT\r\n";
  header += "Location: /login\r\n";
  header += "Cache-Control: no-cache\r\n\r\n";
  server.sendContent(header);
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

  loadConfig();

  // Relay output (button)
  pinMode(BUTTON, OUTPUT);
  digitalWrite(BUTTON, LOW);

  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);

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

  // Wi-Fi driver behavior: keep credentials in flash, rejoin automatically
  // if the link drops, and don't let the radio sleep (more stable on weak
  // signal). Must be set before connecting.
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  // WIFIManager
  // After a power outage this device often boots before the router is back.
  // Without timeouts, autoConnect() would sit in the config portal forever.
  // Give up after a while and reboot so we try the saved network again.
  wifiManager.setConnectTimeout(20);       // seconds to wait for the saved network
  wifiManager.setConfigPortalTimeout(120); // seconds the config portal stays up
  if (!wifiManager.autoConnect("GarageOpener", "password")) {
    Serial.println("failed to connect and hit timeout, restarting to retry");
    delay(3000);

    ESP.restart();
    delay(5000);
  }

  Serial.println("Connected to wifi");
  WiFi.printDiag(Serial);

  // Set up the web server
  server.on("/", handleRoot);
  server.on("/login", handleLogin);

  server.on("/gui", handleGui);

  server.on("/config", handleConfig);

  server.on("/logout", handleLogout);

  server.on("/status", handleStatus);

  server.on("/button", handleButton);

  server.on("/resetrebootflag", handleResetRebootFlag);

  server.on("/dist", detectDistance);
  
  server.onNotFound(handleNotFound);

  // Track the Cookie header so is_authentified() can read it
  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);
  server.collectHeaders(headerkeys, headerkeyssize );

  server.begin();
  Serial.println("HTTP server started");

  // MQTT setup
  client.setServer(config.mqttServer, 1883);
  client.setCallback(callbackMQTT);
  client.setBufferSize(512); // discovery payloads exceed the 256 byte default

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

// Wi-Fi watchdog escalation thresholds, measured from when the link was lost.
#define WIFI_RECONNECT_AFTER_MS   30000UL   // nudge the driver with WiFi.reconnect()
#define WIFI_FULL_RESET_AFTER_MS  120000UL  // tear down and re-begin the connection
#define WIFI_REBOOT_AFTER_MS      600000UL  // last resort: restart the chip

// Watches the Wi-Fi link and escalates recovery the longer it stays down.
// Non-blocking; call every pass through loop().
void checkWifi() {
  static bool wasConnected = true;
  static unsigned long offlineSince = 0;
  static bool triedReconnect = false;
  static bool triedFullReset = false;

  if( WiFi.status() == WL_CONNECTED ) {
    if( !wasConnected ) {
      Serial.print("Wi-Fi restored after ");
      Serial.print((millis() - offlineSince) / 1000);
      Serial.println("s, IP: " + WiFi.localIP().toString());
      // Re-announce the connection once MQTT is back
      needToPublishWifiConnect = true;
      wasConnected = true;
    }
    return;
  }

  // Link is down
  if( wasConnected ) {
    Serial.println("Wi-Fi connection lost");
    wasConnected = false;
    offlineSince = millis();
    triedReconnect = false;
    triedFullReset = false;
    return;
  }

  unsigned long offlineFor = millis() - offlineSince;

  if( offlineFor >= WIFI_REBOOT_AFTER_MS ) {
    Serial.println("Wi-Fi down too long, restarting");
    delay(100);
    ESP.restart();
  }
  else if( offlineFor >= WIFI_FULL_RESET_AFTER_MS && !triedFullReset ) {
    Serial.println("Wi-Fi still down, resetting Wi-Fi stack");
    triedFullReset = true;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // reuse the saved credentials
  }
  else if( offlineFor >= WIFI_RECONNECT_AFTER_MS && !triedReconnect ) {
    Serial.println("Wi-Fi still down, calling WiFi.reconnect()");
    triedReconnect = true;
    WiFi.reconnect();
  }
}

void loop()
{
  // Handle http stuff
  server.handleClient();

  // Supervise the Wi-Fi link
  checkWifi();

  // Handle reconnecting to MQTT if necessary. Skipped while Wi-Fi is down so
  // the rest of loop() (web server, OTA, sensing) keeps running.
  if (!client.connected()) {
    if( WiFi.status() == WL_CONNECTED ) {
      needToPublishMQTTConnect = true;
      reconnectMQTT();
    }
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

// Publishes one retained Home Assistant discovery config message.
void publishDiscoveryEntry(const char* component, const char* objectId, const char* body) {
  char topic[96];
  snprintf(topic, sizeof(topic), "%s/%s/%s/config", DISCOVERY_PREFIX, component, objectId);

  // Every entity shares the same device block so they group under one device.
  char payload[480];
  int n = snprintf(payload, sizeof(payload),
    "{\"unique_id\":\"%s_%s\",%s,"
    "\"availability_topic\":\"" AVAILABILITY_TOPIC "\","
    "\"device\":{\"identifiers\":[\"" HA_DEVICE_ID "\"],"
    "\"name\":\"Garage Opener\",\"manufacturer\":\"DIY\","
    "\"model\":\"ESP-12E + VL53L0X\"}}",
    HA_DEVICE_ID, objectId, body);

  if( n <= 0 || n >= (int)sizeof(payload) ) {
    Serial.print("Discovery payload too large for ");
    Serial.println(objectId);
    return;
  }

  client.publish(topic, payload, true);
}

// Announces the door sensor, raw distance sensor and door button to Home
// Assistant. Safe to call on every connect: the ids are fixed, so Home
// Assistant updates the existing entities rather than creating new ones.
void publishDiscovery() {
  publishDiscoveryEntry("binary_sensor", "door",
    "\"name\":\"Door\",\"device_class\":\"garage_door\","
    "\"state_topic\":\"" DOOR_TOPIC "\","
    "\"payload_on\":\"open\",\"payload_off\":\"closed\"");

  publishDiscoveryEntry("sensor", "distance",
    "\"name\":\"Raw Distance\",\"device_class\":\"distance\","
    "\"unit_of_measurement\":\"mm\",\"state_class\":\"measurement\","
    "\"state_topic\":\"" DISTANCE_TOPIC "\"");

  publishDiscoveryEntry("button", "button",
    "\"name\":\"Door Button\","
    "\"command_topic\":\"" COMMAND_TOPIC "\","
    "\"payload_press\":\"doorbutton\"");
}

// Non-blocking: makes at most one connection attempt every 5 seconds and
// returns immediately otherwise, so loop() keeps servicing everything else.
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  static bool attempted = false;

  if( client.connected() ) return;
  if( attempted && (millis() - lastAttempt) < 5000 ) return;

  attempted = true;
  lastAttempt = millis();

  Serial.print("Attempting MQTT connection...");

  // Attempt to connect, registering a retained last-will so Home Assistant
  // marks the entities unavailable if the device drops off the network.
  if (client.connect(MQTT_CLIENT_ID, NULL, NULL, AVAILABILITY_TOPIC, 0, true, "offline")) {
    Serial.println("connected");

    // ... and resubscribe
    client.subscribe( COMMAND_TOPIC );

    publishDiscovery();
    client.publish( AVAILABILITY_TOPIC, "online", true );
    needToPublishHaState = true;

  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" will retry in 5 seconds");
  }
}

bool lastPublishedDoorOpen = false;

// Publishes the current confirmed door state and raw distance, retained, to
// the topics Home Assistant's discovered entities read from.
void publishHaState() {
  if( !haStateValid ) return;

  client.publish( DOOR_TOPIC, confirmedDoorOpen ? "open" : "closed", true );
  client.publish( DISTANCE_TOPIC, String(lastRawDistance).c_str(), true );

  lastPublishedDoorOpen = confirmedDoorOpen;
  needToPublishHaState = false;
}

void checkDistance() {
  if( (millis() - lastDistanceCheck) >= config.distCheckInterval ) {
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

        if( pendingDoorOpenCount >= config.doorStateConfirmCount ) {
          confirmedDoorOpen = pendingDoorOpen;
          pendingDoorOpenCount = 0;
        }
      }
      else {
        pendingDoorOpenCount = 0;
      }

      digitalWrite(BLUE_LED, confirmedDoorOpen ? HIGH : LOW);

      // Send status messages if distance has changed by the defined percent
      if( lastDistance==0 || abs( ((float)(distanceInfo.distance-lastDistance))/lastDistance )*100 >= config.distChangePercent ) {
        sendMsg = true;
        lastDistance = distanceInfo.distance;
      }

      // Periodically send messages even if percent distance has not changed enough
      else if( (millis() - lastStatusSend) >= config.statusResendInterval ) {
        sendMsg = true;
      }

      lastRawDistance = distanceInfo.distance;
      haStateValid = true;

      // Home Assistant state (retained). Published when the confirmed door
      // state changes (which can lag the distance change by a few readings),
      // on the regular distance/resend triggers, and after (re)connecting.
      if( client.connected() && (needToPublishHaState || sendMsg || confirmedDoorOpen != lastPublishedDoorOpen) ) {
        publishHaState();
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


