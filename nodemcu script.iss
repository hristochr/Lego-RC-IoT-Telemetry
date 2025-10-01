#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>


// Create web server on port 80
ESP8266WebServer server(80);

// WiFi credentials
const char* ssid = "your-ssid";
const char* password = "your-password";

// MQTT Broker Connection Details 
const char* mqtt_server = "mqtt_server.eu.hivemq.cloud";
const char* mqtt_username = "user";
const char* mqtt_password = "password";
const int mqtt_port =8883;

// root certificate
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// simple data structure to store speed readings
struct SpeedResult {
  float metersPerSecond;
  float kilometersPerHour;
  bool valid;
};

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Hall sensor configuration
const int HALL_PIN = D2;           
const int MAGNET_COUNT = 2;        // Number of magnets on the wheel
const float WHEEL_CIRCUMFERENCE = 0.3361504; // Wheel circumference in meters

// Speed calculation variables
volatile unsigned long lastPulseTime = 0;
volatile unsigned long currentPulseTime = 0;
volatile unsigned long previousPulseTime = 0;
volatile bool newPulseDetected = false;
float currentSpeed = 0.0;  // Speed in m/s
float currentSpeedKmh = 0.0; // Speed in km/h

// Variables for speed smoothing
const int SPEED_BUFFER_SIZE = 5;  // Number of readings to average
float speedBuffer[SPEED_BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;
float smoothedSpeed = 0.0;
float smoothedSpeedKmh = 0.0;

// Sensor reading variables
int hallValue = 0;
bool magnetDetected = false;
bool lastMagnetState = false;
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 1000; // Display speed every 1 second

// Timeout for speed calculation (if no pulse for 3 seconds, speed = 0)
const unsigned long SPEED_TIMEOUT = 3000;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Lego RC Telemetry ===");
  
  // Initialize WiFi
  connectToWiFi();
  
  // Initialize hall sensor
  pinMode(HALL_PIN, INPUT);
  
  Serial.println("Hall sensor initialized on pin A0");
  Serial.println("Calibrating sensor...");

  // Setup web server
  setupWebServer();
  // Configure TLS
  #ifdef ESP8266
    espClient.setInsecure();
  #else
    espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  #endif

  client.setServer(mqtt_server, mqtt_port);
  
  Serial.println("Setup complete. Monitoring speed...");
  Serial.print("Web interface available at: http://");
  Serial.println(WiFi.localIP());
  Serial.println("Rotate wheel with magnet to see speed readings.\n");
}

void loop() {
  // Read hall sensor
  readHallSensor();
  
  
  // Check for speed timeout (wheel stopped)
  checkSpeedTimeout();
  
  // Handle web server requests
  server.handleClient();
  
  // connect to MQTT
  if (!client.connected()) reconnect(); // check if client is connected
  client.loop();

  DynamicJsonDocument doc(1024);

  // Calculate speed if new pulse detected
  if (newPulseDetected) {
    Serial.println("Processing new pulse for speed calculation...");
    
    SpeedResult result = calculateSpeed();
    doc["speed_ms"] = result.metersPerSecond;
    doc["speed_kmh"] = result.kilometersPerHour;
    newPulseDetected = false;

    char mqtt_message[128];

    if (result.valid) {
      serializeJson(doc, mqtt_message);
      publishMessage("esp8266_data", mqtt_message, false);
    }
  }

  delay(0.1); // Small delay for stability. The longer the delay the more innacurate the overall reading!
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm\n");
  } else {
    Serial.println("\nFailed to connect to WiFi");
    Serial.println("Continuing without WiFi connection\n");
  }
}

void readHallSensor() {
  hallValue = digitalRead(HALL_PIN);
  
  // Your sensor: HIGH = magnet detected, LOW = no magnet
  magnetDetected = (hallValue == LOW);
  
  // Detect rising edge (magnet approaching)
  if (magnetDetected && !lastMagnetState) {
    // Magnet detected - record pulse time
    previousPulseTime = currentPulseTime;  // Save previous pulse time
    currentPulseTime = millis();
    
    Serial.print("Pulse detected at: ");
    Serial.print(currentPulseTime);
    
    if (previousPulseTime > 0) {
      // We have a previous pulse to calculate speed from
      unsigned long timeDiff = currentPulseTime - previousPulseTime;
      Serial.print(", Time since last pulse: ");
      Serial.print(timeDiff);
      Serial.println(" ms");
      Serial.println("Setting newPulseDetected = true");
      newPulseDetected = true;
    } else {
      Serial.println(" (first pulse)");
    }
  }
  
  lastMagnetState = magnetDetected;
}

SpeedResult calculateSpeed() {
  SpeedResult result = {0.0, 0.0, false};

  Serial.print("calculateSpeed() called - previousPulseTime: ");
  Serial.print(previousPulseTime);
  Serial.print(", currentPulseTime: ");
  Serial.println(currentPulseTime);

  if (previousPulseTime > 0 && currentPulseTime > previousPulseTime) {
    unsigned long timeDifference = currentPulseTime - previousPulseTime;

    Serial.print("Time difference: ");
    Serial.print(timeDifference);
    Serial.println(" ms");

    if (timeDifference > 0) {
      // Calculate instantaneous speed
      float distancePerPulse = WHEEL_CIRCUMFERENCE / MAGNET_COUNT;
      float timeInSeconds = timeDifference / 1000.0;    
      
      result.metersPerSecond = distancePerPulse / timeInSeconds;
      result.kilometersPerHour = result.metersPerSecond * 3.6;
      
      if (result.metersPerSecond > 0.01) {  // Ignore very slow speeds
        result.valid = true;
        
        // Add to smoothing buffer
        addToSpeedBuffer(result.metersPerSecond);
        
        // Use smoothed values for display/ send to MQTT
        result.metersPerSecond = smoothedSpeed;
        result.kilometersPerHour = smoothedSpeedKmh;
      }

      Serial.print("Raw speed: ");
      Serial.print(distancePerPulse / timeInSeconds, 3);
      Serial.print(" m/s, Smoothed: ");
      Serial.print(result.metersPerSecond, 3);
      Serial.print(" m/s (");
      Serial.print(result.kilometersPerHour, 2);
      Serial.println(" km/h)");
    }
  }

  return result;
}

void checkSpeedTimeout() {
  if (millis() - lastPulseTime > SPEED_TIMEOUT && lastPulseTime > 0) {
    // No pulse detected for a while, assume speed is 0
    smoothedSpeed = 0.0;
    smoothedSpeedKmh = 0.0;
  }
}

void setupWebServer() {
  // Main page route
  server.on("/", handleRoot);
  
  // API endpoint for JSON data
  server.on("/api/data", handleAPI);
  
  // Start the server
  server.begin();
  Serial.println("Web server started");
}

void addToSpeedBuffer(float newSpeed) {
  speedBuffer[bufferIndex] = newSpeed;
  bufferIndex = (bufferIndex + 1) % SPEED_BUFFER_SIZE;
  
  if (bufferIndex == 0) bufferFilled = true;
  
  // Calculate average
  float sum = 0;
  int count = bufferFilled ? SPEED_BUFFER_SIZE : bufferIndex;
  
  for (int i = 0; i < count; i++) {
    sum += speedBuffer[i];
  }
  
  smoothedSpeed = sum / count;
  smoothedSpeedKmh = smoothedSpeed * 3.6;
  
  Serial.print("Speed buffer: [");
  for (int i = 0; i < count; i++) {
    Serial.print(speedBuffer[i], 2);
    if (i < count - 1) Serial.print(", ");
  }
  Serial.print("] -> Average: ");
  Serial.println(smoothedSpeed, 3);
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP8266 Speed Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 0; 
            padding: 20px; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            min-height: 100vh;
        }
        .container { 
            max-width: 800px; 
            margin: 0 auto; 
            background: rgba(255,255,255,0.1);
            padding: 30px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
            box-shadow: 0 8px 32px rgba(0,0,0,0.1);
        }
        h1 { 
            text-align: center; 
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .speed-display { 
            text-align: center; 
            font-size: 3em; 
            font-weight: bold; 
            margin: 30px 0;
            padding: 20px;
            background: rgba(255,255,255,0.2);
            border-radius: 10px;
            text-shadow: 1px 1px 2px rgba(0,0,0,0.3);
        }
        .info-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin: 30px 0;
        }
        .info-card {
            background: rgba(255,255,255,0.15);
            padding: 20px;
            border-radius: 10px;
            text-align: center;
        }
        .info-card h3 {
            margin: 0 0 10px 0;
            color: #ffd700;
        }
        .info-card p {
            margin: 5px 0;
            font-size: 1.1em;
        }
        .status-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .status-connected { background-color: #4CAF50; }
        .status-disconnected { background-color: #f44336; }
        .magnet-detected { background-color: #ff9800; }
        .magnet-not-detected { background-color: #666; }
        .update-time {
            text-align: center;
            font-style: italic;
            margin-top: 20px;
            opacity: 0.8;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Lego RC Speed Monitor</h1>
        
        <div class="speed-display" id="speedDisplay">
            <div id="speedValue">-- km/h</div>
        </div>
        
        <div class="info-grid">
            <div class="info-card">
                <h3>Hall Sensor</h3>
                <p>Value: <span id="hallValue">--</span></p>
                <p>
                    <span class="status-indicator" id="magnetIndicator"></span>
                    <span id="magnetStatus">--</span>
                </p>
            </div>
            
            <div class="info-card">
                <h3>Speed Details</h3>
                <p><span id="speedMS">-- m/s</span></p>
                <p><span id="speedKMH">-- km/h</span></p>
            </div>
            
            <div class="info-card">
                <h3>Connection</h3>
                <p>
                    <span class="status-indicator" id="wifiIndicator"></span>
                    WiFi: <span id="wifiStatus">--</span>
                </p>
                <p>Signal: <span id="signalStrength">--</span></p>
            </div>
        </div>
        
        <div class="update-time">
            Last updated: <span id="lastUpdate">--</span>
        </div>
    </div>

    <script>
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    // Update speed display
                    document.getElementById('speedValue').textContent = data.speedKmh.toFixed(1) + ' km/h';
                    
                    // Update hall sensor info
                    document.getElementById('hallValue').textContent = data.hallValue;
                    document.getElementById('magnetStatus').textContent = data.magnetDetected ? 'DETECTED' : 'Not detected';
                    
                    // Update magnet indicator
                    const magnetIndicator = document.getElementById('magnetIndicator');
                    magnetIndicator.className = 'status-indicator ' + (data.magnetDetected ? 'magnet-detected' : 'magnet-not-detected');
                    
                    // Update speed details
                    document.getElementById('speedMS').textContent = data.speedMS.toFixed(2) + ' m/s';
                    document.getElementById('speedKMH').textContent = data.speedKmh.toFixed(2) + ' km/h';
                    
                    // Update WiFi status
                    document.getElementById('wifiStatus').textContent = data.wifiConnected ? 'Connected' : 'Disconnected';
                    document.getElementById('signalStrength').textContent = data.signalStrength + ' dBm';
                    
                    // Update WiFi indicator
                    const wifiIndicator = document.getElementById('wifiIndicator');
                    wifiIndicator.className = 'status-indicator ' + (data.wifiConnected ? 'status-connected' : 'status-disconnected');
                    
                    // Update timestamp
                    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
                })
                .catch(error => {
                    console.error('Error fetching data:', error);
                    document.getElementById('speedValue').textContent = 'Error';
                });
        }
        
        // Update data every second
        setInterval(updateData, 1000);
        
        // Initial load
        updateData();
    </script>
</body>
</html>
)";
  
  server.send(200, "text/html", html);
}

void handleAPI() {
  // Create JSON response with current data
  String json = "{";
  json += "\"hallValue\":" + String(hallValue) + ",";
  json += "\"magnetDetected\":" + String(magnetDetected ? "true" : "false") + ",";
  json += "\"speedMS\":" + String(smoothedSpeed, 2) + ",";
  json += "\"speedKmh\":" + String(smoothedSpeedKmh, 2) + ",";
  json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"signalStrength\":" + String(WiFi.RSSI());
  json += "}";
  
  server.send(200, "application/json", json);
}

// Connect to MQTT Broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      client.subscribe("led_state");   // subscribe the topics here

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Method for Publishing MQTT Messages
void publishMessage(const char* topic, String payload, boolean retained){
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}