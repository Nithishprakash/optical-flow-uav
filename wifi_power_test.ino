/*
 * wifi_power_test.ino
 * 
 * Hardware validation and testing utility for ESP32-based UAVs.
 * 
 * This script hosts a local web server accessible via a smartphone 
 * to manually control the throttle of four independent ESCs. 
 * It is primarily used for:
 * - Verifying circuitry and ESC calibration.
 * - Testing correct motor spin directions.
 * - Streaming real-time MPU6050 IMU telemetry to the web interface.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "ABCD";
const char* password = "12345678";

WebServer server(80);
Adafruit_MPU6050 mpu;

Servo escFL;
Servo escFR;
Servo escBL;
Servo escBR;

const int pinFL = 13;
const int pinFR = 12;
const int pinBL = 26;
const int pinBR = 27;


const int MIN_PULSE = 1000;
const int MAX_LIMIT_PULSE = 2000; 

const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
	<meta name="viewport" content="width=device-width, initial-scale=1">
	<style>
		body { font-family: Arial; text-align: center; margin-top: 20px; background-color: #121212; color: white; }
		input[type=range] { width: 70%; margin: 10px 0; }
		.container { border: 1px solid #333; padding: 15px; margin: 10px auto; width: 90%; max-width: 450px; border-radius: 10px; background-color: #1e1e1e; }
		h2, h3 { color: #ff4c4c; margin-bottom: 5px; }
		button { padding: 10px 20px; background-color: #ff4c4c; color: white; border: none; border-radius: 5px; cursor: pointer; font-weight: bold; }
		button:hover { background-color: #ff3333; }
		.sensor-data { font-family: monospace; font-size: 16px; text-align: left; display: inline-block; }
	</style>
	<script>
		function updateText(id, val) {
			document.getElementById(id).innerText = val + '%';
		}
		
		function sendMaster() {
			let val = document.getElementById('sliderMaster').value;
			fetch('/setMaster?val=' + val);
			
			// Sync visual sliders
			document.getElementById('sliderFL').value = val;
			document.getElementById('valFL').innerText = val + '%';
			document.getElementById('sliderFR').value = val;
			document.getElementById('valFR').innerText = val + '%';
			document.getElementById('sliderBL').value = val;
			document.getElementById('valBL').innerText = val + '%';
			document.getElementById('sliderBR').value = val;
			document.getElementById('valBR').innerText = val + '%';
		}

		function sendMotor(motorId) {
			let val = document.getElementById('slider' + motorId).value;
			fetch('/set?m=' + motorId + '&val=' + val);
		}

		// Fetch IMU data every 500ms
		setInterval(() => {
			fetch('/imu')
				.then(response => response.json())
				.then(data => {
					if(data.error) {
						document.getElementById('imu-readings').innerHTML = "MPU Error/Disconnected";
					} else {
						document.getElementById('imu-readings').innerHTML = 
							"Accel X: " + data.aX + " | Y: " + data.aY + " | Z: " + data.aZ + "<br>" +
							"Gyro  X: " + data.gX + " | Y: " + data.gY + " | Z: " + data.gZ;
					}
				})
				.catch(err => console.log(err));
		}, 500);
	</script>
</head>
<body>
	<h2>Flight Controller Test</h2>
	<p><strong>DANGER: 100% THROTTLE UNLOCKED</strong></p>

	<div class="container">
		<h3>MPU-9250 Readings</h3>
		<div class="sensor-data" id="imu-readings">Loading IMU data...</div>
	</div>

	<div class="container">
		<h3>Master Control</h3>
		<input type="range" min="0" max="100" value="0" id="sliderMaster" oninput="updateText('valMaster', this.value)"><br>
		<span id="valMaster">0%</span> 
		<button onclick="sendMaster()">SEND ALL</button>
	</div>

	<div class="container">
		<h3>Individual Motors</h3>
		FL (Pin 13): <input type="range" min="0" max="100" value="0" id="sliderFL" oninput="updateText('valFL', this.value)"> <span id="valFL">0%</span> <button onclick="sendMotor('FL')">Send</button><br>
		FR (Pin 12): <input type="range" min="0" max="100" value="0" id="sliderFR" oninput="updateText('valFR', this.value)"> <span id="valFR">0%</span> <button onclick="sendMotor('FR')">Send</button><br>
		BL (Pin 36): <input type="range" min="0" max="100" value="0" id="sliderBL" oninput="updateText('valBL', this.value)"> <span id="valBL">0%</span> <button onclick="sendMotor('BL')">Send</button><br>
		BR (Pin 27): <input type="range" min="0" max="100" value="0" id="sliderBR" oninput="updateText('valBR', this.value)"> <span id="valBR">0%</span> <button onclick="sendMotor('BR')">Send</button><br>
	</div>
</body>
</html>
)rawliteral";

void handleRoot() {
	server.send(200, "text/html", htmlPage);
}

void applyThrottle(Servo& esc, int percent) {
	int pulse = map(percent, 0, 100, MIN_PULSE, MAX_LIMIT_PULSE);
	esc.writeMicroseconds(pulse);
}

void handleSetMotor() {
	if (server.hasArg("m") && server.hasArg("val")) {
		String m = server.arg("m");
		int val = server.arg("val").toInt();
		
		if (m == "FL") applyThrottle(escFL, val);
		else if (m == "FR") applyThrottle(escFR, val);
		else if (m == "BL") applyThrottle(escBL, val);
		else if (m == "BR") applyThrottle(escBR, val);
		
		server.send(200, "text/plain", "OK");
	} else {
		server.send(400, "text/plain", "Bad Request");
	}
}

void handleSetMaster() {
	if (server.hasArg("val")) {
		int val = server.arg("val").toInt();
		
		applyThrottle(escFL, val);
		applyThrottle(escFR, val);
		applyThrottle(escBL, val);
		applyThrottle(escBR, val);
		
		server.send(200, "text/plain", "OK");
	} else {
		server.send(400, "text/plain", "Bad Request");
	}
}

void handleIMU() {
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
	
	String json = "{";
	json += "\"aX\":\"" + String(a.acceleration.x, 2) + "\",";
	json += "\"aY\":\"" + String(a.acceleration.y, 2) + "\",";
	json += "\"aZ\":\"" + String(a.acceleration.z, 2) + "\",";
	json += "\"gX\":\"" + String(g.gyro.x, 2) + "\",";
	json += "\"gY\":\"" + String(g.gyro.y, 2) + "\",";
	json += "\"gZ\":\"" + String(g.gyro.z, 2) + "\"";
	json += "}";
	
	server.send(200, "application/json", json);
}

void setup() {
	Serial.begin(115200);
	
	Wire.begin(21, 22);
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU sensor!");
	} else {
		Serial.println("MPU sensor initialized.");
		mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
		mpu.setGyroRange(MPU6050_RANGE_500_DEG);
		mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
	}
	
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	
	escFL.setPeriodHertz(50);
	escFR.setPeriodHertz(50);
	escBL.setPeriodHertz(50);
	escBR.setPeriodHertz(50);
	
	escFL.attach(pinFL, 1000, 2000);
	escFR.attach(pinFR, 1000, 2000);
	escBL.attach(pinBL, 1000, 2000);
	escBR.attach(pinBR, 1000, 2000);
	
	Serial.println("Arming ESCs...");
	escFL.writeMicroseconds(MIN_PULSE);
	escFR.writeMicroseconds(MIN_PULSE);
	escBL.writeMicroseconds(MIN_PULSE);
	escBR.writeMicroseconds(MIN_PULSE);
	delay(3000);
	
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	
	Serial.println("\nWiFi Connected.");
	Serial.println(WiFi.localIP());
	
	server.on("/", HTTP_GET, handleRoot);
	server.on("/set", HTTP_GET, handleSetMotor);
	server.on("/setMaster", HTTP_GET, handleSetMaster);
	server.on("/imu", HTTP_GET, handleIMU);
	
	server.begin();
}

void loop() {
	server.handleClient();
}
