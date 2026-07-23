/*
  SkyTime NTP Server - ESP32-P4 Phase 2
  GPS NMEA Parsing + OLED Display + PPS Detection
  
  Board: Waveshare ESP32-P4-ETH with POE
  Hardware:
    - SSD1306 OLED Display (128x64, I2C @ 0x3C)
    - ATGM336H GPS Module (Dual-Mode GPS+BDS, Serial2, 9600 baud, RX:GPIO21)
    - PPS Output from ATGM336H (GPIO1, for Phase 3 preparation)
    - LED Driver Board (WS2812B, GPIO45 for status)
  
  Features:
    - Real-time GPS NMEA parsing
    - OLED display of GPS status
    - PPS pulse detection and counting
    - Lock detection
    - Satellite count
    - Position & altitude display
    - Time/date display
    - PPS frequency monitoring
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// ============ GPIO Pin Definitions (ESP32-P4-ETH) ============
#define GPS_RX_PIN        21
#define GPS_TX_PIN        22
#define PPS_IN_PIN        1      // ← PPS from ATGM336H
#define BUTTON_PIN        2
#define LED_PIN           45

// ============ I2C Configuration ============
#define I2C_SDA_PIN       39
#define I2C_SCL_PIN       40
#define I2C_FREQ          100000

// ============ OLED Configuration ============
#define OLED_WIDTH        128
#define OLED_HEIGHT       64
#define OLED_ADDR         0x3C

// ============ GPS Configuration ============
#define GPS_BAUD_RATE     9600

// ============ Global Objects ============
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
TinyGPSPlus gps;

// ============ PPS Interrupt Variables ============
volatile uint32_t pps_count = 0;
volatile uint32_t pps_pulse_width = 0;
volatile unsigned long pps_last_time = 0;
volatile bool pps_detected = false;
uint32_t last_pps_count = 0;
uint32_t pps_per_second = 0;

// ============ GPS Data Structure ============
struct {
  bool locked = false;
  uint8_t satellites = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
} gps_data;

// ============ PPS Interrupt Handler ============
void IRAM_ATTR pps_interrupt_handler() {
  unsigned long current_time = micros();
  pps_count++;
  pps_pulse_width = current_time - pps_last_time;
  pps_last_time = current_time;
  pps_detected = true;
  
  // Toggle LED on every PPS pulse (visual indicator)
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ============ Function Prototypes ============
void init_i2c();
void init_gps_uart();
void init_gpio();
void init_pps_interrupt();
void init_display();
void update_display();
void process_gps();
void check_pps();
void display_startup();
void display_gps_status();

// ============ I2C Initialization ============
void init_i2c() {
  Serial.print("[I2C] Initializing I2C (SDA:39, SCL:40)... ");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
  delay(100);
  Serial.println("✓");
}

// ============ GPS UART Initialization ============
void init_gps_uart() {
  Serial.print("[GPS] Initializing GPS UART (RX:21, 9600 baud)... ");
  Serial2.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  Serial.println("✓");
}

// ============ GPIO Initialization ============
void init_gpio() {
  Serial.print("[GPIO] Initializing pins... ");
  pinMode(PPS_IN_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("✓");
}

// ============ PPS Interrupt Initialization ============
void init_pps_interrupt() {
  Serial.print("[PPS] Initializing PPS interrupt (GPIO1, rising edge)... ");
  attachInterrupt(digitalPinToInterrupt(PPS_IN_PIN), pps_interrupt_handler, RISING);
  Serial.println("✓");
}

// ============ OLED Display Initialization ============
void init_display() {
  Serial.print("[OLED] Initializing SSD1306 display (0x3C)... ");
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("FAILED!");
    Serial.println("ERROR: SSD1306 display not found at 0x3C");
    return;
  }
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  
  Serial.println("✓");
}

// ============ Display Startup Screen ============
void display_startup() {
  display.clearDisplay();
  
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("SkyTime");
  display.println("NTP");
  
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("GPS-Disciplined");
  display.println("Stratum-1 Server");
  display.println("PPS Ready");
  
  display.display();
  delay(2000);
}

// ============ Display GPS Status ============
void display_gps_status() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Header with PPS indicator
  display.setCursor(0, 0);
  if (pps_detected) {
    display.print("=== GPS [PPS]===");
  } else {
    display.print("=== GPS [----] ===");
  }
  
  // GPS Lock Status & Satellites
  display.setCursor(0, 10);
  if (gps_data.locked) {
    display.print("LOCK: YES");
  } else {
    display.print("LOCK: NO ");
  }
  display.print(" SAT:");
  display.println(gps_data.satellites);
  
  // Latitude
  display.setCursor(0, 20);
  display.print("Lat: ");
  display.println(gps_data.latitude, 5);
  
  // Longitude
  display.setCursor(0, 28);
  display.print("Lon: ");
  display.println(gps_data.longitude, 5);
  
  // Altitude
  display.setCursor(0, 36);
  display.print("Alt: ");
  display.print(gps_data.altitude, 1);
  display.println("m");
  
  // Time
  display.setCursor(0, 44);
  if (gps_data.hour < 10) display.print("0");
  display.print(gps_data.hour);
  display.print(":");
  if (gps_data.minute < 10) display.print("0");
  display.print(gps_data.minute);
  display.print(":");
  if (gps_data.second < 10) display.print("0");
  display.print(gps_data.second);
  display.print(" ");
  
  // PPS frequency
  display.print("PPS:");
  display.print(pps_per_second);
  display.println("Hz");
  
  // Date
  display.setCursor(0, 52);
  if (gps_data.month < 10) display.print("0");
  display.print(gps_data.month);
  display.print("/");
  if (gps_data.day < 10) display.print("0");
  display.print(gps_data.day);
  display.print("/");
  display.print(gps_data.year);
  display.print(" PPS:");
  display.print(pps_count);
  
  // Bottom status bar
  display.setCursor(0, 60);
  display.println("========================");
  
  display.display();
}

// ============ Check PPS Frequency ============
void check_pps() {
  static unsigned long last_check = 0;
  
  if (millis() - last_check > 1000) {
    // Calculate PPS frequency (pulses per second)
    pps_per_second = pps_count - last_pps_count;
    last_pps_count = pps_count;
    
    // Reset PPS detected flag every second
    if (pps_per_second == 0) {
      pps_detected = false;
    } else {
      pps_detected = true;
    }
    
    last_check = millis();
  }
}

// ============ Process GPS Data ============
void process_gps() {
  // Read data from GPS module
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }
  
  // Update GPS data if new data received
  if (gps.location.isUpdated()) {
    gps_data.latitude = gps.location.lat();
    gps_data.longitude = gps.location.lng();
  }
  
  if (gps.altitude.isUpdated()) {
    gps_data.altitude = gps.altitude.meters();
  }
  
  if (gps.date.isUpdated()) {
    gps_data.year = gps.date.year();
    gps_data.month = gps.date.month();
    gps_data.day = gps.date.day();
  }
  
  if (gps.time.isUpdated()) {
    gps_data.hour = gps.time.hour();
    gps_data.minute = gps.time.minute();
    gps_data.second = gps.time.second();
  }
  
  // Update satellite count
  gps_data.satellites = gps.satellites.value();
  
  // Determine lock status (need at least 4 satellites for 3D fix)
  gps_data.locked = (gps_data.satellites >= 4) && gps.location.isValid();
}

// ============ Update Display ============
void update_display() {
  static unsigned long last_update = 0;
  
  // Update display every 500ms
  if (millis() - last_update > 500) {
    display_gps_status();
    last_update = millis();
  }
}

// ============ Arduino Setup Function ============
void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(500);
  
  // Print startup message
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   SkyTime NTP Server - ESP32-P4       ║");
  Serial.println("║   Phase 2: GPS + OLED + PPS Detection ║");
  Serial.println("║   GPS-Disciplined Stratum-1 NTP       ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize all systems
  init_gpio();
  delay(100);
  
  init_i2c();
  delay(100);
  
  init_display();
  delay(100);
  
  init_gps_uart();
  delay(100);
  
  init_pps_interrupt();
  delay(100);
  
  // Show startup screen
  display_startup();
  
  Serial.println();
  Serial.println("═══════════════════════════════════════");
  Serial.println("Initialization Complete!");
  Serial.println("Waiting for GPS lock and PPS signal...");
  Serial.println("═══════════════════════════════════════");
  Serial.println();
}

// ============ Arduino Loop Function ============
void loop() {
  // Process GPS data continuously
  process_gps();
  
  // Check PPS frequency
  check_pps();
  
  // Update OLED display
  update_display();
  
  // Print GPS & PPS status to serial every 5 seconds
  static unsigned long last_serial = 0;
  if (millis() - last_serial > 5000) {
    Serial.print("[GPS] Locked: ");
    Serial.print(gps_data.locked ? "YES" : "NO");
    Serial.print(" | Satellites: ");
    Serial.print(gps_data.satellites);
    Serial.print(" | Lat: ");
    Serial.print(gps_data.latitude, 6);
    Serial.print(" | Lon: ");
    Serial.print(gps_data.longitude, 6);
    Serial.print(" | [PPS] Count: ");
    Serial.print(pps_count);
    Serial.print(" | Freq: ");
    Serial.print(pps_per_second);
    Serial.println("Hz");
    
    last_serial = millis();
  }
  
  delay(10);
}
