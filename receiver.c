/*
 * NRF24L01+ PA+LNA Receiver for Arduino Pro Micro
 * Receives telemetry from rocket flight computer
 * Forwards data to PC via USB Serial
 */

#include <SPI.h>

/* ===================== PIN CONFIGURATION ===================== */
// Arduino Pro Micro SPI pins:
// MOSI: 16
// MISO: 14
// SCK: 15
// For NRF24L01, we need CE and CSN

#define NRF_CE  9   // Any digital pin
#define NRF_CSN 10  // Any digital pin
#define LED_PIN LED_BUILTIN

/* ===================== NRF24L01 REGISTERS ===================== */
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_RX_ADDR_P0  0x0A
#define NRF_RX_PW_P0    0x11
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD       0x1C
#define NRF_FEATURE     0x1D

#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_NOP           0xFF

#define NRF_CHANNEL 92  // Must match transmitter

/* ===================== TELEMETRY PACKET ===================== */
struct TelemetryPacket {
  uint32_t timestamp;
  uint8_t state;
  float altitude;
  float velocity;
  float acceleration;
  float max_altitude;
  float max_velocity;
  float max_accel;
  uint8_t battery_percent;
  uint8_t checksum;
} __attribute__((packed));

TelemetryPacket rxPacket;

uint8_t rxAddress[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};  // Must match transmitter

const char* state_names[] = {
  "BOOT", "SENSOR_INIT", "CALIBRATING", "PAD_IDLE", "ARMED", 
  "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "ERROR"
};

/* ===================== STATISTICS ===================== */
struct Stats {
  uint32_t packets_received;
  uint32_t packets_lost;
  uint32_t checksum_errors;
  unsigned long last_packet_time;
  float rssi_estimate;
  uint8_t last_state;
} stats = {0};

/* ===================== NRF24L01 FUNCTIONS ===================== */
uint8_t nrf_read_register(uint8_t reg) {
  digitalWrite(NRF_CSN, LOW);
  SPI.transfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
  uint8_t result = SPI.transfer(0xFF);
  digitalWrite(NRF_CSN, HIGH);
  return result;
}

void nrf_write_register(uint8_t reg, uint8_t value) {
  digitalWrite(NRF_CSN, LOW);
  SPI.transfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  SPI.transfer(value);
  digitalWrite(NRF_CSN, HIGH);
}

void nrf_write_register_multi(uint8_t reg, const uint8_t* data, uint8_t len) {
  digitalWrite(NRF_CSN, LOW);
  SPI.transfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(NRF_CSN, HIGH);
}

void nrf_flush_rx() {
  digitalWrite(NRF_CSN, LOW);
  SPI.transfer(NRF_CMD_FLUSH_RX);
  digitalWrite(NRF_CSN, HIGH);
}

void nrf_read_payload(uint8_t* data, uint8_t len) {
  digitalWrite(NRF_CSN, LOW);
  SPI.transfer(NRF_CMD_R_RX_PAYLOAD);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = SPI.transfer(0xFF);
  }
  digitalWrite(NRF_CSN, HIGH);
}

bool nrf_init() {
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CSN, OUTPUT);
  digitalWrite(NRF_CE, LOW);
  digitalWrite(NRF_CSN, HIGH);
  
  delay(100);
  
  // Test communication
  nrf_write_register(NRF_CONFIG, 0x0C);
  delay(5);
  uint8_t config = nrf_read_register(NRF_CONFIG);
  
  if (config != 0x0C) {
    Serial.print(F("NRF24L01 init failed - CONFIG: 0x"));
    Serial.println(config, HEX);
    return false;
  }
  
  // Configure for RX mode (PA+LNA)
  nrf_write_register(NRF_CONFIG, 0x0F);        // Power up, RX mode, CRC enabled (2 bytes)
  nrf_write_register(NRF_EN_AA, 0x00);         // DISABLED - no auto-ack (fire-and-forget)
  nrf_write_register(NRF_EN_RXADDR, 0x01);     // Enable RX pipe 0
  nrf_write_register(NRF_SETUP_AW, 0x03);      // 5-byte address
  nrf_write_register(NRF_SETUP_RETR, 0x00);    // DISABLED - no retries
  nrf_write_register(NRF_RF_CH, NRF_CHANNEL);  // Channel 92
  nrf_write_register(NRF_RF_SETUP, 0x01);      // 1Mbps, -18dBm (PA+LNA will amplify)
  nrf_write_register(NRF_RX_PW_P0, 32);        // 32-byte payload on pipe 0
  
  // Set RX address (must match TX address)
  nrf_write_register_multi(NRF_RX_ADDR_P0, rxAddress, 5);
  
  // Clear flags and flush
  nrf_flush_rx();
  nrf_write_register(NRF_STATUS, 0x70);
  
  delay(10);
  
  // Start listening
  digitalWrite(NRF_CE, HIGH);
  
  Serial.println(F("NRF24L01+ PA+LNA initialized (RX mode)"));
  Serial.print(F("Channel: "));
  Serial.println(NRF_CHANNEL);
  Serial.print(F("Address: "));
  for (int i = 0; i < 5; i++) {
    Serial.print(F("0x"));
    Serial.print(rxAddress[i], HEX);
    if (i < 4) Serial.print(F(":"));
  }
  Serial.println();
  
  return true;
}

uint8_t calculate_checksum(const uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum ^= data[i];
  }
  return sum;
}

bool nrf_data_available() {
  uint8_t status = nrf_read_register(NRF_STATUS);
  
  // Check if RX FIFO is not empty
  if (status & (1 << 6)) {  // RX_DR flag
    return true;
  }
  
  uint8_t fifo_status = nrf_read_register(NRF_FIFO_STATUS);
  return !(fifo_status & 0x01);  // RX_EMPTY bit
}

void process_packet() {
  // Read packet
  nrf_read_payload((uint8_t*)&rxPacket, sizeof(TelemetryPacket));
  
  // Clear RX flag
  nrf_write_register(NRF_STATUS, (1 << 6));
  
  // Verify checksum
  uint8_t calc_checksum = calculate_checksum((uint8_t*)&rxPacket, sizeof(TelemetryPacket) - 1);
  
  if (calc_checksum != rxPacket.checksum) {
    stats.checksum_errors++;
    Serial.println(F("CHECKSUM_ERROR"));
    return;
  }
  
  // Valid packet received
  stats.packets_received++;
  stats.last_packet_time = millis();
  stats.last_state = rxPacket.state;
  
  // Output telemetry in CSV format (compatible with dashboard)
  Serial.print(rxPacket.timestamp / 1000.0, 3);
  Serial.print(F(","));
  Serial.print(state_names[rxPacket.state]);
  Serial.print(F(","));
  Serial.print(rxPacket.altitude, 2);
  Serial.print(F(","));
  Serial.print(rxPacket.velocity, 2);
  Serial.print(F(","));
  Serial.println(rxPacket.acceleration, 3);
  
  // Blink LED on packet receive
  digitalWrite(LED_PIN, HIGH);
  delay(1);
  digitalWrite(LED_PIN, LOW);
}

void print_stats() {
  static unsigned long last_stats = 0;
  unsigned long now = millis();
  
  if (now - last_stats < 5000) return;  // Print every 5 seconds
  last_stats = now;
  
  Serial.println(F("\n===== RECEIVER STATS ====="));
  Serial.print(F("Packets received: "));
  Serial.println(stats.packets_received);
  Serial.print(F("Checksum errors: "));
  Serial.println(stats.checksum_errors);
  
  if (stats.packets_received > 0) {
    float error_rate = (stats.checksum_errors * 100.0) / (stats.packets_received + stats.checksum_errors);
    Serial.print(F("Error rate: "));
    Serial.print(error_rate, 2);
    Serial.println(F("%"));
  }
  
  Serial.print(F("Last packet: "));
  if (now - stats.last_packet_time < 10000) {
    Serial.print(now - stats.last_packet_time);
    Serial.println(F(" ms ago"));
    Serial.print(F("Last state: "));
    Serial.println(state_names[stats.last_state]);
  } else {
    Serial.println(F("NO SIGNAL"));
  }
  
  Serial.println(F("==========================\n"));
}

void check_connection() {
  static unsigned long last_check = 0;
  unsigned long now = millis();
  
  if (now - last_check < 3000) return;
  last_check = now;
  
  // If no packets for 3+ seconds, warn
  if (stats.packets_received > 0 && now - stats.last_packet_time > 3000) {
    Serial.println(F("WARNING: No packets received for 3+ seconds"));
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for USB serial
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║  NRF24L01+ PA+LNA Receiver         ║"));
  Serial.println(F("║  Arduino Pro Micro Ground Station  ║"));
  Serial.println(F("╚════════════════════════════════════╝\n"));
  
  // Initialize SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);  // 4 MHz
  
  // Initialize NRF24L01
  Serial.println(F("Initializing NRF24L01+..."));
  if (!nrf_init()) {
    Serial.println(F("FATAL: NRF24L01 initialization failed!"));
    Serial.println(F("Check wiring:"));
    Serial.println(F("  CE  -> Pin 9"));
    Serial.println(F("  CSN -> Pin 10"));
    Serial.println(F("  MOSI -> Pin 16"));
    Serial.println(F("  MISO -> Pin 14"));
    Serial.println(F("  SCK  -> Pin 15"));
    Serial.println(F("  VCC  -> 3.3V"));
    Serial.println(F("  GND  -> GND"));
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  Serial.println(F("\n✓ Receiver ready!"));
  Serial.println(F("Listening for telemetry..."));
  Serial.println(F("\nTime(s) | State | AGL(m) | Velocity(m/s) | Accel(g)\n"));
  
  stats.last_packet_time = millis();
}

/* ===================== MAIN LOOP ===================== */
void loop() {
  // Check for incoming packets
  if (nrf_data_available()) {
    process_packet();
  }
  
  // Print statistics
  print_stats();
  
  // Check connection status
  check_connection();
  
  delay(1);  // Small delay to prevent overwhelming CPU
}
