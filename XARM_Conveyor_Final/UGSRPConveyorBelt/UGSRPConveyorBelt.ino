// =====================================================
// MCP41010_percent_control_softSPI_with_pin_control.ino
// =====================================================

// For your wiring:
// SI  -> D13
// SCK -> D6
// CS  -> D7

const int PIN_SI  = 13;   // Data to MCP41010
const int PIN_SCK = 6;    // Clock
const int PIN_CS  = 7;    // Chip Select

// New output pins you want to control:
const int OUT_PIN_1 = 2;
const int OUT_PIN_2 = 3;

// MCP41010 nominal end-to-end resistance (10k)
const float R_AB_OHMS = 10000.0;

bool invertPercent = false;

// ====== Software SPI send (MSB first) ======
void softSPItransfer(byte data) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_SI, (data >> i) & 0x01);
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(3);  // small pulse
    digitalWrite(PIN_SCK, LOW);
  }
}

// ====== Write raw wiper value to MCP41010 ======
void writePotRaw(byte value) {
  const byte CMD_WRITE = 0x11;  // write pot 0

  digitalWrite(PIN_CS, LOW);
  softSPItransfer(CMD_WRITE);
  softSPItransfer(value);
  digitalWrite(PIN_CS, HIGH);
}

// ====== Convert percent (0-100) to 0-255 ======
byte percentToValue(float percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  byte v = round((percent / 100.0) * 255.0);
  if (invertPercent) v = 255 - v;
  return v;
}

// ====== Set percent AND pins ======
void setPercentAndPins(float percent, int p1, int p2) {
  // --- Set digital pot ---
  byte wiper = percentToValue(percent);
  writePotRaw(wiper);

  float R_WA = (wiper / 255.0) * R_AB_OHMS;
  float R_WB = ((255 - wiper) / 255.0) * R_AB_OHMS;

  // --- Set pin states ---
  digitalWrite(OUT_PIN_1, p1 ? HIGH : LOW);
  digitalWrite(OUT_PIN_2, p2 ? HIGH : LOW);

  // --- Serial output ---
  Serial.print("Percent: ");
  Serial.print(percent);
  Serial.print("%  -> Wiper = ");
  Serial.println(wiper);

  Serial.print("Pin2 = ");
  Serial.print(p1);
  Serial.print("   Pin3 = ");
  Serial.println(p2);

  Serial.print("R_WA: ");
  Serial.print(R_WA, 1);
  Serial.print(" ohm    R_WB: ");
  Serial.print(R_WB, 1);
  Serial.println(" ohm\n");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_CS, OUTPUT);

  pinMode(OUT_PIN_1, OUTPUT);
  pinMode(OUT_PIN_2, OUTPUT);

  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_SCK, LOW);

  Serial.println("MCP41010 with Pin Control Ready");
  Serial.println("Use: SET <percent>,<pin2>,<pin3>");
  Serial.println("Example: SET 40,1,0");
  Serial.println();

  setPercentAndPins(1, 0, 0);  // startup default
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("SET")) {
      line.remove(0, 3);  // remove "SET"
      line.trim();

      // Expect format: 40,1,0
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        float percent = line.substring(0, firstComma).toFloat();
        int p1 = line.substring(firstComma + 1, secondComma).toInt();
        int p2 = line.substring(secondComma + 1).toInt();

        // setPercentAndPins(percent, p1, p2);
        
      }
    }
  }
  setPercentAndPins(20, 0, 1);
}