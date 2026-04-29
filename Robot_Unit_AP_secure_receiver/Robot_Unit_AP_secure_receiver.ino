#include <WiFi.h>
#include <AESLib.h>

const char* ssid = "Painting_Robot_Arm";
const char* password = "12345678";

WiFiServer server(3333);
WiFiClient activeClient;
AESLib aesLib;

const uint32_t DH_P = 2147483647UL;
const uint32_t DH_G = 5;

const size_t MAX_CIPHER_BUF_LEN = 256;

bool dhSent = false;
bool dhReceived = false;
bool sessionReady = false;

uint32_t serverPrivateKey = 0;
uint32_t serverPublicKey = 0;
uint32_t clientPublicKey = 0;
uint32_t sharedSecret = 0;

byte keyClientToServer[16];
byte keyServerToClient[16];
byte ivClientToServer[16];
byte ivServerToClient[16];

uint32_t modExp(uint32_t base, uint32_t exp, uint32_t mod) {
  uint64_t result = 1;
  uint64_t b = base % mod;

  while (exp > 0) {
    if (exp & 1) result = (result * b) % mod;
    b = (b * b) % mod;
    exp >>= 1;
  }

  return (uint32_t)result;
}

uint32_t generatePrivateKey() {
  return (uint32_t)random(10000, 60000);
}

void deriveDirectionalMaterial(uint32_t secret) {
  byte s[4];
  s[0] = (byte)((secret >> 24) & 0xFF);
  s[1] = (byte)((secret >> 16) & 0xFF);
  s[2] = (byte)((secret >> 8) & 0xFF);
  s[3] = (byte)(secret & 0xFF);

  for (int i = 0; i < 16; i++) {
    keyClientToServer[i] = (byte)(s[i % 4] ^ (0x21 + i * 7) ^ 0xC2);
    keyServerToClient[i] = (byte)(s[(i + 1) % 4] ^ (0x42 + i * 5) ^ 0x5A);
    ivClientToServer[i]  = (byte)(s[(i + 2) % 4] ^ (0x11 + i * 3) ^ 0x9C);
    ivServerToClient[i]  = (byte)(s[(i + 3) % 4] ^ (0x33 + i * 9) ^ 0x6D);
  }
}

char nibbleToHex(uint8_t nibble) {
  return (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
}

int hexCharToNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool hexStringToBytes(const String& hex, byte* out, size_t& outLen) {
  if (hex.length() % 2 != 0) return false;

  outLen = hex.length() / 2;
  for (size_t i = 0; i < outLen; i++) {
    int hi = hexCharToNibble(hex[2 * i]);
    int lo = hexCharToNibble(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (byte)((hi << 4) | lo);
  }

  return true;
}

bool decryptHexToText(const String& cipherHex, const byte* key, const byte* ivBase, String& outText) {
  byte cipherBuf[MAX_CIPHER_BUF_LEN];
  memset(cipherBuf, 0, sizeof(cipherBuf));

  size_t cipherLen = 0;
  if (!hexStringToBytes(cipherHex, cipherBuf, cipherLen)) return false;

  byte plainBuf[MAX_CIPHER_BUF_LEN];
  memset(plainBuf, 0, sizeof(plainBuf));

  byte ivWork[16];
  memcpy(ivWork, ivBase, 16);

  aesLib.set_paddingmode((paddingMode)0);
  uint16_t decLen = aesLib.decrypt(
    cipherBuf,
    cipherLen,
    plainBuf,
    (byte*)key,
    16,
    ivWork
  );

  if (decLen == 0) return false;
  outText = String((char*)plainBuf);
  return true;
}

void sendLine(const String& line) {
  if (activeClient && activeClient.connected()) {
    activeClient.println(line);
  }
}

void resetSession() {
  dhSent = false;
  dhReceived = false;
  sessionReady = false;

  serverPrivateKey = 0;
  serverPublicKey = 0;
  clientPublicKey = 0;
  sharedSecret = 0;

  memset(keyClientToServer, 0, sizeof(keyClientToServer));
  memset(keyServerToClient, 0, sizeof(keyServerToClient));
  memset(ivClientToServer, 0, sizeof(ivClientToServer));
  memset(ivServerToClient, 0, sizeof(ivServerToClient));
}

void startDHHandshake() {
  serverPrivateKey = generatePrivateKey();
  serverPublicKey = modExp(DH_G, serverPrivateKey, DH_P);
  sendLine("DH_PUB:" + String(serverPublicKey));
  dhSent = true;
}

void tryCompleteHandshake() {
  if (dhSent && dhReceived && !sessionReady) {
    sharedSecret = modExp(clientPublicKey, serverPrivateKey, DH_P);
    deriveDirectionalMaterial(sharedSecret);
    sessionReady = true;
    sendLine("READY");
    Serial.println("[SERVER] Secure session ready.");
  }
}

void printReceivedControlData(const String& text) {
  int c1 = text.indexOf(',');
  int c2 = text.indexOf(',', c1 + 1);
  int c3 = text.indexOf(',', c2 + 1);
  int c4 = text.indexOf(',', c3 + 1);

  if (c1 < 0 || c2 < 0 || c3 < 0) {
    Serial.print("[SERVER] RX: ");
    Serial.println(text);
    return;
  }

  float roll = text.substring(0, c1).toFloat();
  float pitch = text.substring(c1 + 1, c2).toFloat();
  float yawCmd = text.substring(c2 + 1, c3).toFloat();
  float pitchCmd = text.substring(c3 + 1).toFloat();
  bool sprayActive = (c4 > 0) ? (text.substring(c4 + 1).toInt() == 1) : false;

  Serial.print("Roll: ");
  Serial.print(roll, 2);
  Serial.print("  Pitch: ");
  Serial.print(pitch, 2);
  Serial.print("  |  YawCmd: ");
  Serial.print(yawCmd, 2);
  Serial.print("  PitchCmd: ");
  Serial.print(pitchCmd, 2);
  Serial.print("  |  Spray: ");
  Serial.println(sprayActive ? "ON" : "OFF");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("HELLO:")) {
    Serial.print("[SERVER] Client: ");
    Serial.println(line.substring(6));
    if (!dhSent) startDHHandshake();
    return;
  }

  if (line.startsWith("DH_PUB:")) {
    clientPublicKey = (uint32_t)line.substring(7).toInt();
    dhReceived = true;
    tryCompleteHandshake();
    return;
  }

  if (line == "READY") {
    return;
  }

  if (line.startsWith("ENC:")) {
    if (!sessionReady) return;

    String plainText;
    if (decryptHexToText(line.substring(4), keyClientToServer, ivClientToServer, plainText)) {
      printReceivedControlData(plainText);
    } else {
      Serial.println("[SERVER] Decrypt failed.");
    }
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  randomSeed((uint32_t)micros());

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("[SERVER] Painting Robot Arm AP started.");
  Serial.print("[SERVER] SSID: ");
  Serial.println(ssid);
  Serial.print("[SERVER] IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  Serial.println("[SERVER] TCP server started on port 3333.");
}

void loop() {
  if (!activeClient || !activeClient.connected()) {
    if (activeClient) {
      activeClient.stop();
      Serial.println("[SERVER] Client disconnected.");
    }

    resetSession();
    activeClient = server.available();

    if (activeClient) {
      Serial.println("[SERVER] Client connected.");
    }
  }

  if (activeClient && activeClient.connected() && activeClient.available()) {
    String line = activeClient.readStringUntil('\n');
    handleLine(line);
  }

  delay(5);
}
