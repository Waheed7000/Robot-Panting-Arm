#include "comm_link.h"

static const char* WIFI_SSID = "Painting_Robot_Arm";
static const char* WIFI_PASSWORD = "12345678";
static const char* SERVER_IP = "192.168.4.1";
static const uint16_t SERVER_PORT = 3333;

static const uint32_t DH_P = 2147483647UL;
static const uint32_t DH_G = 5;

static const size_t MAX_PLAINTEXT_LEN = 96;
static const size_t MAX_CIPHER_BUF_LEN = 256;
static const uint32_t RECONNECT_INTERVAL_MS = 1000;

void CommLink::begin() {
  randomSeed((uint32_t)micros());
  connectWiFi();
  connectServer();
}

void CommLink::update() {
  if (WiFi.status() != WL_CONNECTED) {
    uint32_t now = millis();
    if (now - lastReconnectAttemptMs >= RECONNECT_INTERVAL_MS) {
      lastReconnectAttemptMs = now;
      connectWiFi();
    }
    return;
  }

  if (!client.connected()) {
    uint32_t now = millis();
    if (now - lastReconnectAttemptMs >= RECONNECT_INTERVAL_MS) {
      lastReconnectAttemptMs = now;
      client.stop();
      connectServer();
    }
    return;
  }

  while (client.available()) {
    String line = client.readStringUntil('\n');
    handleLine(line);
  }
}

bool CommLink::isReady() {
  return sessionReady && client.connected() && WiFi.status() == WL_CONNECTED;
}

bool CommLink::sendControlData(const ControlPacketData& data) {
  if (!isReady()) return false;

  String plain = String(data.roll, 2) + "," +
                 String(data.pitch, 2) + "," +
                 String(data.yawCmd, 2) + "," +
                 String(data.pitchCmd, 2) + "," +
                 String(data.sprayActive ? 1 : 0);;

  String cipherHex;
  if (!encryptTextToHex(plain, keyClientToServer, ivClientToServer, cipherHex)) {
    return false;
  }

  sendLine("ENC:" + cipherHex);
  return true;
}

void CommLink::connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[COMM] Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 5000) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[COMM] WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[COMM] WiFi connect failed.");
  }
}

void CommLink::connectServer() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) return;

  Serial.println("[COMM] Connecting server...");
  if (!client.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println("[COMM] Server connect failed.");
    return;
  }

  Serial.println("[COMM] Server connected.");
  resetSession();
  sendLine("HELLO:CONTROL_UNIT");
}

void CommLink::resetSession() {
  dhSent = false;
  dhReceived = false;
  sessionReady = false;

  clientPrivateKey = 0;
  clientPublicKey = 0;
  serverPublicKey = 0;
  sharedSecret = 0;

  memset(keyClientToServer, 0, sizeof(keyClientToServer));
  memset(keyServerToClient, 0, sizeof(keyServerToClient));
  memset(ivClientToServer, 0, sizeof(ivClientToServer));
  memset(ivServerToClient, 0, sizeof(ivServerToClient));
}

void CommLink::handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("DH_PUB:")) {
    serverPublicKey = (uint32_t)line.substring(7).toInt();
    dhReceived = true;

    if (!dhSent) {
      sendClientPublicKey();
    }

    tryCompleteHandshake();
    return;
  }

  if (line == "READY") {
    return;
  }
}

void CommLink::sendLine(const String& line) {
  if (client.connected()) {
    client.println(line);
  }
}

void CommLink::sendClientPublicKey() {
  clientPrivateKey = generatePrivateKey();
  clientPublicKey = modExp(DH_G, clientPrivateKey, DH_P);

  sendLine("DH_PUB:" + String(clientPublicKey));
  dhSent = true;
}

void CommLink::tryCompleteHandshake() {
  if (dhSent && dhReceived && !sessionReady) {
    sharedSecret = modExp(serverPublicKey, clientPrivateKey, DH_P);
    deriveDirectionalMaterial(sharedSecret);
    sessionReady = true;
    sendLine("READY");
    Serial.println("[COMM] Secure session ready.");
  }
}

uint32_t CommLink::modExp(uint32_t base, uint32_t exp, uint32_t mod) {
  uint64_t result = 1;
  uint64_t b = base % mod;

  while (exp > 0) {
    if (exp & 1) result = (result * b) % mod;
    b = (b * b) % mod;
    exp >>= 1;
  }

  return (uint32_t)result;
}

uint32_t CommLink::generatePrivateKey() {
  return (uint32_t)random(10000, 60000);
}

void CommLink::deriveDirectionalMaterial(uint32_t secret) {
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

char CommLink::nibbleToHex(uint8_t nibble) {
  return (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
}

int CommLink::hexCharToNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

String CommLink::bytesToHexString(const byte* data, size_t len) {
  String out;
  out.reserve(len * 2);

  for (size_t i = 0; i < len; i++) {
    out += nibbleToHex((data[i] >> 4) & 0x0F);
    out += nibbleToHex(data[i] & 0x0F);
  }

  return out;
}

bool CommLink::hexStringToBytes(const String& hex, byte* out, size_t& outLen) {
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

bool CommLink::encryptTextToHex(const String& plainText, const byte* key, const byte* ivBase, String& outHex) {
  if (plainText.length() + 1 > MAX_PLAINTEXT_LEN) return false;

  byte plainBuf[MAX_PLAINTEXT_LEN];
  memset(plainBuf, 0, sizeof(plainBuf));
  plainText.getBytes(plainBuf, plainText.length() + 1);

  byte cipherBuf[MAX_CIPHER_BUF_LEN];
  memset(cipherBuf, 0, sizeof(cipherBuf));

  byte ivWork[16];
  memcpy(ivWork, ivBase, 16);

  aesLib.set_paddingmode((paddingMode)0);
  uint16_t cipherLen = aesLib.encrypt(
    plainBuf,
    plainText.length() + 1,
    cipherBuf,
    (byte*)key,
    16,
    ivWork
  );

  outHex = bytesToHexString(cipherBuf, cipherLen);
  return true;
}
