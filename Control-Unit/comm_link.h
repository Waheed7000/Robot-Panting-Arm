#ifndef COMM_LINK_H
#define COMM_LINK_H

#include <Arduino.h>
#include <WiFi.h>
#include <AESLib.h>

struct ControlPacketData {
  float roll;
  float pitch;
  float yawCmd;
  float pitchCmd;
};

class CommLink {
public:
  void begin();
  void update();
  bool isReady();
  bool sendControlData(const ControlPacketData& data);

private:
  void connectWiFi();
  void connectServer();
  void resetSession();
  void handleLine(String line);
  void sendLine(const String& line);
  void sendClientPublicKey();
  void tryCompleteHandshake();

  uint32_t modExp(uint32_t base, uint32_t exp, uint32_t mod);
  uint32_t generatePrivateKey();
  void deriveDirectionalMaterial(uint32_t secret);

  char nibbleToHex(uint8_t nibble);
  int hexCharToNibble(char c);
  String bytesToHexString(const byte* data, size_t len);
  bool hexStringToBytes(const String& hex, byte* out, size_t& outLen);
  bool encryptTextToHex(const String& plainText, const byte* key, const byte* ivBase, String& outHex);

private:
  WiFiClient client;
  AESLib aesLib;

  bool dhSent = false;
  bool dhReceived = false;
  bool sessionReady = false;

  uint32_t clientPrivateKey = 0;
  uint32_t clientPublicKey = 0;
  uint32_t serverPublicKey = 0;
  uint32_t sharedSecret = 0;

  byte keyClientToServer[16];
  byte keyServerToClient[16];
  byte ivClientToServer[16];
  byte ivServerToClient[16];

  uint32_t lastReconnectAttemptMs = 0;
};

#endif
