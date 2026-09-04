/*
  ============================================================================
  NexulTech - Painel de 6 Leitos + Monitor de Bateria - ESP32
  ============================================================================

  FUNCIONALIDADES:
  - 6 leitos independentes no mesmo ESP32, cada um com:
      * 1 LED vermelho (chamada pendente, acionado via MQTT)
      * 1 botao com LED verde embutido (confirmacao da enfermagem)
  - Ciclo de cada leito (maquina de estados):
      IDLE -> chega MQTT "LeitoX/chamar"       -> acende vermelho (CHAMANDO)
      CHAMANDO -> 1a pressao do botao verde     -> apaga vermelho, acende
                  verde e inicia cronometro      (ATENDENDO)
      ATENDENDO -> 2a pressao do botao verde     -> apaga verde, publica no
                  MQTT o tempo (segundos) que o verde ficou aceso, volta
                  para IDLE
      IDLE -> pressao do botao (3a vez ou fora do ciclo) -> nao faz nada;
              o ciclo so reinicia com uma nova mensagem MQTT de chamada.
  - Nome de cada leito (ex. "Leito 01") e os topicos MQTT de entrada/saida
    de cada leito sao editaveis pela pagina web de configuracao.
  - Pagina web local de configuracao (SSID, senha, IP/porta MQTT, nomes e
    topicos por leito), com scan de redes WiFi (para ambientes tipo
    hospital/shopping/hotel).
  - Captive Portal assistido: se a rede WiFi do local exigir login numa
    pagina antes de liberar internet, o ESP32 sobe um AP proprio
    (NexulTech) para um operador conectar o celular, ser redirecionado
    para a pagina de login real e liberar a rede tambem para o proprio
    ESP32 (NAT best-effort entre AP e STA).
  - Monitor de bateria via ADC, com alerta MQTT quando a carga atingir 80%+.
  - Configuracoes salvas na memoria NVS (sobrevivem a reinicio/queda de
    energia).
  - Se nao houver WiFi configurado ou a conexao falhar, o dispositivo sobe
    um Access Point proprio (SSID "NexulTech", senha "nexultech") para
    voce acessar a pagina de configuracao em 192.168.4.1

  BIBLIOTECAS NECESSARIAS:
  - PubSubClient (Nick O'Leary)
  - WiFi, WebServer, Preferences -> ja vem no pacote da placa ESP32

  PINAGEM (6 leitos = 18 GPIOs, ver tabela completa abaixo):
  - Leito 1: botao=GPIO34  led verde=GPIO4   led vermelho=GPIO5
  - Leito 2: botao=GPIO36  led verde=GPIO13  led vermelho=GPIO14
  - Leito 3: botao=GPIO39  led verde=GPIO18  led vermelho=GPIO19
  - Leito 4: botao=GPIO33  led verde=GPIO21  led vermelho=GPIO22
  - Leito 5: botao=GPIO32  led verde=GPIO23  led vermelho=GPIO25
  - Leito 6: botao=GPIO27  led verde=GPIO26  led vermelho=GPIO16 (esp32dev)
                                                          GPIO12 (wemos_d1_r32)
  - Leitura bateria ..... GPIO35 (ADC, entrada de um divisor resistivo)
  - LED status .......... GPIO2  (acende fixo quando conectado ao WiFi)

  OBS. sobre o GPIO12 no Wemos D1 R32 (leito 6, so nesse board): e um pino
  de "strapping" (MTDI) que o ESP32 le no boot para decidir a tensao da
  flash. So e usado no board wemos_d1_r32 porque nele o 16/17 esta
  reservado pra PSRAM e faltou 1 pino para fechar os 18 necessarios. O
  firmware NUNCA escreve nele antes do fim do boot (Arduino ja inicializa
  os pinos depois disso), entao na pratica nao ha conflito -- mas se um dia
  precisar recolocar um resistor de pull nesse pino fisicamente, cuidado
  para nao forcar HIGH durante o boot.
  ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiUdp.h>

// NAT (Network Address Translation) entre a interface AP (NexulTech) e a
// interface STA (rede do hospital/shopping). Presente no core arduino-esp32
// (esp-lwip) atual; se o seu core for muito antigo, atualize-o.
#include "lwip/lwip_napt.h"
#include "lwip/dns.h"
// Necessario para LOCK_TCPIP_CORE()/UNLOCK_TCPIP_CORE(): chamadas "cruas"
// do lwIP (fora do Arduino/WiFi.h), como ip_napt_enable(), exigem esse lock.
#include "lwip/tcpip.h"

// ---------------- NUMERO DE LEITOS ----------------
#define NUM_LEITOS 6

// ---------------- PINOS POR BOARD ----------------
#if defined(NEXULTECH_BOARD_WEMOS_D1_R32)
  #define LEITO6_LED_RED_PIN 12   // Wemos D1 R32: 16/17 reservados p/ PSRAM
#else
  #define LEITO6_LED_RED_PIN 16   // esp32dev (WROOM, sem PSRAM)
#endif

const uint8_t LEITO_BTN_PIN[NUM_LEITOS]       = { 34, 36, 39, 33, 32, 27 };
const uint8_t LEITO_LED_GREEN_PIN[NUM_LEITOS] = { 4,  13, 18, 21, 23, 26 };
const uint8_t LEITO_LED_RED_PIN[NUM_LEITOS]   = { 5,  14, 19, 22, 25, LEITO6_LED_RED_PIN };

// 34, 36 e 39 sao entrada-somente (nao precisam de INPUT_PULLUP por
// hardware -- ver observacao em setup() sobre resistor de pull externo).
const bool LEITO_BTN_INPUT_ONLY[NUM_LEITOS] = { true, true, true, false, false, false };

#define BATTERY_ADC_PIN 35
#define LED_STATUS_PIN  2

// ---------------- CALIBRACAO DE BATERIA ----------------
#define BATTERY_MIN_V         3.0    // tensao considerada 0%
#define BATTERY_MAX_V         4.2    // tensao considerada 100%
#define ADC_REF_V             3.3
#define ADC_RESOLUTION        4095.0
#define VOLTAGE_DIVIDER_RATIO 2.0    // ex: dois resistores iguais = divide por 2

// Limite de alerta de bateria "carregada"
#define BATTERY_ALERT_HIGH    80.0
#define BATTERY_ALERT_RESET   75.0   // histerese p/ nao repetir o alerta o tempo todo

// ---------------- OBJETOS GLOBAIS ----------------
Preferences prefs;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ---------------- CONFIGURACOES (carregadas da NVS) ----------------
String cfgSSID;
String cfgPASS;
String cfgMqttServer;
int    cfgMqttPort;
String cfgMqttUser;
String cfgMqttPass;
String cfgBaseTopic;     // usado para telemetria geral (bateria/status)
String cfgAdminUser;
String cfgAdminPass;

String cfgLeitoName[NUM_LEITOS];      // ex: "Leito 01" (aparece no payload)
String cfgLeitoTopicChamar[NUM_LEITOS]; // topico MQTT de ENTRADA (aciona o vermelho)
String cfgLeitoTopicTempo[NUM_LEITOS];  // topico MQTT de SAIDA (tempo decorrido)
String cfgLeitoTopicEstado[NUM_LEITOS]; // topico MQTT de SAIDA (estado ao vivo p/ dashboard)

// ---------------- ESTADO POR LEITO ----------------
enum LeitoState { LEITO_IDLE, LEITO_CHAMANDO, LEITO_ATENDENDO };
LeitoState leitoState[NUM_LEITOS];
unsigned long leitoAtendStart[NUM_LEITOS]; // millis() de quando o verde acendeu
unsigned long lastBtnPress[NUM_LEITOS];
const unsigned long DEBOUNCE_MS = 400;

bool apMode = false;

unsigned long lastBatteryCheck = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 60000; // le bateria a cada 1 min
bool battery80Sent = false;
float lastBatteryPercent = 0;
float lastBatteryVoltage = 0;

unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL = 5000;

// ---------------- VALORES PADRAO (1a vez que liga) ----------------
const char* DEFAULT_MQTT_SERVER = "iot.nexultech.com.br";
const int   DEFAULT_MQTT_PORT   = 2883;
const char* DEFAULT_BASE_TOPIC  = "hospital/painel_leitos01";
const char* AP_SSID = "NexulTech";
const char* AP_PASS = "nexultech";
const char* DEFAULT_ADMIN_USER = "admin";
const char* DEFAULT_ADMIN_PASS = "nexultech"; // ALTERE isso assim que configurar o dispositivo

// ---------------- CAPTIVE PORTAL ASSISTIDO (qualquer rede -- hospital, ----
// ---------------- shopping, hotel, aeroporto, qualquer ISP) --------------
// Mesmo esquema ja validado no projeto LEITO_V1_2: mini-proxy de DNS
// seletivo (so os dominios de deteccao de captive portal do proprio SO
// recebem o IP do ESP32) + NAT best-effort entre AP e STA.
WiFiUDP dnsUdp;
const uint16_t DNS_PORT = 53;
bool assistPortalActive = false;
bool natEnabled = false;

const char* CAPTIVE_TRIGGER_HOSTS[] = {
  "connectivitycheck.gstatic.com",
  "connectivitycheck.android.com",
  "clients3.google.com",
  "clients.l.google.com",
  "captive.apple.com",
  "www.apple.com",
  "www.msftconnecttest.com",
  "www.msftncsi.com",
  "detectportal.firefox.com",
};
const int CAPTIVE_TRIGGER_HOSTS_COUNT = sizeof(CAPTIVE_TRIGGER_HOSTS) / sizeof(CAPTIVE_TRIGGER_HOSTS[0]);

String portalRedirectURL = "";

unsigned long lastPortalProbe = 0;
const unsigned long PORTAL_PROBE_INTERVAL = 20000;
const char* CAPTIVE_CHECK_HOST = "connectivitycheck.gstatic.com";
const char* CAPTIVE_CHECK_PATH = "/generate_204";

// Se der problema de novo: mude para "false" para testar so o auto-popup
// (DNS) sem tentar habilitar o NAT. NAT ficou desligado por padrao no
// LEITO_V1_2 por um bug conhecido de reboot ao desconectar um cliente do
// AP -- mantido desligado aqui pelo mesmo motivo, ate resolvermos.
const bool ENABLE_NAT = false;

const unsigned long WIFI_SETTLE_MS = 8000;
unsigned long staConnectedSince = 0;
bool staWasConnected = false;

// =====================================================================
// PROTOTIPOS
// =====================================================================
void loadConfig();
void saveConfig();
void connectWiFi();
void startAPMode();
void setupWebServer();
void handleRoot();
void handleConfig();
void handleSave();
void handleStatus();
bool checkAdminAuth();
void mqttReconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishMessage(const String &topic, const String &payload);
void publishLeitoState(int i, const char* stateStr);
void subscribeLeitoTopics();
void checkLeitoButtons();
void checkBattery();
float readBatteryVoltage();
float voltageToPercent(float v);
String htmlEscape(const String &in);
String scanNetworksJSON();
void handleScan();
bool probeCaptivePortal(String &redirectUrl);
void enableAssistPortalAP();
void disableAssistPortalAP();
void enableNAT();
void manageCaptivePortal();
void handleNotFound();
void redirectToGatewayPortal();
bool isRequestFromAP();
void handleDnsRequests();
void sendDnsAnswer(uint8_t *query, int queryLen, IPAddress destIp, uint16_t destPort, IPAddress answerIp);
void forwardDnsQuery(uint8_t *query, int queryLen, IPAddress clientIp, uint16_t clientPort);
bool extractDnsQuestionName(uint8_t *buf, int len, String &qname);
int leitoIndexFromTopic(const String &topic);
void setLeitoLeds(int i);

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < NUM_LEITOS; i++) {
    // Botoes 34/36/39 sao entrada-somente e NAO tem pull-up interno no
    // silicio do ESP32 -- por isso o circuito precisa de um resistor de
    // pull-up externo (10k para 3V3) nesses 3 leitos. Nos demais (33,32,27)
    // o INPUT_PULLUP interno resolve sozinho.
    if (LEITO_BTN_INPUT_ONLY[i]) {
      pinMode(LEITO_BTN_PIN[i], INPUT);
    } else {
      pinMode(LEITO_BTN_PIN[i], INPUT_PULLUP);
    }
    pinMode(LEITO_LED_GREEN_PIN[i], OUTPUT);
    pinMode(LEITO_LED_RED_PIN[i], OUTPUT);
    digitalWrite(LEITO_LED_GREEN_PIN[i], LOW);
    digitalWrite(LEITO_LED_RED_PIN[i], LOW);

    leitoState[i] = LEITO_IDLE;
    leitoAtendStart[i] = 0;
    lastBtnPress[i] = 0;
  }

  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);

  analogReadResolution(12); // 0-4095
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db); // permite ler ate ~3.3V no pino do ADC

  loadConfig();
  connectWiFi();
  setupWebServer();

  mqttClient.setServer(cfgMqttServer.c_str(), cfgMqttPort);
  mqttClient.setBufferSize(640);
  mqttClient.setCallback(mqttCallback);

  // IMPORTANTE: a checagem de Captive Portal NAO e feita aqui no setup().
  // Subir o AP (NexulTech) e abrir uma conexao TCP quase ao mesmo tempo,
  // logo na inicializacao, e uma causa comum de crash/reboot no ESP32.
  // Por isso ela so roda dentro do loop(), depois que o WiFi ja esta
  // estavel ha alguns segundos (veja manageCaptivePortal()).

  Serial.println("Sistema NexulTech - Painel de 6 Leitos.");
}

void loop() {
  server.handleClient();

  if (assistPortalActive) {
    handleDnsRequests();
  }

  if (!apMode) {
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED_STATUS_PIN, LOW);
      staWasConnected = false;
      connectWiFi();
    } else {
      digitalWrite(LED_STATUS_PIN, HIGH);
      manageCaptivePortal();
      if (!mqttClient.connected()) {
        mqttReconnect();
      }
      mqttClient.loop();
    }
  }

  checkLeitoButtons();
  checkBattery();
}

// =====================================================================
// CONFIG - NVS (memoria nao volatil)
// =====================================================================
void loadConfig() {
  prefs.begin("config", false);
  cfgSSID       = prefs.getString("ssid", "");
  cfgPASS       = prefs.getString("pass", "");
  cfgMqttServer = prefs.getString("mqttsrv", DEFAULT_MQTT_SERVER);
  cfgMqttPort   = prefs.getInt("mqttport", DEFAULT_MQTT_PORT);
  cfgMqttUser   = prefs.getString("mqttuser", "");
  cfgMqttPass   = prefs.getString("mqttpass", "");
  cfgBaseTopic  = prefs.getString("basetopic", DEFAULT_BASE_TOPIC);
  cfgAdminUser  = prefs.getString("adminuser", DEFAULT_ADMIN_USER);
  cfgAdminPass  = prefs.getString("adminpass", DEFAULT_ADMIN_PASS);

  for (int i = 0; i < NUM_LEITOS; i++) {
    String idx = String(i + 1);
    String defName   = "Leito 0" + idx; // "Leito 01".."Leito 06"
    String defChamar = "Leito" + idx + "/chamar";
    String defTempo  = "Leito" + idx + "/tempo";
    String defEstado = "Leito" + idx + "/estado";

    cfgLeitoName[i]        = prefs.getString(("lname" + idx).c_str(), defName);
    cfgLeitoTopicChamar[i] = prefs.getString(("ltopc" + idx).c_str(), defChamar);
    cfgLeitoTopicTempo[i]  = prefs.getString(("ltopt" + idx).c_str(), defTempo);
    cfgLeitoTopicEstado[i] = prefs.getString(("ltope" + idx).c_str(), defEstado);
  }

  prefs.end();
}

void saveConfig() {
  prefs.begin("config", false);
  prefs.putString("ssid", cfgSSID);
  prefs.putString("pass", cfgPASS);
  prefs.putString("mqttsrv", cfgMqttServer);
  prefs.putInt("mqttport", cfgMqttPort);
  prefs.putString("mqttuser", cfgMqttUser);
  prefs.putString("mqttpass", cfgMqttPass);
  prefs.putString("basetopic", cfgBaseTopic);
  prefs.putString("adminuser", cfgAdminUser);
  prefs.putString("adminpass", cfgAdminPass);

  for (int i = 0; i < NUM_LEITOS; i++) {
    String idx = String(i + 1);
    prefs.putString(("lname" + idx).c_str(), cfgLeitoName[i]);
    prefs.putString(("ltopc" + idx).c_str(), cfgLeitoTopicChamar[i]);
    prefs.putString(("ltopt" + idx).c_str(), cfgLeitoTopicTempo[i]);
    prefs.putString(("ltope" + idx).c_str(), cfgLeitoTopicEstado[i]);
  }

  prefs.end();
}

// =====================================================================
// WIFI
// =====================================================================
void connectWiFi() {
  if (cfgSSID.length() == 0) {
    startAPMode();
    return;
  }

  // Redes publicas abertas (hospital/shopping) costumam ter muitos
  // aparelhos conectados e podem demorar mais para associar/pegar IP.
  const int MAX_ATTEMPTS = 2;
  const unsigned long CONNECT_TIMEOUT_MS = 20000;

  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    Serial.printf("Conectando ao WiFi: %s (tentativa %d/%d)\n", cfgSSID.c_str(), attempt, MAX_ATTEMPTS);
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSSID.c_str(), cfgPASS.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < CONNECT_TIMEOUT_MS) {
      delay(300);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      apMode = false;
      Serial.println();
      Serial.print("WiFi conectado. IP: ");
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println("\nTentativa falhou.");
    WiFi.disconnect(true);
    delay(500);
  }

  Serial.println("Falha ao conectar apos todas as tentativas. Iniciando modo AP de configuracao.");
  startAPMode();
}

void startAPMode() {
  apMode = true;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA); // AP_STA permite escanear redes vizinhas mesmo em modo de configuracao
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Modo AP ativo. Conecte-se a rede '");
  Serial.print(AP_SSID);
  Serial.print("' e acesse: ");
  Serial.println(WiFi.softAPIP());
}

// =====================================================================
// CAPTIVE PORTAL ASSISTIDO (redes abertas de hospital/shopping/hotel)
// =====================================================================
bool probeCaptivePortal(String &redirectUrl) {
  redirectUrl = "";
  WiFiClient client;
  client.setTimeout(4000);

  if (!client.connect(CAPTIVE_CHECK_HOST, 80)) {
    return true;
  }

  client.print(String("GET ") + CAPTIVE_CHECK_PATH + " HTTP/1.1\r\n" +
               "Host: " + CAPTIVE_CHECK_HOST + "\r\n" +
               "Connection: close\r\n\r\n");

  unsigned long start = millis();
  while (client.connected() && !client.available() && millis() - start < 4000) {
    delay(10);
  }

  String response;
  while (client.available()) {
    response += (char)client.read();
  }
  client.stop();

  if (response.startsWith("HTTP/1.1 204") || response.startsWith("HTTP/1.0 204")) {
    return false; // internet livre, sem portal
  }

  int locIdx = response.indexOf("Location: ");
  if (locIdx >= 0) {
    int end = response.indexOf("\r\n", locIdx);
    if (end > locIdx) {
      redirectUrl = response.substring(locIdx + 10, end);
      redirectUrl.trim();
    }
  }

  return true;
}

void enableAssistPortalAP() {
  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
    delay(100);
  }

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (!apOk) {
    Serial.println("Falha ao subir o AP NexulTech para autenticacao assistida.");
    return;
  }

  IPAddress apIp = WiFi.softAPIP();
  unsigned long waitStart = millis();
  while (apIp == IPAddress(0, 0, 0, 0) && millis() - waitStart < 2000) {
    delay(50);
    apIp = WiFi.softAPIP();
  }
  if (apIp == IPAddress(0, 0, 0, 0)) {
    Serial.println("AP NexulTech sem IP valido, abortando autenticacao assistida.");
    return;
  }

  dnsUdp.begin(DNS_PORT);

  if (ENABLE_NAT) {
    enableNAT();
  }

  assistPortalActive = true;
  Serial.print("NexulTech ativo para autenticacao assistida. IP: ");
  Serial.println(apIp);
}

void disableAssistPortalAP() {
  dnsUdp.stop();
  assistPortalActive = false;
  portalRedirectURL = "";
  Serial.println("Rede STA liberada. Encerrando modo de autenticacao assistida.");
}

void enableNAT() {
  if (natEnabled) return;
  LOCK_TCPIP_CORE();
  ip_napt_enable(WiFi.softAPIP(), 1);
  UNLOCK_TCPIP_CORE();
  natEnabled = true;
  Serial.println("NAT habilitado entre NexulTech (AP) e a rede STA.");
}

void manageCaptivePortal() {
  unsigned long now = millis();

  if (!staWasConnected) {
    staWasConnected = true;
    staConnectedSince = now;
    return;
  }

  if (now - staConnectedSince < WIFI_SETTLE_MS) return;

  if (now - lastPortalProbe < PORTAL_PROBE_INTERVAL) return;
  lastPortalProbe = now;

  String redirect;
  bool blocked = probeCaptivePortal(redirect);
  if (redirect.length() > 0) {
    portalRedirectURL = redirect;
    Serial.print("URL de login capturada: ");
    Serial.println(portalRedirectURL);
  }

  if (blocked && !assistPortalActive) {
    Serial.println("Captive Portal detectado. Ativando NexulTech para autenticacao assistida.");
    enableAssistPortalAP();
  } else if (!blocked && assistPortalActive) {
    Serial.println("Internet liberada na rede STA!");
    disableAssistPortalAP();
  }
}

// =====================================================================
// MINI-PROXY DE DNS (seletivo -- funciona com qualquer captive portal)
// =====================================================================
bool extractDnsQuestionName(uint8_t *buf, int len, String &qname) {
  qname = "";
  int pos = 12;
  while (pos < len && buf[pos] != 0) {
    int labelLen = buf[pos];
    if (labelLen < 0 || labelLen > 63) return false;
    pos++;
    if (pos + labelLen > len) return false;
    if (qname.length() > 0) qname += ".";
    for (int i = 0; i < labelLen; i++) {
      qname += (char)buf[pos + i];
    }
    pos += labelLen;
  }
  return pos < len;
}

void sendDnsAnswer(uint8_t *query, int queryLen, IPAddress destIp, uint16_t destPort, IPAddress answerIp) {
  if (queryLen < 12 || queryLen > 480) return;

  uint8_t response[512];
  memcpy(response, query, queryLen);
  int len = queryLen;

  response[2] = 0x81;
  response[3] = 0x80;
  response[6] = 0x00; response[7] = 0x01;
  response[8] = 0x00; response[9] = 0x00;
  response[10] = 0x00; response[11] = 0x00;

  response[len++] = 0xC0; response[len++] = 0x0C;
  response[len++] = 0x00; response[len++] = 0x01;
  response[len++] = 0x00; response[len++] = 0x01;
  response[len++] = 0x00; response[len++] = 0x00; response[len++] = 0x00; response[len++] = 0x1E;
  response[len++] = 0x00; response[len++] = 0x04;
  response[len++] = answerIp[0];
  response[len++] = answerIp[1];
  response[len++] = answerIp[2];
  response[len++] = answerIp[3];

  dnsUdp.beginPacket(destIp, destPort);
  dnsUdp.write(response, len);
  dnsUdp.endPacket();
}

void forwardDnsQuery(uint8_t *query, int queryLen, IPAddress clientIp, uint16_t clientPort) {
  IPAddress realDns = WiFi.dnsIP();
  if (realDns == IPAddress(0, 0, 0, 0)) return;

  WiFiUDP fwd;
  fwd.begin(0);
  fwd.beginPacket(realDns, 53);
  fwd.write(query, queryLen);
  fwd.endPacket();

  uint8_t respBuf[512];
  int respLen = 0;
  unsigned long start = millis();
  while (millis() - start < 1500) {
    int packetSize = fwd.parsePacket();
    if (packetSize > 0) {
      respLen = fwd.read(respBuf, sizeof(respBuf));
      break;
    }
    delay(5);
  }
  fwd.stop();

  if (respLen > 0) {
    dnsUdp.beginPacket(clientIp, clientPort);
    dnsUdp.write(respBuf, respLen);
    dnsUdp.endPacket();
  }
}

void handleDnsRequests() {
  int packetSize = dnsUdp.parsePacket();
  if (packetSize <= 0) return;

  uint8_t buf[512];
  int len = dnsUdp.read(buf, sizeof(buf));
  if (len < 12) return;

  IPAddress clientIp = dnsUdp.remoteIP();
  uint16_t clientPort = dnsUdp.remotePort();

  String qname;
  bool ok = extractDnsQuestionName(buf, len, qname);

  bool isTrigger = false;
  if (ok) {
    for (int i = 0; i < CAPTIVE_TRIGGER_HOSTS_COUNT; i++) {
      if (qname.equalsIgnoreCase(CAPTIVE_TRIGGER_HOSTS[i])) {
        isTrigger = true;
        break;
      }
    }
  }

  if (isTrigger) {
    sendDnsAnswer(buf, len, clientIp, clientPort, WiFi.softAPIP());
  } else {
    forwardDnsQuery(buf, len, clientIp, clientPort);
  }
}

// =====================================================================
// MQTT
// =====================================================================
int leitoIndexFromTopic(const String &topic) {
  for (int i = 0; i < NUM_LEITOS; i++) {
    if (topic.equals(cfgLeitoTopicChamar[i])) return i;
  }
  return -1;
}

// Acende o vermelho e liga o ciclo de um leito ao chegar mensagem MQTT no
// topico de chamada dele. So tem efeito se o leito estiver em IDLE (uma
// nova chamada durante CHAMANDO/ATENDENDO e ignorada ate o ciclo fechar).
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  int i = leitoIndexFromTopic(topicStr);
  if (i < 0) return;

  Serial.printf("MQTT recebido em [%s] -> %s\n", topic, cfgLeitoName[i].c_str());

  if (leitoState[i] == LEITO_IDLE) {
    leitoState[i] = LEITO_CHAMANDO;
    digitalWrite(LEITO_LED_RED_PIN[i], HIGH);
    digitalWrite(LEITO_LED_GREEN_PIN[i], LOW);
    publishLeitoState(i, "Chamando");
    Serial.printf("%s: chamada recebida, LED vermelho aceso.\n", cfgLeitoName[i].c_str());
  } else {
    Serial.printf("%s: chamada ignorada (ciclo ja em andamento).\n", cfgLeitoName[i].c_str());
  }
}

void subscribeLeitoTopics() {
  for (int i = 0; i < NUM_LEITOS; i++) {
    mqttClient.subscribe(cfgLeitoTopicChamar[i].c_str());
    // So publica "Livre" retido se o leito realmente estiver livre agora --
    // evita sobrescrever o estado real de um leito em atendimento no meio
    // de uma reconexao de rede.
    if (leitoState[i] == LEITO_IDLE) {
      publishLeitoState(i, "Livre");
    }
  }
}

void mqttReconnect() {
  if (millis() - lastMqttAttempt < MQTT_RETRY_INTERVAL) return;
  lastMqttAttempt = millis();

  mqttClient.setServer(cfgMqttServer.c_str(), cfgMqttPort);

  String clientId = "ESP32-NexulTech-Leitos-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  bool ok;
  if (cfgMqttUser.length() > 0) {
    ok = mqttClient.connect(clientId.c_str(), cfgMqttUser.c_str(), cfgMqttPass.c_str());
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }

  if (ok) {
    Serial.println("MQTT conectado.");
    subscribeLeitoTopics();
  } else {
    Serial.printf("Falha MQTT, rc=%d (tentando de novo em %lus)\n",
                   mqttClient.state(), MQTT_RETRY_INTERVAL / 1000);
  }
}

void publishMessage(const String &topic, const String &payload) {
  if (apMode) return;
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  if (mqttClient.connected()) {
    bool sent = mqttClient.publish(topic.c_str(), payload.c_str());
    Serial.printf("MQTT [%s]: %s -> %s\n", topic.c_str(), payload.c_str(),
                  sent ? "OK" : "FALHOU");
  } else {
    Serial.println("MQTT indisponivel, mensagem nao enviada.");
  }
}

// Publica o estado ao vivo do leito (Livre/Chamando/Atendendo) com
// retain=true -- assim um dashboard que conectar depois ja recebe o
// ultimo estado conhecido, sem precisar esperar a proxima mudanca.
void publishLeitoState(int i, const char* stateStr) {
  if (apMode) return;
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  if (mqttClient.connected()) {
    mqttClient.publish(cfgLeitoTopicEstado[i].c_str(), stateStr, true);
  }
}

// =====================================================================
// BOTOES / MAQUINA DE ESTADOS DOS LEITOS
// =====================================================================
void checkLeitoButtons() {
  unsigned long now = millis();

  for (int i = 0; i < NUM_LEITOS; i++) {
    bool pressed = (digitalRead(LEITO_BTN_PIN[i]) == LOW);
    if (!pressed) continue;
    if (now - lastBtnPress[i] <= DEBOUNCE_MS) continue;
    lastBtnPress[i] = now;

    switch (leitoState[i]) {
      case LEITO_CHAMANDO:
        // 1a pressao: apaga vermelho, acende verde, inicia cronometro
        digitalWrite(LEITO_LED_RED_PIN[i], LOW);
        digitalWrite(LEITO_LED_GREEN_PIN[i], HIGH);
        leitoAtendStart[i] = now;
        leitoState[i] = LEITO_ATENDENDO;
        publishLeitoState(i, "Atendendo");
        Serial.printf("%s: atendimento iniciado, LED verde aceso.\n", cfgLeitoName[i].c_str());
        break;

      case LEITO_ATENDENDO: {
        // 2a pressao: apaga verde, publica o tempo decorrido, volta a IDLE
        digitalWrite(LEITO_LED_GREEN_PIN[i], LOW);
        unsigned long elapsedSec = (now - leitoAtendStart[i]) / 1000UL;
        leitoState[i] = LEITO_IDLE;
        publishLeitoState(i, "Livre");

        String payload = "{\"leito\":\"" + cfgLeitoName[i] +
                          "\",\"tempo_segundos\":" + String(elapsedSec) + "}";
        publishMessage(cfgLeitoTopicTempo[i], payload);
        Serial.printf("%s: atendimento concluido em %lus.\n", cfgLeitoName[i].c_str(), elapsedSec);
        break;
      }

      case LEITO_IDLE:
      default:
        // Pressao fora do ciclo (3a vez ou sem chamada pendente): ignora.
        // O ciclo so reinicia com uma nova mensagem MQTT de chamada.
        break;
    }
  }
}

// =====================================================================
// BATERIA
// =====================================================================
float readBatteryVoltage() {
  const int NUM_SAMPLES = 16;
  uint32_t total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += analogRead(BATTERY_ADC_PIN);
    delay(2);
  }
  float rawAvg = total / (float)NUM_SAMPLES;

  float vAdc = (rawAvg / ADC_RESOLUTION) * ADC_REF_V;
  float vBat = vAdc * VOLTAGE_DIVIDER_RATIO;

  // Debug: se vBat continuar em 0.00V mesmo com bateria conectada, o
  // problema e de hardware/fiacao. Verifique: divisor ligado ao GPIO35,
  // GND compartilhado com o ESP32, e a tensao real no pino (<=3.3V, nunca
  // a tensao total da bateria sem passar pelo divisor).
  Serial.printf("[Bateria] ADC bruto=%.0f  Vadc=%.3fV  Vbat=%.3fV\n", rawAvg, vAdc, vBat);

  return vBat;
}

float voltageToPercent(float v) {
  float pct = (v - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0;
  if (pct > 100) pct = 100;
  if (pct < 0) pct = 0;
  return pct;
}

void checkBattery() {
  unsigned long now = millis();
  if (now - lastBatteryCheck < BATTERY_CHECK_INTERVAL) return;
  lastBatteryCheck = now;

  lastBatteryVoltage = readBatteryVoltage();
  lastBatteryPercent = voltageToPercent(lastBatteryVoltage);

  Serial.printf("Bateria: %.2fV (%.1f%%)\n", lastBatteryVoltage, lastBatteryPercent);

  String payload = "{\"voltage\":" + String(lastBatteryVoltage, 2) +
                    ",\"percent\":" + String(lastBatteryPercent, 1) + "}";
  publishMessage(cfgBaseTopic + "/bateria", payload);

  if (lastBatteryPercent >= BATTERY_ALERT_HIGH && !battery80Sent) {
    battery80Sent = true;
    publishMessage(cfgBaseTopic + "/bateria/alerta",
                    "Bateria com 80% ou mais de carga");
  } else if (lastBatteryPercent < BATTERY_ALERT_RESET) {
    battery80Sent = false;
  }
}

// =====================================================================
// AUTENTICACAO DA AREA DE CONFIGURACAO (somente admin)
// =====================================================================
bool checkAdminAuth() {
  if (!server.authenticate(cfgAdminUser.c_str(), cfgAdminPass.c_str())) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

// =====================================================================
// ESCANEAMENTO DE REDES WIFI
// =====================================================================
String scanNetworksJSON() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(false);
    delay(100);
  }

  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    bool secure = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
            ",\"secure\":" + String(secure ? "true" : "false") + "}";
  }
  json += "]";
  WiFi.scanDelete();
  return json;
}

void handleScan() {
  if (!checkAdminAuth()) return;
  String json = scanNetworksJSON();
  server.send(200, "application/json", json);
}

// =====================================================================
// WEB SERVER - PAGINA LOCAL DE CONFIGURACAO
// =====================================================================
bool isRequestFromAP() {
  IPAddress clientIp = server.client().remoteIP();
  IPAddress apIp = WiFi.softAPIP();
  return clientIp[0] == apIp[0] && clientIp[1] == apIp[1] && clientIp[2] == apIp[2];
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", handleStatus);
  server.on("/scan", handleScan);
  server.onNotFound(handleNotFound);
  server.begin();
}

void handleNotFound() {
  if (assistPortalActive) {
    redirectToGatewayPortal();
    return;
  }

  server.send(404, "text/plain", "Nao encontrado");
}

void redirectToGatewayPortal() {
  if (portalRedirectURL.length() > 0) {
    server.sendHeader("Location", portalRedirectURL, true);
    server.send(302, "text/plain", "");
    return;
  }

  IPAddress gw = WiFi.gatewayIP();
  if (gw == IPAddress(0, 0, 0, 0)) {
    server.send(200, "text/html",
      "<html><body><h3>Aguardando rede...</h3>"
      "<script>setTimeout(function(){location.reload();},3000);</script>"
      "</body></html>");
    return;
  }
  String target = "http://" + gw.toString() + "/";
  server.sendHeader("Location", target, true);
  server.send(302, "text/plain", "");
}

String htmlEscape(const String &in) {
  String out = in;
  out.replace("&", "&amp;");
  out.replace("\"", "&quot;");
  out.replace("<", "&lt;");
  out.replace(">", "&gt;");
  return out;
}

const char* leitoStateLabel(LeitoState s) {
  switch (s) {
    case LEITO_CHAMANDO:  return "Chamando (vermelho)";
    case LEITO_ATENDENDO: return "Atendendo (verde)";
    default:               return "Livre";
  }
}

const char* leitoStateBadgeClass(LeitoState s) {
  switch (s) {
    case LEITO_CHAMANDO:  return "bad";
    case LEITO_ATENDENDO: return "warn";
    default:               return "ok";
  }
}

void handleRoot() {
  if (assistPortalActive && isRequestFromAP()) {
    redirectToGatewayPortal();
    return;
  }

  String ip = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String mqttStatus = mqttClient.connected() ? "Conectado" : "Desconectado";

  String html = "<!DOCTYPE html><html lang='pt-br'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>NexulTech - Painel de Leitos</title><style>";
  html += "body{font-family:Arial,sans-serif;background:#f4f6f8;margin:0;padding:20px;color:#222}";
  html += ".card{background:#fff;border-radius:8px;padding:20px;max-width:520px;margin:0 auto 16px;box-shadow:0 2px 6px rgba(0,0,0,.1)}";
  html += "h2{margin-top:0;color:#0a5c8a}";
  html += "button{margin-top:16px;padding:10px 16px;background:#0a5c8a;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:15px}";
  html += "button:hover{background:#084a70}";
  html += ".adminbtn{background:#6c757d}";
  html += ".status{font-size:13px;color:#555;margin-bottom:8px}";
  html += ".badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;color:#fff}";
  html += ".ok{background:#2a9d8f}.bad{background:#e76f51}.warn{background:#e9c46a;color:#333}";
  html += ".leitorow{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid #eee}";
  html += ".leitorow:last-child{border-bottom:none}";
  html += "</style></head><body>";

  html += "<div class='card'><h2>Status</h2>";
  html += "<div class='status'>IP do dispositivo: " + ip + "</div>";
  html += "<div class='status'>WiFi: <span class='badge " + String(apMode ? "bad" : "ok") +
          "'>" + String(apMode ? "Modo AP (configuracao)" : "Conectado") + "</span></div>";
  html += "<div class='status'>MQTT: <span class='badge " + String(mqttClient.connected() ? "ok" : "bad") +
          "'>" + mqttStatus + "</span></div>";
  if (assistPortalActive) {
    html += "<div class='status'>Captive Portal: <span class='badge bad'>Aguardando autenticacao</span> "
            "-- conecte um celular no WiFi '" + String(AP_SSID) + "' para liberar a rede</div>";
  }
  html += "<div class='status'>Bateria: " + String(lastBatteryPercent, 1) + "% (" +
          String(lastBatteryVoltage, 2) + "V)</div>";
  html += "</div>";

  html += "<div class='card'><h2>Leitos</h2>";
  for (int i = 0; i < NUM_LEITOS; i++) {
    html += "<div class='leitorow'><span>" + htmlEscape(cfgLeitoName[i]) + "</span>";
    html += "<span class='badge " + String(leitoStateBadgeClass(leitoState[i])) + "'>" +
            String(leitoStateLabel(leitoState[i])) + "</span></div>";
  }
  html += "</div>";

  html += "<div class='card'><h2>Area restrita</h2>";
  html += "<div class='status'>As configuracoes de WiFi, MQTT e dos leitos ficam protegidas por login de administrador.</div>";
  html += "<a href='/config'><button class='adminbtn'>Configuracoes (admin)</button></a>";
  html += "</div>";

  html += "<script>setTimeout(function(){location.reload();}, 8000);</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleConfig() {
  if (!checkAdminAuth()) return;

  String html = "<!DOCTYPE html><html lang='pt-br'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Configuracao NexulTech (admin)</title><style>";
  html += "body{font-family:Arial,sans-serif;background:#f4f6f8;margin:0;padding:20px;color:#222}";
  html += ".card{background:#fff;border-radius:8px;padding:20px;max-width:520px;margin:0 auto 16px;box-shadow:0 2px 6px rgba(0,0,0,.1)}";
  html += "h2{margin-top:0;color:#0a5c8a}label{display:block;margin-top:10px;font-weight:bold;font-size:14px}";
  html += "input[type=text],input[type=password],input[type=number]{width:100%;padding:8px;margin-top:4px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}";
  html += "button{margin-top:16px;padding:10px 16px;background:#0a5c8a;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:15px}";
  html += "button:hover{background:#084a70}";
  html += ".hint{font-size:12px;color:#888;margin-top:4px}";
  html += "a{color:#0a5c8a}";
  html += ".leitobox{border:1px solid #eee;border-radius:6px;padding:10px;margin-top:10px}";
  html += "</style></head><body>";

  html += "<div class='card'><a href='/'>&larr; Voltar ao painel</a></div>";

  html += "<div class='card'><h2>Configuracao</h2><form action='/save' method='POST'>";
  html += "<label>SSID WiFi</label><input type='text' id='ssidInput' name='ssid' value='" + htmlEscape(cfgSSID) + "'>";
  html += "<button type='button' id='scanBtn' onclick='scanWifi()' style='background:#e9c46a;color:#333'>Escanear redes WiFi</button>";
  html += "<select id='wifiScanSelect' onchange='fillSSID(this)' style='width:100%;padding:8px;margin-top:8px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box'>";
  html += "<option value=''>-- Clique em 'Escanear redes WiFi' --</option></select>";
  html += "<label>Senha WiFi</label><input type='password' name='pass' value='" + htmlEscape(cfgPASS) + "'>";
  html += "<label>Servidor MQTT (IP ou host)</label><input type='text' name='mqttsrv' value='" + htmlEscape(cfgMqttServer) + "'>";
  html += "<label>Porta MQTT</label><input type='number' name='mqttport' value='" + String(cfgMqttPort) + "'>";
  html += "<label>Usuario MQTT (opcional)</label><input type='text' name='mqttuser' value='" + htmlEscape(cfgMqttUser) + "'>";
  html += "<label>Senha MQTT (opcional)</label><input type='password' name='mqttpass' value='" + htmlEscape(cfgMqttPass) + "'>";
  html += "<label>Topico base MQTT (bateria/status geral do painel)</label><input type='text' name='basetopic' value='" + htmlEscape(cfgBaseTopic) + "'>";

  html += "<h2 style='margin-top:24px'>Leitos</h2>";
  html += "<div class='hint'>Nome exibido, topico MQTT de entrada (aciona o vermelho) e topico de saida (tempo decorrido) de cada leito.</div>";
  for (int i = 0; i < NUM_LEITOS; i++) {
    String idx = String(i + 1);
    html += "<div class='leitobox'><b>Leito " + idx + "</b>";
    html += "<label>Nome</label><input type='text' name='lname" + idx + "' value='" + htmlEscape(cfgLeitoName[i]) + "'>";
    html += "<label>Topico MQTT de entrada (chamada)</label><input type='text' name='ltopc" + idx + "' value='" + htmlEscape(cfgLeitoTopicChamar[i]) + "'>";
    html += "<label>Topico MQTT de saida (tempo)</label><input type='text' name='ltopt" + idx + "' value='" + htmlEscape(cfgLeitoTopicTempo[i]) + "'>";
    html += "<label>Topico MQTT de saida (estado ao vivo p/ dashboard)</label><input type='text' name='ltope" + idx + "' value='" + htmlEscape(cfgLeitoTopicEstado[i]) + "'>";
    html += "</div>";
  }

  html += "<h2 style='margin-top:24px'>Acesso admin</h2>";
  html += "<label>Usuario de acesso</label><input type='text' name='adminuser' value='" + htmlEscape(cfgAdminUser) + "'>";
  html += "<label>Senha de acesso</label><input type='password' name='adminpass' value=''>";
  html += "<div class='hint'>Deixe a senha em branco para nao altera-la. Recomendamos trocar a senha padrao no primeiro acesso.</div>";

  html += "<button type='submit'>Salvar e Reiniciar</button>";
  html += "</form></div>";

  html += "<script>";
  html += "function scanWifi(){";
  html += "  var btn=document.getElementById('scanBtn');";
  html += "  var sel=document.getElementById('wifiScanSelect');";
  html += "  btn.disabled=true; btn.innerText='Escaneando...';";
  html += "  sel.innerHTML='<option>Escaneando...</option>';";
  html += "  fetch('/scan').then(function(r){return r.json();}).then(function(list){";
  html += "    list.sort(function(a,b){return b.rssi-a.rssi;});";
  html += "    sel.innerHTML='<option value=\"\">-- Selecione uma rede --</option>';";
  html += "    list.forEach(function(n){";
  html += "      var opt=document.createElement('option');";
  html += "      opt.value=n.ssid;";
  html += "      opt.text=n.ssid+' ('+n.rssi+' dBm) '+(n.secure?'[protegida]':'[aberta]');";
  html += "      sel.appendChild(opt);";
  html += "    });";
  html += "    if(list.length===0){ sel.innerHTML='<option value=\"\">Nenhuma rede encontrada</option>'; }";
  html += "    btn.disabled=false; btn.innerText='Escanear redes WiFi';";
  html += "  }).catch(function(e){";
  html += "    sel.innerHTML='<option value=\"\">Erro ao escanear</option>';";
  html += "    btn.disabled=false; btn.innerText='Escanear redes WiFi';";
  html += "  });";
  html += "}";
  html += "function fillSSID(sel){ if(sel.value){ document.getElementById('ssidInput').value = sel.value; } }";
  html += "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSave() {
  if (!checkAdminAuth()) return;

  if (server.hasArg("ssid"))      cfgSSID       = server.arg("ssid");
  if (server.hasArg("pass"))      cfgPASS       = server.arg("pass");
  if (server.hasArg("mqttsrv"))   cfgMqttServer = server.arg("mqttsrv");
  if (server.hasArg("mqttport"))  cfgMqttPort   = server.arg("mqttport").toInt();
  if (server.hasArg("mqttuser"))  cfgMqttUser   = server.arg("mqttuser");
  if (server.hasArg("mqttpass"))  cfgMqttPass   = server.arg("mqttpass");
  if (server.hasArg("basetopic")) cfgBaseTopic  = server.arg("basetopic");

  for (int i = 0; i < NUM_LEITOS; i++) {
    String idx = String(i + 1);
    if (server.hasArg("lname" + idx))  cfgLeitoName[i]        = server.arg("lname" + idx);
    if (server.hasArg("ltopc" + idx))  cfgLeitoTopicChamar[i] = server.arg("ltopc" + idx);
    if (server.hasArg("ltopt" + idx))  cfgLeitoTopicTempo[i]  = server.arg("ltopt" + idx);
    if (server.hasArg("ltope" + idx))  cfgLeitoTopicEstado[i] = server.arg("ltope" + idx);
  }

  if (server.hasArg("adminuser") && server.arg("adminuser").length() > 0) {
    cfgAdminUser = server.arg("adminuser");
  }
  if (server.hasArg("adminpass") && server.arg("adminpass").length() > 0) {
    cfgAdminPass = server.arg("adminpass");
  }

  saveConfig();

  String html = "<html><head><meta charset='UTF-8'></head><body>";
  html += "<h3>Configuracoes salvas! Reiniciando em alguns segundos...</h3>";
  html += "<script>setTimeout(function(){window.location='/';}, 5000);</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);

  delay(1500);
  ESP.restart();
}

void handleStatus() {
  String json = "{";
  json += "\"wifi\":\"" + String(apMode ? "AP" : "STA") + "\",";
  json += "\"mqtt\":" + String(mqttClient.connected() ? "true" : "false") + ",";
  json += "\"battery_percent\":" + String(lastBatteryPercent, 1) + ",";
  json += "\"battery_voltage\":" + String(lastBatteryVoltage, 2) + ",";
  json += "\"captive_portal_pending\":" + String(assistPortalActive ? "true" : "false") + ",";
  json += "\"leitos\":[";
  for (int i = 0; i < NUM_LEITOS; i++) {
    if (i > 0) json += ",";
    json += "{\"nome\":\"" + cfgLeitoName[i] + "\",\"estado\":\"" + String(leitoStateLabel(leitoState[i])) + "\"}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}
