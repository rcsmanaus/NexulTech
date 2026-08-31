/*
  ============================================================================
  Controle de Leito + Reles + Monitor de Bateria - ESP32
  ============================================================================

  FUNCIONALIDADES:
  - 2 botoes fisicos que enviam mensagem via MQTT (mensagem editavel pela web)
  - Pagina web local de configuracao (SSID, senha, IP/porta MQTT, mensagens)
  - Controle de 2 reles pela pagina web local
  - Monitor de bateria via ADC, com alerta MQTT quando a carga atingir 80%+
  - Configuracoes salvas na memoria NVS (sobrevivem a reinicio/queda de energia)
  - Se nao houver WiFi configurado ou a conexao falhar, o dispositivo sobe um
    Access Point proprio (SSID "NexulTech", senha "nexultech") para voce
    acessar a pagina de configuracao em 192.168.4.1

  BIBLIOTECAS NECESSARIAS (Arduino IDE > Gerenciador de Bibliotecas):
  - PubSubClient (Nick O'Leary)
  - WiFi, WebServer, Preferences  -> ja vem no pacote da placa ESP32

  PINAGEM (ajuste conforme sua montagem):
  - Botao 1 (vermelho) . GPIO 32  (para GND, usa pull-up interno)
  - Botao 2 (verde) .... GPIO 33  (para GND, usa pull-up interno)
  - LED do botao 1 ..... GPIO 27  (vermelho)
  - LED do botao 2 ..... GPIO 14  (verde)
  - Rele 1 .............. GPIO 25
  - Rele 2 .............. GPIO 26
  - Leitura bateria ..... GPIO 35  (ADC, entrada de um divisor resistivo)
  - LED status .......... GPIO 2   (acende fixo quando conectado ao WiFi)
  ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiUdp.h>

// NAT (Network Address Translation) entre a interface AP (NexulTech) e a
// interface STA (rede do hospital/shopping). E isso que permite que o
// trafego do celular conectado no NexulTech "atravesse" o ESP32 e chegue
// ate o gateway real da rede -- inclusive a pagina de login do Captive
// Portal. Sem isso, o celular so conseguiria falar com o proprio ESP32.
// Presente no core arduino-esp32 (esp-lwip) atual; se o seu core for muito
// antigo, atualize-o em Ferramentas > Gerenciador de Placas.
#include "lwip/lwip_napt.h"
#include "lwip/dns.h"
// Necessario para LOCK_TCPIP_CORE()/UNLOCK_TCPIP_CORE(): o lwIP do ESP-IDF
// exige que chamadas "cruas" (fora da API do Arduino/WiFi.h), como
// ip_napt_enable(), sejam feitas segurando esse lock -- sem isso o proprio
// lwIP derruba o firmware com "assert failed: sys_timeout ... Required to
// lock TCPIP core functionality!" (foi exatamente esse o crash reportado).
#include "lwip/tcpip.h"

// ---------------- PINOS ----------------
#define BUTTON1_PIN     32
#define BUTTON2_PIN     33
#define LED1_PIN        27   // LED vermelho embutido no botao 1
#define LED2_PIN        14   // LED verde embutido no botao 2
#define RELAY1_PIN      25
#define RELAY2_PIN      26
#define BATTERY_ADC_PIN 35
#define LED_STATUS_PIN  2

// ---------------- CALIBRACAO DE BATERIA ----------------
// Ajuste esses valores conforme a bateria e o divisor resistivo usados.
#define BATTERY_MIN_V         3.0    // tensao considerada 0%
#define BATTERY_MAX_V         3.9    // tensao considerada 100%
#define ADC_REF_V             3.3
#define ADC_RESOLUTION        4095.0
#define VOLTAGE_DIVIDER_RATIO 2.0    // ex: dois resistores iguais = divide por 2

// Limite de alerta de bateria "carregada" (conforme solicitado: 80% em diante)
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
String cfgBaseTopic;
String cfgMsgButton1;
String cfgMsgButton2;
String cfgAdminUser;   // usuario de acesso a pagina de configuracao
String cfgAdminPass;   // senha de acesso a pagina de configuracao

// ---------------- ESTADOS ----------------
bool relay1State = false;
bool relay2State = false;
bool apMode = false;

// Estado dos LEDs dos botoes: vermelho (botao1) e verde (botao2) sao
// mutuamente exclusivos -- apertar um acende o seu LED e apaga o outro,
// e o LED fica aceso ate o outro botao ser pressionado.
bool led1RedOn   = false;
bool led2GreenOn = false;

unsigned long lastBtn1Press = 0;
unsigned long lastBtn2Press = 0;
const unsigned long DEBOUNCE_MS = 400;

unsigned long lastBatteryCheck = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 60000; // le bateria a cada 1 min
bool battery80Sent = false;
float lastBatteryPercent = 0;
float lastBatteryVoltage = 0;

unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL = 5000;

// ---------------- VALORES PADRAO (1a vez que liga) ----------------
const char* DEFAULT_MQTT_SERVER = "179.125.32.238";
const int   DEFAULT_MQTT_PORT   = 2883;
const char* DEFAULT_BASE_TOPIC  = "hospital/leito01";
const char* DEFAULT_MSG_B1      = "Entrar - Botao 1 acionado";
const char* DEFAULT_MSG_B2      = "Sair - Botao 2 acionado";
const char* AP_SSID = "NexulTech";
const char* AP_PASS = "nexultech";
const char* DEFAULT_ADMIN_USER = "admin";
const char* DEFAULT_ADMIN_PASS = "nexultech"; // ALTERE isso assim que configurar o dispositivo

// ---------------- CAPTIVE PORTAL ASSISTIDO (qualquer rede -- hospital, ----
// ---------------- shopping, hotel, aeroporto, qualquer ISP) --------------
// Quando o ESP32 conecta numa rede aberta que exige aceitar termos / logar
// numa pagina antes de liberar internet, ele fica "preso" (conectado no
// WiFi mas sem rota real para fora). Nesse caso o NexulTech continua ligado
// e um operador conecta o CELULAR nele; o trafego do celular e roteado
// (NAT) pela interface STA ate o gateway real, e a tela de login do
// provedor (seja qual for) abre sozinha no celular -- igual abriria se o
// celular estivesse ligado direto naquela rede. Uma vez autenticado, como o
// trafego sai com o mesmo IP/MAC da interface STA do ESP32, o proprio
// ESP32 passa a ter internet liberada tambem.
//
// Em vez da biblioteca DNSServer (que responde TODO dominio com o IP do
// ESP32), usamos um mini-proxy de DNS proprio: so os dominios que os
// proprios celulares usam para *detectar* que a rede exige login (fabrica
// do iOS/Android/Windows -- isso nao muda de provedor para provedor)
// recebem o IP do ESP32, disparando o popup automatico. Qualquer outro
// dominio -- inclusive o do provedor de internet real, seja ele qual for
// -- e encaminhado para o DNS de verdade da rede STA. Isso evita loop de
// DNS e faz o HTTPS (certificado por nome de dominio) funcionar
// corretamente com QUALQUER captive portal, nao so com um provedor
// especifico.
WiFiUDP dnsUdp;
const uint16_t DNS_PORT = 53;
bool assistPortalActive = false;   // true = NexulTech ligado propositalmente p/ liberar a rede
bool natEnabled = false;

// Domínios padrão de fábrica usados pelos sistemas operacionais para
// detectar Captive Portal. Isso é o mesmo em qualquer rede/provedor -- não
// precisa (e nao deve) ser customizado por ISP.
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

// URL de login capturada dinamicamente na sondagem (probeCaptivePortal) --
// funciona com QUALQUER provedor que devolva um redirecionamento HTTP
// padrao (isso e o comportamento generico de captive portal, nao algo
// especifico de uma operadora).
String portalRedirectURL = "";

unsigned long lastPortalProbe = 0;
const unsigned long PORTAL_PROBE_INTERVAL = 20000; // reavalia a cada 20s enquanto preso no portal
const char* CAPTIVE_CHECK_HOST = "connectivitycheck.gstatic.com";
const char* CAPTIVE_CHECK_PATH = "/generate_204";

// Se der problema de novo e voce quiser isolar a causa: mude para "false"
// para testar so o auto-popup (DNS) sem tentar habilitar o NAT.
const bool ENABLE_NAT = false;

// So comeca a sondar Captive Portal depois que o STA estiver conectado ha
// pelo menos esse tempo (radio "assentado"), e nunca antes da primeira vez.
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
void handleRelay();
void handleStatus();
bool checkAdminAuth();
void mqttReconnect();
void publishMessage(const String &topic, const String &payload);
void checkButtons();
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

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  analogReadResolution(12); // 0-4095
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db); // permite ler ate ~3.3V no pino do ADC

  loadConfig();
  connectWiFi();
  setupWebServer();

  mqttClient.setServer(cfgMqttServer.c_str(), cfgMqttPort);
  mqttClient.setBufferSize(640);

  // IMPORTANTE: a checagem de Captive Portal NAO e feita aqui no setup().
  // Subir o AP (NexulTech) e abrir uma conexao TCP quase ao mesmo tempo,
  // logo na inicializacao, e uma causa comum de crash/reboot no ESP32.
  // Por isso ela so roda dentro do loop(), depois que o WiFi ja esta
  // estavel ha alguns segundos (veja manageCaptivePortal()).

  Serial.println("Sistema Nexultech Leito.");
}

void loop() {
  server.handleClient();

  if (assistPortalActive) {
    handleDnsRequests();
  }

  if (!apMode) {
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED_STATUS_PIN, LOW);
      staWasConnected = false; // forca reassentamento antes de sondar de novo
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

  checkButtons();
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
  cfgMsgButton1 = prefs.getString("msg1", DEFAULT_MSG_B1);
  cfgMsgButton2 = prefs.getString("msg2", DEFAULT_MSG_B2);
  cfgAdminUser  = prefs.getString("adminuser", DEFAULT_ADMIN_USER);
  cfgAdminPass  = prefs.getString("adminpass", DEFAULT_ADMIN_PASS);
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
  prefs.putString("msg1", cfgMsgButton1);
  prefs.putString("msg2", cfgMsgButton2);
  prefs.putString("adminuser", cfgAdminUser);
  prefs.putString("adminpass", cfgAdminPass);
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

  // Redes publicas abertas (tipo captive portal de hospital/shopping)
  // costumam ter muitos aparelhos conectados ao mesmo tempo e podem
  // demorar mais que o normal pra associar/pegar IP via DHCP. Por isso
  // damos mais tempo por tentativa e mais de uma tentativa antes de
  // desistir e cair no modo AP de configuracao.
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
  // Para qualquer tentativa de conexao/reconexao pendente da STA. Sem isso,
  // se a conexao anterior falhou, o WiFi core do ESP-IDF fica tentando
  // reconectar sozinho em segundo plano pra sempre -- e enquanto isso
  // acontece, ele BLOQUEIA qualquer scanNetworks() (foi a causa do
  // "Escanear rede" nao achar nada: "STA is connecting, scan are not
  // allowed!" nos logs).
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

// Faz uma requisicao HTTP simples para um endpoint de teste de
// conectividade (o mesmo que o Android usa). Se a resposta vier "204 No
// Content" a internet esta realmente livre. Qualquer outra coisa (erro de
// conexao, redirecionamento, pagina HTML no lugar do 204) indica que a
// rede esta interceptando o trafego -- ou seja, tem Captive Portal.
// Quando ha um redirecionamento (Location:) ele e capturado em redirectUrl.
bool probeCaptivePortal(String &redirectUrl) {
  redirectUrl = "";
  WiFiClient client;
  client.setTimeout(4000);

  if (!client.connect(CAPTIVE_CHECK_HOST, 80)) {
    // Nao conseguiu nem abrir a conexao -> trata como possivel bloqueio
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

  return true; // qualquer resposta diferente de 204 = portal/bloqueio
}

// Liga o NexulTech junto com a conexao STA (modo AP+STA) e prepara DNS +
// NAT para que o celular do operador consiga ser redirecionado ate a
// pagina de login real e, uma vez logado, o proprio ESP32 ganhe acesso.
void enableAssistPortalAP() {
  // Se o AP ja estava ligado (ex.: apMode por falha de STA), nao precisa
  // religar -- so garante o modo AP_STA.
  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
    delay(100);
  }

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (!apOk) {
    Serial.println("Falha ao subir o AP NexulTech para autenticacao assistida.");
    return;
  }

  // Espera o AP realmente ter um IP valido antes de mexer em DNS/NAT
  // (evita chamar essas APIs sobre uma interface ainda nao pronta).
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

  // Nosso mini-proxy de DNS: os dominios "gatilho" (checagem de
  // conectividade do proprio SO) sao respondidos com o IP do ESP32 --
  // isso e o que faz o celular, ao conectar, detectar "rede com Captive
  // Portal" e abrir o navegador sozinho. Todo o resto (inclusive o
  // dominio do provedor de internet, seja ele qual for) e encaminhado
  // para o DNS real da rede STA (ver handleDnsRequests()).
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
  // Mantem o AP_STA ligado (nao desliga o NexulTech) para continuar
  // permitindo acesso local a pagina de configuracao pelo celular/notebook.
}

// Habilita o roteamento (NAT) entre a interface AP (NexulTech) e a
// interface STA, para que pacotes vindos do celular conectado no NexulTech
// consigam sair pela rede do hospital/shopping (e vice-versa).
// Se o core arduino-esp32 instalado nao tiver NAPT habilitado, essa chamada
// simplesmente nao ativa o roteamento -- o auto-popup do celular (via DNS)
// continua funcionando, mas a pagina de login real pode nao carregar. Isso
// e uma limitacao do core, nao um bug do sketch (veja aviso no topo do
// arquivo).
void enableNAT() {
  if (natEnabled) return;
  // ip_napt_enable() e uma chamada "crua" do lwIP -- precisa ser feita com
  // o TCPIP core travado, ou o firmware derruba com um assert (foi a causa
  // do crash "Required to lock TCPIP core functionality!").
  LOCK_TCPIP_CORE();
  ip_napt_enable(WiFi.softAPIP(), 1);
  UNLOCK_TCPIP_CORE();
  natEnabled = true;
  Serial.println("NAT habilitado entre NexulTech (AP) e a rede STA.");
}

// Chamada periodicamente enquanto o ESP32 esta conectado via STA.
// Detecta a entrada e a saida do estado de Captive Portal.
// So comeca a agir depois que o WiFi ficou estavel por WIFI_SETTLE_MS,
// para nao mexer no radio (subir AP + abrir TCP) logo na inicializacao.
void manageCaptivePortal() {
  unsigned long now = millis();

  if (!staWasConnected) {
    staWasConnected = true;
    staConnectedSince = now;
    return; // primeira vez que entra aqui: so marca o tempo e sai
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

// Extrai o QNAME (nome de dominio perguntado) da primeira pergunta de um
// pacote DNS bruto. Formato: sequencia de "labels" prefixados por tamanho,
// terminando em um byte 0x00 (ex.: 3 "www" 8 "exemplo" 3 "com" 0).
bool extractDnsQuestionName(uint8_t *buf, int len, String &qname) {
  qname = "";
  int pos = 12; // pula o cabecalho fixo de 12 bytes
  while (pos < len && buf[pos] != 0) {
    int labelLen = buf[pos];
    if (labelLen < 0 || labelLen > 63) return false; // formato invalido/comprimido, nao suportado
    pos++;
    if (pos + labelLen > len) return false;
    if (qname.length() > 0) qname += ".";
    for (int i = 0; i < labelLen; i++) {
      qname += (char)buf[pos + i];
    }
    pos += labelLen;
  }
  return pos < len; // encontrou o terminador 0x00 dentro do pacote
}

// Monta e envia uma resposta DNS simples (1 registro tipo A) reaproveitando
// a pergunta original recebida -- e assim que a biblioteca DNSServer
// tambem faz por baixo dos panos.
void sendDnsAnswer(uint8_t *query, int queryLen, IPAddress destIp, uint16_t destPort, IPAddress answerIp) {
  if (queryLen < 12 || queryLen > 480) return; // deixa margem pra resposta caber em 512 bytes

  uint8_t response[512];
  memcpy(response, query, queryLen);
  int len = queryLen;

  response[2] = 0x81; // QR=1 (resposta), Opcode=0, AA=0, TC=0, RD=1
  response[3] = 0x80; // RA=1, Z=0, RCODE=0 (sem erro)
  response[6] = 0x00; response[7] = 0x01; // ANCOUNT = 1
  response[8] = 0x00; response[9] = 0x00; // NSCOUNT = 0
  response[10] = 0x00; response[11] = 0x00; // ARCOUNT = 0

  response[len++] = 0xC0; response[len++] = 0x0C; // ponteiro de compressao -> nome no offset 12
  response[len++] = 0x00; response[len++] = 0x01; // TYPE = A
  response[len++] = 0x00; response[len++] = 0x01; // CLASS = IN
  response[len++] = 0x00; response[len++] = 0x00; response[len++] = 0x00; response[len++] = 0x1E; // TTL = 30s
  response[len++] = 0x00; response[len++] = 0x04; // RDLENGTH = 4
  response[len++] = answerIp[0];
  response[len++] = answerIp[1];
  response[len++] = answerIp[2];
  response[len++] = answerIp[3];

  dnsUdp.beginPacket(destIp, destPort);
  dnsUdp.write(response, len);
  dnsUdp.endPacket();
}

// Encaminha a pergunta (sem modificar) para o DNS de verdade que a rede
// STA recebeu via DHCP, espera a resposta real e repassa pro cliente do
// NexulTech. Funciona para QUALQUER dominio de QUALQUER provedor.
void forwardDnsQuery(uint8_t *query, int queryLen, IPAddress clientIp, uint16_t clientPort) {
  IPAddress realDns = WiFi.dnsIP();
  if (realDns == IPAddress(0, 0, 0, 0)) return; // sem DNS real conhecido ainda

  WiFiUDP fwd;
  fwd.begin(0); // porta local livre, escolhida pelo sistema
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

// Chamada a cada loop() enquanto assistPortalActive. Le UMA pergunta DNS
// pendente (se houver) e decide: responder com o IP do ESP32 (gatilho de
// popup) ou encaminhar pro DNS real (qualquer outro dominio).
void handleDnsRequests() {
  int packetSize = dnsUdp.parsePacket();
  if (packetSize <= 0) return;

  uint8_t buf[512];
  int len = dnsUdp.read(buf, sizeof(buf));
  if (len < 12) return; // pacote menor que um cabecalho DNS valido

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
    // Dominio desconhecido (inclui o do provedor de internet real, seja
    // qual for) -- encaminha pro DNS de verdade em vez de responder com
    // o IP do ESP32. Isso e o que evita o loop de DNS e faz o HTTPS da
    // pagina de login funcionar (certificado bate com o dominio certo).
    forwardDnsQuery(buf, len, clientIp, clientPort);
  }
}

// =====================================================================
// MQTT
// =====================================================================
void mqttReconnect() {
  if (millis() - lastMqttAttempt < MQTT_RETRY_INTERVAL) return;
  lastMqttAttempt = millis();

  mqttClient.setServer(cfgMqttServer.c_str(), cfgMqttPort);

  String clientId = "ESP32-NexulTech-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  bool ok;
  if (cfgMqttUser.length() > 0) {
    ok = mqttClient.connect(clientId.c_str(), cfgMqttUser.c_str(), cfgMqttPass.c_str());
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }

  if (ok) {
    Serial.println("MQTT conectado.");
  } else {
    Serial.printf("Falha MQTT, rc=%d (tentando de novo em %lus)\n",
                   mqttClient.state(), MQTT_RETRY_INTERVAL / 1000);
  }
}

void publishMessage(const String &topic, const String &payload) {
  if (apMode) return; // sem STA nao ha rota ate o broker
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

// =====================================================================
// BOTOES NexulTech
// =====================================================================
void checkButtons() {
  unsigned long now = millis();

  if (digitalRead(BUTTON1_PIN) == LOW && (now - lastBtn1Press > DEBOUNCE_MS)) {
    lastBtn1Press = now;
    Serial.println("Botao 1 pressionado.");

    // Acende o LED vermelho (botao1) e apaga o verde (botao2)
    led1RedOn = true;
    led2GreenOn = false;
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);

    publishMessage(cfgBaseTopic + "/botao1", cfgMsgButton1);
    publishMessage(cfgBaseTopic + "/led", "vermelho");
  }

  if (digitalRead(BUTTON2_PIN) == LOW && (now - lastBtn2Press > DEBOUNCE_MS)) {
    lastBtn2Press = now;
    Serial.println("Botao 2 pressionado.");

    // Acende o LED verde (botao2) e apaga o vermelho (botao1)
    led2GreenOn = true;
    led1RedOn = false;
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED1_PIN, LOW);

    publishMessage(cfgBaseTopic + "/botao2", cfgMsgButton2);
    publishMessage(cfgBaseTopic + "/led", "verde");
  }
}

// =====================================================================
// BATERIA
// =====================================================================
float readBatteryVoltage() {
  // Faz varias leituras e tira a media para reduzir ruido do ADC do ESP32
  const int NUM_SAMPLES = 16;
  uint32_t total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += analogRead(BATTERY_ADC_PIN);
    delay(2);
  }
  float rawAvg = total / (float)NUM_SAMPLES;

  float vAdc = (rawAvg / ADC_RESOLUTION) * ADC_REF_V;
  float vBat = vAdc * VOLTAGE_DIVIDER_RATIO;

  // Debug: se vBat continuar em 0.00V mesmo com bateria conectada, o problema
  // e de hardware/fiacao (nao de software). Verifique nessa ordem:
  //  1) O divisor resistivo esta de fato ligado ao pino BATTERY_ADC_PIN (GPIO35)?
  //  2) O GND do divisor esta no mesmo GND do ESP32?
  //  3) Confira com um multimetro a tensao que chega EXATAMENTE no pino do ADC
  //     (deve ser <= 3.3V, nunca a tensao total da bateria sem passar pelo divisor).
  //  4) Em alguns modelos de placa (ex.: TTGO T-Energy, LoRa32, WROVER) o pino
  //     de leitura de bateria de fabrica NAO e o GPIO35 -- confira o pinout
  //     especifico da sua placa e ajuste BATTERY_ADC_PIN se necessario.
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

  // Telemetria periodica da bateria
  String payload = "{\"voltage\":" + String(lastBatteryVoltage, 2) +
                    ",\"percent\":" + String(lastBatteryPercent, 1) + "}";
  publishMessage(cfgBaseTopic + "/bateria", payload);

  // Alerta quando a carga chega em 80% ou mais (envia 1x por ciclo de carga,
  // so libera novo alerta depois de cair abaixo de 75%)
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
// ESCANEAMENTO DE REDES WIFI (para redes de shopping/hospital/publicas)
// =====================================================================
String scanNetworksJSON() {
  // Se a STA estiver no meio de uma tentativa de conexao/reconexao (mas
  // AINDA NAO conectada), o ESP-IDF recusa fazer scan ("STA is
  // connecting, scan are not allowed!"). So interrompemos nesse caso --
  // se ja estiver conectada de verdade, nao mexemos em nada.
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
// Verifica se quem esta fazendo o pedido HTTP e um cliente conectado no
// NexulTech (AP), comparando os 3 primeiros octetos do IP com o IP do AP
// (ex.: 192.168.4.x). Usado para nao misturar acesso do celular (via AP,
// durante autenticacao assistida) com acesso normal a dashboard (via STA).
bool isRequestFromAP() {
  IPAddress clientIp = server.client().remoteIP();
  IPAddress apIp = WiFi.softAPIP();
  return clientIp[0] == apIp[0] && clientIp[1] == apIp[1] && clientIp[2] == apIp[2];
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/relay", handleRelay);
  server.on("/status", handleStatus);
  server.on("/scan", handleScan);
  // Qualquer caminho nao reconhecido (inclusive os que o celular usa para
  // detectar Captive Portal, tipo /generate_204, /hotspot-detect.html,
  // /ncsi.txt etc.) cai aqui -- e essencial para o auto-popup funcionar.
  server.onNotFound(handleNotFound);
  server.begin();
}

// Trata os pedidos que chegam via NexulTech quando estamos em modo de
// autenticacao assistida (o DNS wildcard faz todo dominio cair aqui).
void handleNotFound() {
  if (assistPortalActive) {
    redirectToGatewayPortal();
    return;
  }

  server.send(404, "text/plain", "Nao encontrado");
}

// Redireciona o navegador do celular para o IP do gateway da rede STA
// (ex.: o roteador da Claro/hospital/shopping), sem passar por nome de
// dominio -- assim o nosso proprio DNS wildcard nao interfere. E o
// gateway real quem vai interceptar esse pedido HTTP (por nao estarmos
// autenticados ainda) e mandar para a pagina de login de verdade.
// Redireciona o navegador do celular para a pagina de login de verdade.
// Prioridade 1: a URL capturada dinamicamente na sondagem (funciona com
// qualquer provedor -- e por nome de dominio, entao o HTTPS/certificado
// funciona corretamente, ja que nosso DNS agora encaminha esse dominio
// para o DNS real em vez de responder com o proprio IP).
// Prioridade 2 (fallback, se ainda nao capturamos nenhuma URL): manda pro
// IP do gateway da rede STA, sem passar por nome de dominio.
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

void handleRoot() {
  // Se o pedido veio de um cliente conectado no NexulTech (AP) enquanto
  // estamos aguardando autenticacao na rede STA, nao mostra a dashboard --
  // manda direto para o gateway real, senao o celular fica preso aqui e
  // nunca ve a tela de login do hospital/shopping/hotel.
  if (assistPortalActive && isRequestFromAP()) {
    redirectToGatewayPortal();
    return;
  }

  String ip = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String mqttStatus = mqttClient.connected() ? "Conectado" : "Desconectado";

  String html = "<!DOCTYPE html><html lang='pt-br'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>NexulTech</title><style>";
  html += "body{font-family:Arial,sans-serif;background:#f4f6f8;margin:0;padding:20px;color:#222}";
  html += ".card{background:#fff;border-radius:8px;padding:20px;max-width:480px;margin:0 auto 16px;box-shadow:0 2px 6px rgba(0,0,0,.1)}";
  html += "h2{margin-top:0;color:#0a5c8a}";
  html += "button{margin-top:16px;padding:10px 16px;background:#0a5c8a;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:15px}";
  html += "button:hover{background:#084a70}";
  html += ".relaybtn{background:#2a9d8f;margin-right:8px;margin-top:0}";
  html += ".adminbtn{background:#6c757d}";
  html += ".status{font-size:13px;color:#555;margin-bottom:8px}";
  html += ".badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;color:#fff}";
  html += ".ok{background:#2a9d8f}.bad{background:#e76f51}";
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
  html += "<div class='status'>LED botoes: <span class='badge " +
          String(led1RedOn ? "bad" : (led2GreenOn ? "ok" : "bad")) + "'>" +
          String(led1RedOn ? "Vermelho (botao 1)" : (led2GreenOn ? "Verde (botao 2)" : "Nenhum")) +
          "</span></div>";
  html += "</div>";

  html += "<div class='card'><h2>Reles</h2>";
  html += "<a href='/relay?r=1&s=" + String(relay1State ? 0 : 1) + "'><button class='relaybtn'>Rele 1: " +
          String(relay1State ? "ON (clique p/ desligar)" : "OFF (clique p/ ligar)") + "</button></a><br><br>";
  html += "<a href='/relay?r=2&s=" + String(relay2State ? 0 : 1) + "'><button class='relaybtn'>Rele 2: " +
          String(relay2State ? "ON (clique p/ desligar)" : "OFF (clique p/ ligar)") + "</button></a>";
  html += "</div>";

  html += "<div class='card'><h2>Area restrita</h2>";
  html += "<div class='status'>As configuracoes de WiFi, MQTT e mensagens ficam protegidas por login de administrador.</div>";
  html += "<a href='/config'><button class='adminbtn'>Configuracoes (admin)</button></a>";
  html += "</div>";

  html += "<script>setTimeout(function(){location.reload();}, 15000);</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Pagina de configuracao -- somente admin. Protegida por HTTP Basic Auth:
// o navegador pede usuario/senha automaticamente ao acessar /config.
void handleConfig() {
  if (!checkAdminAuth()) return;

  String html = "<!DOCTYPE html><html lang='pt-br'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Configuracao NexulTech (admin)</title><style>";
  html += "body{font-family:Arial,sans-serif;background:#f4f6f8;margin:0;padding:20px;color:#222}";
  html += ".card{background:#fff;border-radius:8px;padding:20px;max-width:480px;margin:0 auto 16px;box-shadow:0 2px 6px rgba(0,0,0,.1)}";
  html += "h2{margin-top:0;color:#0a5c8a}label{display:block;margin-top:10px;font-weight:bold;font-size:14px}";
  html += "input[type=text],input[type=password],input[type=number]{width:100%;padding:8px;margin-top:4px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}";
  html += "button{margin-top:16px;padding:10px 16px;background:#0a5c8a;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:15px}";
  html += "button:hover{background:#084a70}";
  html += ".hint{font-size:12px;color:#888;margin-top:4px}";
  html += "a{color:#0a5c8a}";
  html += "</style></head><body>";

  html += "<div class='card'><a href='/'>&larr; Voltar ao painel</a></div>";

  html += "<div class='card'><h2>Configuracao</h2><form action='/save' method='POST'>";
  html += "<label>SSID WiFi</label><input type='text' id='ssidInput' name='ssid' value='" + htmlEscape(cfgSSID) + "'>";
  html += "<button type='button' id='scanBtn' onclick='scanWifi()' style='background:#e9c46a;color:#333'>Escanear redes WiFi</button>";
  html += "<select id='wifiScanSelect' onchange='fillSSID(this)' style='width:100%;padding:8px;margin-top:8px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box'>";
  html += "<option value=''>-- Clique em 'Escanear redes WiFi' --</option></select>";
  html += "<label>Senha WiFi</label><input type='password' name='pass' value='" + htmlEscape(cfgPASS) + "'>";
  html += "<label>Servidor MQTT (IP)</label><input type='text' name='mqttsrv' value='" + htmlEscape(cfgMqttServer) + "'>";
  html += "<label>Porta MQTT</label><input type='number' name='mqttport' value='" + String(cfgMqttPort) + "'>";
  html += "<label>Usuario MQTT (opcional)</label><input type='text' name='mqttuser' value='" + htmlEscape(cfgMqttUser) + "'>";
  html += "<label>Senha MQTT (opcional)</label><input type='password' name='mqttpass' value='" + htmlEscape(cfgMqttPass) + "'>";
  html += "<label>Topico base MQTT</label><input type='text' name='basetopic' value='" + htmlEscape(cfgBaseTopic) + "'>";
  html += "<label>Mensagem Botao 1 (NexulTech)</label><input type='text' name='msg1' value='" + htmlEscape(cfgMsgButton1) + "'>";
  html += "<label>Mensagem Botao 2 (NexulTech)</label><input type='text' name='msg2' value='" + htmlEscape(cfgMsgButton2) + "'>";

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
  if (server.hasArg("msg1"))      cfgMsgButton1 = server.arg("msg1");
  if (server.hasArg("msg2"))      cfgMsgButton2 = server.arg("msg2");

  if (server.hasArg("adminuser") && server.arg("adminuser").length() > 0) {
    cfgAdminUser = server.arg("adminuser");
  }
  // Senha do admin so e alterada se o campo vier preenchido (deixar em
  // branco no formulario mantem a senha atual)
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

void handleRelay() {
  if (server.hasArg("r") && server.hasArg("s")) {
    int r = server.arg("r").toInt();
    int s = server.arg("s").toInt();

    if (r == 1) {
      relay1State = (s == 1);
      digitalWrite(RELAY1_PIN, relay1State ? HIGH : LOW);
      publishMessage(cfgBaseTopic + "/rele1/estado", relay1State ? "ON" : "OFF");
    } else if (r == 2) {
      relay2State = (s == 1);
      digitalWrite(RELAY2_PIN, relay2State ? HIGH : LOW);
      publishMessage(cfgBaseTopic + "/rele2/estado", relay2State ? "ON" : "OFF");
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStatus() {
  String json = "{";
  json += "\"wifi\":\"" + String(apMode ? "AP" : "STA") + "\",";
  json += "\"mqtt\":" + String(mqttClient.connected() ? "true" : "false") + ",";
  json += "\"battery_percent\":" + String(lastBatteryPercent, 1) + ",";
  json += "\"battery_voltage\":" + String(lastBatteryVoltage, 2) + ",";
  json += "\"relay1\":" + String(relay1State ? "true" : "false") + ",";
  json += "\"relay2\":" + String(relay2State ? "true" : "false") + ",";
  json += "\"led_red\":" + String(led1RedOn ? "true" : "false") + ",";
  json += "\"led_green\":" + String(led2GreenOn ? "true" : "false") + ",";
  json += "\"captive_portal_pending\":" + String(assistPortalActive ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}