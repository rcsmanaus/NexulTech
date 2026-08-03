// ============================================================
//  SoroCare Monitor v2.2.2
//  - Botão físico (GPIO26) para confirmar presença no leito
//    e silenciar alarme de soro vazio
//  - Fator de calibração E offset do tare salvos na NVS
//    (não perde calibração após queda de energia)
//  - Login/senha para Configuração e Calibração
//  - Zerar balança corrigido (tare assíncrono)
//  - Dashboard atualiza peso e nível em tempo real
//  Hardware: Wemos D1 R32 (ESP32) + HX711 + LM393 + Buzzer
// ============================================================
//
//  BOTÃO FÍSICO — CONEXÃO:
//  Pino GPIO26 → botão → GND
//  (pull-up interno ativo, pressionar = nível LOW)
//  Função: silenciar alarme de SORO VAZIO exigindo
//  presença física da enfermeira no leito.
//
//  CALIBRAÇÃO PERSISTENTE:
//  Após calibrar, tanto o fator quanto o offset do tare
//  são gravados na NVS (flash). Na próxima inicialização
//  os valores são restaurados automaticamente — sem precisar
//  recalibrar após queda de energia ou reinicialização.
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <HX711.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <esp_mac.h>

#define FW_VERSION    "2.2.2"
#define DEVICE_PREFIX "NexulTech"

// ── Pinos ────────────────────────────────────────────────────
#define HX711_DOUT   16
#define HX711_SCK    17
#define LM393_PIN    34
//#define LM393_PIN  18   // Depende da placa descomentar 
#define LED_STATUS    2
#define BUZZER_PIN   27
#define BTN_CONFIRMA 26   // Botão físico de confirmação da enfermeira

// ── Parâmetros ───────────────────────────────────────────────
#define MQTT_PORT_DEFAULT   2883
#define TARE_SAMPLES        20
#define PESO_VAZIO_DEFAULT  50.0f
#define PESO_ALERTA_DEFAULT 100.0f
#define GOTAS_POR_ML        20.0f
#define DEBOUNCE_GOTA_MS    30
#define DEBOUNCE_BTN_MS     50    // debounce do botão físico

// ── Credenciais admin padrão ─────────────────────────────────
#define ADMIN_USER_DEFAULT  "admin"
#define ADMIN_PASS_DEFAULT  "nexultech"

// ── Objetos ──────────────────────────────────────────────────
Preferences  prefs;
WebServer    server(80);
DNSServer    dns;
HX711        balanca;
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// ── Config ───────────────────────────────────────────────────
struct Config {
  char  ssid[64]         = "";
  char  senha[64]        = "";
  char  mqttHost[64]     = "179.125.32.238";
  int   mqttPort         = MQTT_PORT_DEFAULT;
  char  mqttUser[32]     = "";
  char  mqttPass[32]     = "";
  char  mqttTopico[64]   = "Hospiptal/Setor/Quaro/Leito01";
  char  nomePaciente[64] = "Paciente";
  char  nomeLeito[32]    = "Leito 01";
  float fatorCalibracao  = -420.0f;
  float pesoVazio        = PESO_VAZIO_DEFAULT;
  float pesoAlerta       = PESO_ALERTA_DEFAULT;
  float volumeTotal      = 500.0f;
  float taxaGotas        = 60.0f;
  char  logoUrl[128]     = "";
  char  nomeEmpresa[64]  = "NexulTech";
  char  adminUser[32]    = ADMIN_USER_DEFAULT;
  char  adminPass[32]    = ADMIN_PASS_DEFAULT;
} cfg;

// ── Runtime ──────────────────────────────────────────────────
volatile unsigned long contadorGotas = 0;
volatile unsigned long ultimaGota    = 0;

float   pesoAtual        = 0;
float   pesoInicial      = 0;   // peso registrado ao iniciar soro (para % restante)
float   taxaGotasAtual   = 0;
float   volumeRestante   = 0;
float   tempoRestante    = 0;
bool    alarmeAtivo      = false;
bool    soroVazioConfirmado = false; // true após enfermeira pressionar botão
bool    modoAP           = false;
bool    mqttConectado    = false;
bool    tararePendente   = false;
String  deviceId         = "";

// ── Botão físico ─────────────────────────────────────────────
bool     btnEstadoAnterior  = HIGH;
unsigned long tUltBtn       = 0;

// ── Timers ───────────────────────────────────────────────────
unsigned long tUltPublish = 0;
unsigned long tUltGotas   = 0;
unsigned long tUltPeso    = 0;
unsigned long tUltBlink   = 0;
unsigned long tUltBuzzer  = 0;   // controla beep contínuo sem bloquear o loop
bool          buzzerLigado = false;
int           blinkState   = 0;

// ── Forward declarations ──────────────────────────────────────
void bipCurto(int n);
void piscaLed(int pin, unsigned long intervalo);
void carregarConfig();
void salvarConfig();
void salvarCalibracaoNVS();
void lerCalibracaoNVS();
bool conectarWiFi();
void iniciarModoAP();
bool conectarMQTT();
void publicarMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int len);
void lerPeso();
void calcularTaxaGotas(unsigned long agora);
void verificarAlarmes();
void acionarAlarme(String motivo);
void silenciarAlarme(String origem);
void verificarBotaoFisico();
void configurarRotas();
String cssBase();
String htmlHeader(String paginaAtiva);
String htmlFooter();
void handleRoot();
void handleSaveConfig();
void handleApiDados();
void handleTare();
void handleCalibrar();
void handleReset();
void handleAlarmeOff();
void handleConfirmarSoro();
void handleLoginPage();
void handleLogin();
void handleLogout();
void handleConfigPage();
void handleCalibrarPage();
bool estaAutenticado();
bool verificarAuth(String u, String p);
void redirecionarLogin(bool erro = false);
void gerarToken();

// ── Sessão ───────────────────────────────────────────────────
String sessionToken = "";

// ── ISR gotas ────────────────────────────────────────────────
void IRAM_ATTR isrGota() {
  unsigned long agora = millis();
  if (agora - ultimaGota > DEBOUNCE_GOTA_MS) {
    contadorGotas++;
    ultimaGota = agora;
  }
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== NexulTech Monitor v" FW_VERSION " ===");

  pinMode(LED_STATUS,    OUTPUT);
  pinMode(BUZZER_PIN,    OUTPUT);
  pinMode(LM393_PIN,     INPUT);
  pinMode(BTN_CONFIRMA,  INPUT_PULLUP); // pull-up interno; pressionar = LOW
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  bipCurto(2);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char idBuf[32];
  snprintf(idBuf, sizeof(idBuf), "%s_%02X%02X", DEVICE_PREFIX, mac[4], mac[5]);
  deviceId = String(idBuf);
  deviceId.toUpperCase();
  Serial.println("ID: " + deviceId);

  carregarConfig();

  // ── HX711 — restaura fator e offset salvos ────────────────
  balanca.begin(HX711_DOUT, HX711_SCK);
  Serial.println("Aguardando HX711...");
  unsigned long t0 = millis();
  while (!balanca.is_ready() && millis() - t0 < 1000) delay(10);

  if (balanca.is_ready()) {
    lerCalibracaoNVS(); // restaura fator + offset da NVS
    Serial.printf("[CAL] Fator restaurado: %.4f  Offset: %ld\n",
                  cfg.fatorCalibracao, balanca.get_offset());
    Serial.println("HX711 OK — calibracao restaurada da NVS");
  } else {
    Serial.println("AVISO: HX711 nao respondeu");
  }

  attachInterrupt(digitalPinToInterrupt(LM393_PIN), isrGota, FALLING);

  if (strlen(cfg.ssid) > 0) {
    if (!conectarWiFi()) iniciarModoAP();
  } else {
    iniciarModoAP();
  }

  configurarRotas();
  server.begin();
  Serial.println("HTTP OK");

  if (!modoAP) {
    // Buffer padrão do PubSubClient é 256 bytes (tópico + payload).
    // Nosso JSON + tópico costuma passar disso, o que fazia o publish()
    // falhar silenciosamente depois de um tempo. Aumentando para 640 bytes.
    mqtt.setBufferSize(640);
    mqtt.setSocketTimeout(5);   // evita travar o loop() por muito tempo se o broker não responder
    mqtt.setKeepAlive(30);
    mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
    mqtt.setCallback(mqttCallback);
    conectarMQTT();
  }
  Serial.println("Pronto!\n");
  Serial.println("Botao fisico: GPIO" + String(BTN_CONFIRMA) + " (pressionar para confirmar presenca)");
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long agora = millis();

  // ── Tare agendado (thread-safe) ──────────────────────────
  if (tararePendente) {
    tararePendente = false;
    Serial.println("[TARE] Aguardando HX711...");
    unsigned long tw = millis();
    while (!balanca.is_ready() && millis() - tw < 1000) delay(5);
    if (balanca.is_ready()) {
      balanca.tare(TARE_SAMPLES);
      pesoAtual      = 0;
      volumeRestante = 0;
      salvarCalibracaoNVS(); // salva novo offset na NVS
      Serial.println("[TARE] OK. Offset=" + String(balanca.get_offset()));
    } else {
      Serial.println("[TARE] FALHOU");
    }
  }

  // ── Botão físico ─────────────────────────────────────────
  verificarBotaoFisico();

  if (modoAP) {
    dns.processNextRequest();
    server.handleClient();
    piscaLed(LED_STATUS, 200);
    return;
  }

  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) conectarWiFi();

  if (!mqtt.connected()) {
    static unsigned long tRetry = 0;
    if (agora - tRetry > 1000) { conectarMQTT(); tRetry = agora; }
  }
  mqtt.loop();

  if (agora - tUltPeso > 400) {
    tUltPeso = agora;
    lerPeso();
    verificarAlarmes();
  }

  if (agora - tUltGotas > 1000) {
    calcularTaxaGotas(agora);
    tUltGotas = agora;
  }

  if (agora - tUltPublish > 1000) {
    tUltPublish = agora;
    publicarMQTT();
  }

  // ── Buzzer não-bloqueante (beep repetitivo) ───────────────
  // Enquanto alarme ativo E soro não confirmado: beep a cada 2s
  if (alarmeAtivo && !soroVazioConfirmado) {
    if (agora - tUltBuzzer > 1000) {
      tUltBuzzer = agora;
      digitalWrite(BUZZER_PIN, HIGH);
    } else if (agora - tUltBuzzer > 300) {
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  piscaLed(LED_STATUS, alarmeAtivo ? 150 : 1200);
}

// ════════════════════════════════════════════════════════════
//  BOTÃO FÍSICO — verifica pressionamento com debounce
// ════════════════════════════════════════════════════════════
void verificarBotaoFisico() {
  bool estadoAtual = digitalRead(BTN_CONFIRMA);
  unsigned long agora = millis();

  // Detecta borda de descida (pressionado = LOW)
  if (btnEstadoAnterior == HIGH && estadoAtual == LOW) {
    if (agora - tUltBtn > DEBOUNCE_BTN_MS) {
      tUltBtn = agora;
      Serial.println("[BTN] Botao pressionado — confirmando presenca no leito");
      silenciarAlarme("BOTAO_FISICO");
    }
  }
  btnEstadoAnterior = estadoAtual;
}

// ════════════════════════════════════════════════════════════
//  SILENCIAR ALARME (unifica web + botão físico + MQTT)
// ════════════════════════════════════════════════════════════
void silenciarAlarme(String origem) {
  alarmeAtivo         = false;
  soroVazioConfirmado = true;
  digitalWrite(BUZZER_PIN, LOW);

  // 2 bips de confirmação
  bipCurto(2);

  Serial.println("[ALARME] Silenciado via: " + origem);

  // Publica confirmação no MQTT
  String topConfirm = String(cfg.mqttTopico) + "/confirmacao";
  char pay[200];
  snprintf(pay, sizeof(pay),
    "{\"origem\":\"%s\",\"leito\":\"%s\",\"paciente\":\"%s\",\"device\":\"%s\"}",
    origem.c_str(), cfg.nomeLeito, cfg.nomePaciente, deviceId.c_str());
  mqtt.publish(topConfirm.c_str(), pay, true);
}

// ════════════════════════════════════════════════════════════
//  LEITURAS
// ════════════════════════════════════════════════════════════
void lerPeso() {
  if (!balanca.is_ready()) {
    Serial.println("[HX711] nao pronto");
    return;
  }
  float v = balanca.get_units(5);
  Serial.printf("[PESO] raw=%.2f  fator=%.4f\n", v, cfg.fatorCalibracao);
  if (v < -5.0f) v = 0;
  pesoAtual      = max(0.0f, v);
  volumeRestante = max(0.0f, pesoAtual - cfg.pesoVazio);

  // Registra peso inicial na primeira leitura válida após inicio
  if (pesoInicial == 0 && pesoAtual > cfg.pesoAlerta) {
    pesoInicial = pesoAtual;
    Serial.printf("[PESO] Peso inicial registrado: %.1fg\n", pesoInicial);
  }
}

void calcularTaxaGotas(unsigned long agora) {
  static unsigned long gAnt = 0, tAnt = 0;
  unsigned long dg = contadorGotas - gAnt;
  unsigned long dt = agora - tAnt;
  if (dt > 0) taxaGotasAtual = (float)dg / (dt / 1000.0f) * 60.0f;
  if (taxaGotasAtual > 0) tempoRestante = volumeRestante / (taxaGotasAtual / GOTAS_POR_ML);
  gAnt = contadorGotas; tAnt = agora;
}

// ════════════════════════════════════════════════════════════
//  ALARMES
// ════════════════════════════════════════════════════════════
void verificarAlarmes() {
  // Se soro já foi confirmado como vazio, não reativa alarme até novo soro
  if (soroVazioConfirmado) return;

  bool   novoAlarme = false;
  String motivo     = "";

  if (pesoAtual > 10 && pesoAtual <= cfg.pesoVazio + 5) {
    novoAlarme = true;
    motivo     = "SORO VAZIO";
  } else if (pesoAtual > 10 && pesoAtual <= cfg.pesoAlerta) {
    novoAlarme = true;
    motivo     = "NIVEL BAIXO";
  }

  if (taxaGotasAtual > 0 && cfg.taxaGotas > 0) {
    float dev = abs(taxaGotasAtual - cfg.taxaGotas) / cfg.taxaGotas * 100;
    if (dev > 30) { novoAlarme = true; motivo += "|TAXA IRREGULAR"; }
  }

  if (novoAlarme && !alarmeAtivo) {
    alarmeAtivo = true;
    acionarAlarme(motivo);
  } else if (!novoAlarme && alarmeAtivo) {
    // Peso voltou ao normal (novo soro colocado)
    alarmeAtivo         = false;
    soroVazioConfirmado = false; // reseta para próximo ciclo
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[ALARME] Resolvido automaticamente");
  }
}

void acionarAlarme(String motivo) {
  Serial.println("ALARME: " + motivo);

  // Publica imediatamente no MQTT
  String tp = String(cfg.mqttTopico) + "/alarme";
  char pay[256];
  snprintf(pay, sizeof(pay),
    "{\"motivo\":\"%s\",\"peso\":%.1f,\"leito\":\"%s\","
    "\"paciente\":\"%s\",\"aguardando_confirmacao\":true,\"device\":\"%s\"}",
    motivo.c_str(), pesoAtual,
    cfg.nomeLeito, cfg.nomePaciente, deviceId.c_str());
  mqtt.publish(tp.c_str(), pay, true);

  // Primeira sequência de bips (o buzzer repetitivo fica no loop)
  bipCurto(3);
}

// ════════════════════════════════════════════════════════════
//  MQTT
// ════════════════════════════════════════════════════════════
void publicarMQTT() {
  if (!mqtt.connected()) return;

  // Calcula % em relação ao peso inicial
  float pctPeso = (pesoInicial > 0)
    ? (pesoAtual / pesoInicial * 100.0f)
    : 0;

  StaticJsonDocument<512> doc;
  doc["device"]               = deviceId;
  doc["paciente"]             = cfg.nomePaciente;
  doc["leito"]                = cfg.nomeLeito;
  doc["peso_g"]               = String(pesoAtual, 1);
  doc["volume_ml"]            = String(volumeRestante, 1);
  doc["gotas_min"]            = String(taxaGotasAtual, 1);
  doc["gotas_total"]          = (unsigned long)contadorGotas;
  doc["tempo_min"]            = String(tempoRestante, 0);
  doc["alarme"]               = alarmeAtivo;
  doc["soro_confirmado"]      = soroVazioConfirmado;
  doc["pct_peso_inicial"]     = String(pctPeso, 1);
  doc["fw"]                   = FW_VERSION;
  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));

  bool ok = mqtt.publish(cfg.mqttTopico, buf);
  Serial.printf("[MQTT] publish %s -> %s (%u bytes) state=%d\n",
                ok ? "OK" : "FALHOU", cfg.mqttTopico, (unsigned)n, mqtt.state());

  if (!ok) {
    // Se falhar (ex.: buffer estourado, broker caiu), força desconectar
    // para que o loop() detecte e tente reconectar no próximo ciclo,
    // em vez de ficar "conectado" e falhando silenciosamente para sempre.
    mqtt.disconnect();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  if (msg == "TARE")         { tararePendente = true; }
  if (msg == "ALARME_OFF")   { silenciarAlarme("MQTT_CMD"); }
  if (msg == "RESTART")      { ESP.restart(); }
  if (msg == "RESET_SORO")   { soroVazioConfirmado = false; pesoInicial = 0; }
}

bool conectarMQTT() {
  if (strlen(cfg.mqttHost) == 0) return false;
  bool ok = (strlen(cfg.mqttUser) > 0)
    ? mqtt.connect(deviceId.c_str(), cfg.mqttUser, cfg.mqttPass)
    : mqtt.connect(deviceId.c_str());
  if (ok) {
    mqttConectado = true;
    mqtt.subscribe((String(cfg.mqttTopico) + "/cmd").c_str());
    Serial.println("MQTT OK");
  } else {
    mqttConectado = false;
  }
  return ok;
}

// ════════════════════════════════════════════════════════════
//  WIFI
// ════════════════════════════════════════════════════════════
bool conectarWiFi() {
  if (strlen(cfg.ssid) == 0) return false;
  WiFi.mode(WIFI_STA);
  Serial.println("\n===== TENTANDO CONECTAR =====");
  Serial.print("SSID: ");
  Serial.println(cfg.ssid);
  Serial.print("Senha: ");
  Serial.println(cfg.senha);
  Serial.println("=============================\n");
  WiFi.begin(cfg.ssid, cfg.senha);
  for (int t = 0; t < 20 && WiFi.status() != WL_CONNECTED; t++) {
    delay(500);
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi OK — " + WiFi.localIP().toString());
    modoAP = false; return true;
  }
  return false;
}

void iniciarModoAP() {
  modoAP = true;
  String apName = "NexulTech_" + deviceId.substring(deviceId.length()-4);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str(), "nexultech");
  IPAddress apIP(192,168,4,1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  dns.start(53, "*", apIP);
  Serial.println("AP: " + apName + "  " + apIP.toString());
  bipCurto(3);
}

// ════════════════════════════════════════════════════════════
//  AUTENTICAÇÃO
// ════════════════════════════════════════════════════════════
void gerarToken() {
  uint32_t r = esp_random();
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", r);
  sessionToken = String(buf);
}

bool estaAutenticado() {
  if (server.hasHeader("Cookie")) {
    String cookie = server.header("Cookie");
    return cookie.indexOf("sc_sess=" + sessionToken) >= 0;
  }
  return false;
}

bool verificarAuth(String u, String p) {
  return (u == String(cfg.adminUser) && p == String(cfg.adminPass));
}

void redirecionarLogin(bool erro) {
  String url = erro ? "/login?erro=1" : "/login";
  server.sendHeader("Location", url, true);
  server.send(302, "text/plain", "");
}

// ════════════════════════════════════════════════════════════
//  ROTAS
// ════════════════════════════════════════════════════════════
void configurarRotas() {
  server.on("/",                  HTTP_GET,  handleRoot);
  server.on("/login",             HTTP_GET,  handleLoginPage);
  server.on("/login",             HTTP_POST, handleLogin);
  server.on("/logout",            HTTP_GET,  handleLogout);
  server.on("/config",            HTTP_GET,  handleConfigPage);
  server.on("/config",            HTTP_POST, handleSaveConfig);
  server.on("/calibrar",          HTTP_GET,  handleCalibrarPage);
  server.on("/api/dados",         HTTP_GET,  handleApiDados);
  server.on("/api/tare",          HTTP_GET,  handleTare);
  server.on("/api/calibrar",      HTTP_POST, handleCalibrar);
  server.on("/api/reset",         HTTP_GET,  handleReset);
  server.on("/api/alarme/off",    HTTP_GET,  handleAlarmeOff);
  server.on("/api/confirmar/soro",HTTP_GET,  handleConfirmarSoro);
  const char* hdrs[] = { "Cookie" };
  server.collectHeaders(hdrs, 1);
  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
}

// ════════════════════════════════════════════════════════════
//  CSS COMPARTILHADO
// ════════════════════════════════════════════════════════════
String cssBase() {
  return F("<style>"
  ":root{--bg:#0d1117;--card:#161b22;--card2:#1c2230;--border:#30363d;"
  "--accent:#00bfae;--accent2:#00897b;--red:#f44336;--amber:#ff9800;"
  "--green:#4caf50;--text:#e6edf3;--muted:#8b949e}"
  "*{box-sizing:border-box;margin:0;padding:0}"
  "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh}"
  "header{background:linear-gradient(135deg,#0d2a27,#0d1117);"
  "border-bottom:1px solid var(--border);padding:12px 20px;"
  "display:flex;align-items:center;gap:12px}"
  "#logo-img{max-height:46px;max-width:140px;object-fit:contain;border-radius:6px}"
  ".logo-pl{width:44px;height:44px;border-radius:8px;background:var(--accent2);"
  "display:flex;align-items:center;justify-content:center;"
  "font-size:15px;font-weight:700;color:#fff;flex-shrink:0}"
  ".brand h1{font-size:16px;font-weight:700;color:var(--accent)}"
  ".brand p{font-size:11px;color:var(--muted)}"
  ".sdot{width:9px;height:9px;border-radius:50%;background:var(--green);"
  "box-shadow:0 0 7px var(--green);margin-left:auto;flex-shrink:0}"
  ".sdot.off{background:var(--red);box-shadow:0 0 7px var(--red)}"
  "nav{display:flex;gap:4px;padding:8px 20px;border-bottom:1px solid var(--border);"
  "background:var(--card);overflow-x:auto;align-items:center}"
  "nav a{background:transparent;border:none;color:var(--muted);text-decoration:none;"
  "padding:6px 13px;border-radius:6px;cursor:pointer;font-size:12px;white-space:nowrap}"
  "nav a.a,nav a:hover{background:var(--card2);color:var(--accent)}"
  "nav a.lock{color:#ff9800!important;margin-left:4px}"
  ".nav-right{margin-left:auto;display:flex;gap:6px;align-items:center}"
  ".pill{font-size:10px;padding:3px 9px;border-radius:20px;"
  "background:#00bfae22;border:1px solid var(--accent);color:var(--accent)}"
  "main{padding:14px 20px;max-width:860px;margin:0 auto}"
  ".sc{background:var(--card);border:1px solid var(--border);border-radius:9px;"
  "padding:14px;margin-bottom:12px}"
  ".sc h3{font-size:12px;font-weight:600;color:var(--accent);margin:0 0 10px;"
  "display:flex;align-items:center;gap:6px}"
  ".irow{display:flex;justify-content:space-between;padding:5px 0;"
  "border-bottom:1px solid var(--border);font-size:12px}"
  ".irow:last-child{border:none}"
  ".irow span{color:var(--muted)}"
  ".metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin-bottom:12px}"
  ".met{background:var(--card);border:1px solid var(--border);border-radius:9px;"
  "padding:12px;position:relative;overflow:hidden}"
  ".met::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--accent)}"
  ".met.w::before{background:var(--amber)}"
  ".met.e::before{background:var(--red);animation:pulse 1s infinite}"
  "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}"
  ".met label{font-size:10px;color:var(--muted);text-transform:uppercase;"
  "letter-spacing:.7px;display:block;margin-bottom:3px}"
  ".met .v{font-size:26px;font-weight:700;line-height:1;color:var(--text)}"
  ".met .u{font-size:11px;color:var(--muted);margin-top:3px}"
  ".bar-bg{background:var(--card2);border-radius:20px;height:9px;overflow:hidden;margin:8px 0}"
  ".bar-f{height:100%;border-radius:20px;transition:width .6s ease;"
  "background:linear-gradient(90deg,var(--accent2),var(--accent))}"
  ".bar-f.low{background:linear-gradient(90deg,#b71c1c,var(--red))}"
  ".bar-f.warn{background:linear-gradient(90deg,#e65100,var(--amber))}"
  ".abanner{border-radius:9px;padding:14px 16px;margin-bottom:12px;"
  "display:flex;align-items:center;gap:12px}"
  ".abanner.alarme{background:#b71c1c22;border:1px solid var(--red);"
  "animation:pBg 1.5s infinite}"
  ".abanner.confirmado{background:#00897b22;border:1px solid var(--accent)}"
  "@keyframes pBg{0%,100%{background:#b71c1c22}50%{background:#b71c1c44}}"
  ".abanner button{margin-left:auto;border:none;"
  "padding:6px 14px;border-radius:5px;cursor:pointer;font-size:12px;font-weight:600}"
  ".btn-confirmar{background:#f44336;color:#fff;font-size:14px!important;"
  "padding:10px 20px!important;border-radius:8px!important;}"
  ".btn-confirmar:hover{background:#c62828}"
  ".fg{margin-bottom:11px}"
  ".fg label{display:block;font-size:10px;color:var(--muted);margin-bottom:4px;"
  "text-transform:uppercase;letter-spacing:.5px}"
  ".fg input{width:100%;background:var(--card2);border:1px solid var(--border);"
  "color:var(--text);padding:8px 10px;border-radius:6px;font-size:13px;outline:none}"
  ".fg input:focus{border-color:var(--accent);box-shadow:0 0 0 2px #00bfae22}"
  ".fr{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
  "@media(max-width:480px){.fr{grid-template-columns:1fr}}"
  ".btn{padding:8px 18px;border:none;border-radius:7px;cursor:pointer;"
  "font-size:12px;font-weight:600;transition:.15s;display:inline-flex;align-items:center;gap:5px}"
  ".bp{background:var(--accent2);color:#fff}.bp:hover{background:var(--accent)}"
  ".bd{background:#b71c1c;color:#fff}.bg{background:#1b5e20;color:#fff}"
  ".ba{background:#e65100;color:#fff}.bs{padding:6px 12px;font-size:11px}"
  ".amsg{border-radius:7px;padding:8px 12px;font-size:12px;margin-bottom:10px;display:none}"
  ".amsg.ok{background:#00bfae18;border:1px solid var(--accent);color:var(--accent)}"
  ".amsg.err{background:#f4433618;border:1px solid var(--red);color:var(--red)}"
  "footer{text-align:center;padding:14px;color:var(--muted);font-size:10px;"
  "border-top:1px solid var(--border);margin-top:14px}"
  "</style>");
}

// ════════════════════════════════════════════════════════════
//  HEADER / FOOTER HTML compartilhados
// ════════════════════════════════════════════════════════════
String htmlHeader(String paginaAtiva) {
  String h = "<!DOCTYPE html><html lang='pt-BR'><head>"
             "<meta charset='UTF-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<title>SoroCare</title>";
  h += cssBase();
  h += "</head><body><header>";
  if (strlen(cfg.logoUrl) > 0) {
    h += "<img id='logo-img' src='" + String(cfg.logoUrl) + "' alt='logo'>";
  } else {
    String ini = String(cfg.nomeEmpresa).substring(0,2); ini.toUpperCase();
    h += "<div class='logo-pl'>" + ini + "</div>";
  }
  h += "<div class='brand'><h1>" + String(cfg.nomeEmpresa) + "</h1>";
  h += "<p>" + deviceId + " &bull; v" FW_VERSION "</p></div>";
  h += "<div id='sdot' class='sdot'></div></header>";

  bool auth = estaAutenticado();
  h += "<nav>";
  h += "<a href='/' class='" + String(paginaAtiva=="dash"?"a":"") + "'>&#128202; Dashboard</a>";
  if (auth) {
    h += "<a href='/config'   class='" + String(paginaAtiva=="conf"?"a":"") + "'>&#9881; Configura&#231;&#227;o</a>";
    h += "<a href='/calibrar' class='" + String(paginaAtiva=="cal"?"a":"")  + "'>&#9878; Calibra&#231;&#227;o</a>";
  } else {
    h += "<a href='/login?next=config'   class='lock' title='Requer login'>&#128274; Configura&#231;&#227;o</a>";
    h += "<a href='/login?next=calibrar' class='lock' title='Requer login'>&#128274; Calibra&#231;&#227;o</a>";
  }
  h += "<div class='nav-right'>";
  if (auth) {
    h += "<span class='pill'>&#10003; Admin</span>";
    h += "<a href='/logout' style='font-size:11px;color:var(--muted)'>Sair</a>";
  }
  h += "</div></nav><main>";
  return h;
}

String htmlFooter() {
  return "</main><footer>SoroCare Monitor &bull; v" FW_VERSION
         " &bull; ESP32 IoT &bull; Btn GPIO" + String(BTN_CONFIRMA) +
         "</footer></body></html>";
}

// ════════════════════════════════════════════════════════════
//  PÁGINA DE LOGIN
// ════════════════════════════════════════════════════════════
void handleLoginPage() {
  bool erro = server.hasArg("erro");
  String next = server.hasArg("next") ? server.arg("next") : "";

  String html = "<!DOCTYPE html><html lang='pt-BR'><head>"
                "<meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>Login — SoroCare</title>";
  html += cssBase();
  html += F("<style>"
    ".login-wrap{min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}"
    ".login-box{background:var(--card);border:1px solid var(--border);border-radius:12px;"
    "padding:32px 28px;width:100%;max-width:360px}"
    ".login-logo{text-align:center;margin-bottom:24px}"
    ".login-logo .lp{width:60px;height:60px;border-radius:12px;background:var(--accent2);"
    "display:inline-flex;align-items:center;justify-content:center;"
    "font-size:22px;font-weight:700;color:#fff;margin-bottom:10px}"
    ".login-logo h2{font-size:16px;color:var(--accent)}"
    ".login-logo p{font-size:11px;color:var(--muted)}"
    ".err-box{background:#f4433618;border:1px solid var(--red);color:var(--red);"
    "border-radius:7px;padding:9px 12px;font-size:12px;margin-bottom:14px;text-align:center}"
    "</style></head><body><div class='login-wrap'><div class='login-box'>");

  html += "<div class='login-logo'>";
  if (strlen(cfg.logoUrl) > 0) {
    html += "<img src='" + String(cfg.logoUrl) + "' style='max-height:60px;border-radius:8px;margin-bottom:10px'>";
  } else {
    String ini = String(cfg.nomeEmpresa).substring(0,2); ini.toUpperCase();
    html += "<div class='lp'>" + ini + "</div>";
  }
  html += "<h2>" + String(cfg.nomeEmpresa) + "</h2>";
  html += "<p>Acesso restrito &mdash; administradores</p></div>";

  if (erro) html += "<div class='err-box'>&#10006; Usu&aacute;rio ou senha incorretos</div>";

  html += "<form method='POST' action='/login'>";
  if (next.length() > 0) html += "<input type='hidden' name='next' value='" + next + "'>";
  html += F("<div class='fg'><label>Usu&#225;rio</label>"
    "<input type='text' name='user' autocomplete='username' autofocus required></div>"
    "<div class='fg'><label>Senha</label>"
    "<input type='password' name='pass' autocomplete='current-password' required></div>"
    "<button type='submit' class='btn bp' style='width:100%;justify-content:center;margin-top:4px'>"
    "&#128275; Entrar</button>"
    "</form></div></div></body></html>");

  server.send(200, "text/html; charset=utf-8", html);
}

void handleLogin() {
  String user = server.hasArg("user") ? server.arg("user") : "";
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  String next = server.hasArg("next") ? server.arg("next") : "";
  if (verificarAuth(user, pass)) {
    gerarToken();
    server.sendHeader("Set-Cookie", "sc_sess=" + sessionToken + "; Path=/; Max-Age=3600");
    server.sendHeader("Location", (next.length() > 0) ? "/" + next : "/config", true);
    server.send(302, "text/plain", "");
    Serial.println("Login OK: " + user);
  } else {
    Serial.println("Login FALHOU: " + user);
    String n2 = (next.length() > 0) ? "?erro=1&next=" + next : "?erro=1";
    server.sendHeader("Location", "/login" + n2, true);
    server.send(302, "text/plain", "");
  }
}

void handleLogout() {
  sessionToken = "";
  server.sendHeader("Set-Cookie", "sc_sess=; Path=/; Max-Age=0");
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ════════════════════════════════════════════════════════════
//  DASHBOARD
// ════════════════════════════════════════════════════════════
void handleRoot() {
  String html = htmlHeader("dash");

  // Banner de alarme — mostra botão de confirmação grande quando soro vazio
  html += F("<div id='abanner' style='display:none'></div>");

  html += F("<div class='metrics'>"
    "<div class='met' id='cp'><label>Peso Atual</label>"
    "<div class='v' id='vp'>--</div><div class='u'>gramas</div></div>"
    "<div class='met' id='cv'><label>Volume Rest.</label>"
    "<div class='v' id='vv'>--</div><div class='u'>mL</div></div>"
    "<div class='met' id='cg'><label>Taxa Gotas</label>"
    "<div class='v' id='vg'>--</div><div class='u'>gts/min</div></div>"
    "<div class='met' id='ct'><label>Tempo Rest.</label>"
    "<div class='v' id='vt'>--</div><div class='u'>minutos</div></div>"
    "<div class='met'><label>Total Gotas</label>"
    "<div class='v' id='vgt'>--</div><div class='u'>desde o in&#237;cio</div></div>"
    "</div>");

  html += F("<div class='sc'><h3>&#128167; N&#237;vel do Soro</h3>"
    "<div style='display:flex;justify-content:space-between;font-size:11px;color:var(--muted);margin-bottom:4px'>"
    "<span id='pct-lbl'>--</span><span id='vol-tot'>-- mL</span></div>"
    "<div class='bar-bg'><div class='bar-f' id='nbar' style='width:0%'></div></div></div>");

  html += "<div class='sc'><h3>&#128203; Paciente</h3>";
  html += "<div class='irow'><span>Paciente</span><strong>" + String(cfg.nomePaciente) + "</strong></div>";
  html += "<div class='irow'><span>Leito</span><strong>" + String(cfg.nomeLeito) + "</strong></div>";
  html += "<div class='irow'><span>Taxa Prescrita</span><strong>" + String(cfg.taxaGotas,0) + " gts/min</strong></div>";
  html += F("<div class='irow'><span>MQTT</span><strong id='dmqtt'>--</strong></div>"
    "<div class='irow'><span>IP</span><strong id='dip'>--</strong></div>"
    "<div class='irow'><span>Status Soro</span><strong id='dstatus'>--</strong></div></div>");

  html += F("<div id='msg-dash' class='amsg'></div>"
    "<div style='display:flex;gap:8px;flex-wrap:wrap'>"
    "<button class='btn bg bs' onclick='tararBalanca()'>&#9878; Zerar Balan&#231;a</button>"
    "</div>");

  // JavaScript do dashboard com lógica de banner dinâmico
  html += F("<script>"
    "function apiCmd(c){"
      "fetch('/api/'+c).then(r=>r.json()).then(d=>{"
      "let el=document.getElementById('msg-dash');"
      "el.textContent=d.msg;el.className='amsg '+(d.ok?'ok':'err');"
      "el.style.display='block';setTimeout(()=>el.style.display='none',3000);"
      "setTimeout(atualizar,500);});}"

    "function tararBalanca(){"
      "let btn=event.target;btn.disabled=true;btn.textContent='Zerando...';"
      "fetch('/api/tare').then(r=>r.json()).then(d=>{"
      "let el=document.getElementById('msg-dash');"
      "el.textContent=d.msg;el.className='amsg '+(d.ok?'ok':'err');"
      "el.style.display='block';setTimeout(()=>el.style.display='none',3000);"
      "btn.disabled=false;btn.textContent='\\u2696\\uFE0F Zerar Balan\\u00E7a';"
      "setTimeout(atualizar,800);});}"

    "function confirmarSoro(){"
      "fetch('/api/confirmar/soro').then(r=>r.json()).then(d=>{"
      "let el=document.getElementById('msg-dash');"
      "el.textContent=d.msg;el.className='amsg '+(d.ok?'ok':'err');"
      "el.style.display='block';setTimeout(()=>el.style.display='none',4000);"
      "setTimeout(atualizar,500);});}"

    "function atualizarBanner(d){"
      "let ab=document.getElementById('abanner');"
      "if(d.alarme && !d.soro_confirmado){"
        // Alarme ativo aguardando confirmação — mostra botão grande
        "ab.className='abanner alarme';"
        "ab.style.display='flex';"
        "ab.innerHTML='<span style=\"font-size:22px\">&#128680;</span>'"
        "+'<div style=\"flex:1\"><strong style=\"font-size:14px\">'+d.alarme_msg+'</strong><br>'"
        "+'<small style=\"color:var(--muted)\">Enfermeira: v\\u00E1 ao leito e pressione o bot\\u00E3o f\\u00EDsico ou clique abaixo</small></div>'"
        "+'<button class=\"btn-confirmar\" onclick=\"confirmarSoro()\">&#10003; CONFIRMAR PRESEN\\u00C7A</button>';"
      "}else if(d.alarme && d.soro_confirmado){"
        // Confirmado — apenas informa
        "ab.className='abanner confirmado';"
        "ab.style.display='flex';"
        "ab.innerHTML='<span style=\"font-size:18px\">&#10003;</span>'"
        "+'<div><strong>Presen\\u00E7a confirmada no leito</strong><br>'"
        "+'<small style=\"color:var(--muted)\">Aguardando troca do soro</small></div>';"
      "}else{"
        "ab.style.display='none';"
        "ab.innerHTML='';"
      "}"
    "}"

    "function atualizar(){"
      "fetch('/api/dados').then(r=>r.json()).then(d=>{"
      "let peso=parseFloat(d.peso_g)||0;"
      "document.getElementById('vp').textContent=peso.toFixed(1);"
      "let vol=parseFloat(d.volume_ml)||0;"
      "document.getElementById('vv').textContent=vol.toFixed(0);"
      "document.getElementById('vg').textContent=parseFloat(d.gotas_min).toFixed(0);"
      "let t=parseInt(d.tempo_rest)||0;"
      "document.getElementById('vt').textContent=t>0?t:'--';"
      "document.getElementById('vgt').textContent=d.gotas_total;"
      "let pct=Math.min(100,Math.max(0,parseFloat(d.pct)||0));"
      "let bar=document.getElementById('nbar');"
      "bar.style.width=pct+'%';"
      "bar.className='bar-f'+(pct<15?' low':pct<30?' warn':'');"
      "document.getElementById('pct-lbl').textContent=pct.toFixed(0)+'% do volume total';"
      "document.getElementById('vol-tot').textContent=vol.toFixed(0)+' / '+d.vol_total+' mL';"
      "document.getElementById('cp').className='met'+(peso>5&&peso<80?' e':peso<150?' w':'');"
      "document.getElementById('cv').className='met'+(pct<15?' e':pct<30?' w':'');"
      "document.getElementById('dmqtt').textContent=d.mqtt?'&#10003; Conectado':'&#10006; Desconectado';"
      "document.getElementById('dip').textContent=d.ip;"
      "document.getElementById('sdot').className='sdot'+(d.wifi_ok?'':' off');"
      "let st=document.getElementById('dstatus');"
      "if(d.soro_confirmado)st.textContent='Presen\\u00E7a confirmada';"
      "else if(d.alarme)st.textContent='AGUARDANDO CONFIRMA\\u00C7\\u00C3O';"
      "else st.textContent='Normal';"
      "atualizarBanner(d);"
      "}).catch(()=>{});}"

    "setInterval(atualizar,1500);atualizar();"
    "</script>");

  html += htmlFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

// ════════════════════════════════════════════════════════════
//  CONFIGURAÇÃO (protegida)
// ════════════════════════════════════════════════════════════
void handleConfigPage() {
  if (!estaAutenticado()) { redirecionarLogin(); return; }

  String html = htmlHeader("conf");
  html += F("<div id='mc' class='amsg'></div><form method='POST' action='/config'>");

  html += F("<div class='sc'><h3>&#127970; Empresa &amp; Apar&#234;ncia</h3>"
    "<div class='fg'><label>Nome da Empresa</label>"
    "<input type='text' name='nomeEmpresa' maxlength='64' value='");
  html += cfg.nomeEmpresa;
  html += F("'></div>"
    "<div class='fg'><label>URL da Logo (https://...)</label>"
    "<input type='url' name='logoUrl' maxlength='128' "
    "placeholder='https://sua-empresa.com/logo.png' value='");
  html += cfg.logoUrl;
  html += F("'><p style='font-size:10px;color:var(--muted);margin-top:3px'>URL p&#250;blica PNG/JPG</p></div></div>");

  html += F("<div class='sc'><h3>&#128225; Rede WiFi</h3>"
    "<div class='fg'><label>SSID</label>"
    "<input type='text' name='ssid' maxlength='64' required value='");
  html += cfg.ssid;
  html += F("'></div>"
    "<div class='fg'><label>Senha WiFi</label>"
    "<input type='password' name='senha' maxlength='64' placeholder='Deixe em branco para manter'></div></div>");

  html += F("<div class='sc'><h3>&#128268; Servidor MQTT</h3><div class='fr'>"
    "<div class='fg'><label>Host / IP</label>"
    "<input type='text' name='mqttHost' maxlength='64' value='");
  html += cfg.mqttHost;
  html += F("'></div><div class='fg'><label>Porta</label>"
    "<input type='number' name='mqttPort' min='1' max='65535' value='");
  html += cfg.mqttPort;
  html += F("'></div></div><div class='fr'>"
    "<div class='fg'><label>Usu&#225;rio MQTT</label>"
    "<input type='text' name='mqttUser' maxlength='32' value='");
  html += cfg.mqttUser;
  html += F("'></div><div class='fg'><label>Senha MQTT</label>"
    "<input type='password' name='mqttPass' maxlength='32'></div></div>"
    "<div class='fg'><label>T&#243;pico Base</label>"
    "<input type='text' name='mqttTopico' maxlength='64' value='");
  html += cfg.mqttTopico;
  html += F("'></div></div>");

  html += F("<div class='sc'><h3>&#127973; Paciente &amp; Alarmes</h3><div class='fr'>"
    "<div class='fg'><label>Nome do Paciente</label>"
    "<input type='text' name='nomePaciente' maxlength='64' value='");
  html += cfg.nomePaciente;
  html += F("'></div><div class='fg'><label>Leito</label>"
    "<input type='text' name='nomeLeito' maxlength='32' value='");
  html += cfg.nomeLeito;
  html += F("'></div></div><div class='fr'>"
    "<div class='fg'><label>Volume Total (mL)</label>"
    "<input type='number' name='volumeTotal' min='50' max='5000' step='50' value='");
  html += (int)cfg.volumeTotal;
  html += F("'></div><div class='fg'><label>Taxa Prescrita (gts/min)</label>"
    "<input type='number' name='taxaGotas' min='1' max='300' value='");
  html += (int)cfg.taxaGotas;
  html += F("'></div></div><div class='fr'>"
    "<div class='fg'><label>Peso Alarme Cr&#237;tico (g)</label>"
    "<input type='number' name='pesoVazio' step='5' min='0' value='");
  html += (int)cfg.pesoVazio;
  html += F("'></div><div class='fg'><label>Peso Aviso Precoce (g)</label>"
    "<input type='number' name='pesoAlerta' step='5' min='0' value='");
  html += (int)cfg.pesoAlerta;
  html += F("'></div></div></div>");

  html += F("<div class='sc'><h3>&#128274; Credenciais Admin</h3><div class='fr'>"
    "<div class='fg'><label>Usu&#225;rio Admin</label>"
    "<input type='text' name='adminUser' maxlength='32' value='");
  html += cfg.adminUser;
  html += F("'></div><div class='fg'><label>Nova Senha Admin</label>"
    "<input type='password' name='adminPass' maxlength='32' placeholder='Deixe em branco para manter'></div>"
    "</div><p style='font-size:10px;color:var(--muted)'>Alterar a senha encerra a sess&#227;o atual</p></div>");

  html += F("<button type='submit' class='btn bp'>&#128190; Salvar e Reiniciar</button></form>");
  html += htmlFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

// ════════════════════════════════════════════════════════════
//  CALIBRAÇÃO (protegida)
// ════════════════════════════════════════════════════════════
void handleCalibrarPage() {
  if (!estaAutenticado()) { redirecionarLogin(); return; }

  String html = htmlHeader("cal");
  html += F("<div class='sc'><h3>&#9878; Procedimento de Calibra&#231;&#227;o</h3>"
    "<p style='color:var(--muted);font-size:12px;margin-bottom:4px'>"
    "Fator atual: <strong id='fator-txt'>");
  html += String(cfg.fatorCalibracao, 4);
  html += F("</strong> &nbsp;|&nbsp; Offset (tare) salvo: <strong id='offset-txt'>");
  html += String(balanca.get_offset());
  html += F("</strong></p>"
    "<p style='color:var(--accent);font-size:11px;margin-bottom:12px'>"
    "&#128190; Calibra&#231;&#227;o salva na NVS — n&#227;o perde ap&#243;s reiniciar</p>"
    "<ol style='color:var(--muted);font-size:12px;line-height:2.2;padding-left:18px;margin-bottom:14px'>"
    "<li>Retire tudo da balan&#231;a</li>"
    "<li>Clique <strong style='color:var(--text)'>Zerar Balan&#231;a</strong> e aguarde</li>"
    "<li>Coloque um peso padr&#227;o conhecido (ex: 200g)</li>"
    "<li>Informe o valor e clique <strong style='color:var(--text)'>Calibrar</strong></li></ol>"
    "<div id='msg-cal' class='amsg'></div>"
    "<div style='display:flex;gap:8px;margin-bottom:14px'>"
    "<button class='btn bg bs' onclick='tararCal()'>&#9878; Zerar Balan&#231;a</button></div>"
    "<div class='fr'>"
    "<div class='fg'><label>Peso Padr&#227;o (g)</label>"
    "<input type='number' id='pp' placeholder='Ex: 200' min='10' step='0.1'></div>"
    "<div style='display:flex;align-items:flex-end;padding-bottom:1px'>"
    "<button class='btn bp' onclick='calibrar()'>&#128208; Calibrar</button></div></div></div>");

  html += F("<div class='sc'><h3>&#128202; Leitura em Tempo Real</h3>"
    "<div class='irow'><span>Peso medido</span>"
    "<strong id='cal-p' style='font-size:20px;color:var(--accent)'>-- g</strong></div>"
    "<div class='irow'><span>Fator de calibra&#231;&#227;o</span><strong id='cal-f'>");
  html += String(cfg.fatorCalibracao, 4);
  html += F("</strong></div>"
    "<div class='irow'><span>Offset (tare)</span><strong id='cal-off'>");
  html += String(balanca.get_offset());
  html += F("</strong></div>"
    "<div class='irow'><span>Status HX711</span><strong id='cal-st'>--</strong></div></div>");

  html += F("<script>"
    "function mostrarMsg(msg,ok){"
      "let el=document.getElementById('msg-cal');"
      "el.textContent=msg;el.className='amsg '+(ok?'ok':'err');"
      "el.style.display='block';setTimeout(()=>el.style.display='none',5000);}"

    "function tararCal(){"
      "let btn=event.target;btn.disabled=true;btn.textContent='Zerando...';"
      "fetch('/api/tare').then(r=>r.json()).then(d=>{"
      "mostrarMsg(d.msg,d.ok);"
      "btn.disabled=false;btn.textContent='\\u2696\\uFE0F Zerar Balan\\u00E7a';"
      "setTimeout(atuCal,800);});}"

    "function calibrar(){"
      "let v=document.getElementById('pp').value;"
      "if(!v||parseFloat(v)<=0){mostrarMsg('Informe um peso v\\u00E1lido!',false);return;}"
      "fetch('/api/calibrar',{method:'POST',"
      "headers:{'Content-Type':'application/x-www-form-urlencoded'},"
      "body:'peso='+v}).then(r=>r.json()).then(d=>{"
      "mostrarMsg(d.msg,d.ok);"
      "if(d.ok){"
      "document.getElementById('fator-txt').textContent=d.fator;"
      "document.getElementById('cal-f').textContent=d.fator;"
      "if(d.offset){document.getElementById('offset-txt').textContent=d.offset;"
      "document.getElementById('cal-off').textContent=d.offset;}}});}"

    "function atuCal(){"
      "fetch('/api/dados').then(r=>r.json()).then(d=>{"
      "let peso=parseFloat(d.peso_g)||0;"
      "document.getElementById('cal-p').textContent=peso.toFixed(2)+' g';"
      "document.getElementById('cal-f').textContent=d.fator;"
      "document.getElementById('cal-st').textContent='OK';"
      "document.getElementById('sdot').className='sdot'+(d.wifi_ok?'':' off');"
      "}).catch(()=>{document.getElementById('cal-st').textContent='Erro';});}"

    "setInterval(atuCal,1000);atuCal();"
    "</script>");

  html += htmlFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

// ════════════════════════════════════════════════════════════
//  API /api/dados
// ════════════════════════════════════════════════════════════
void handleApiDados() {
  float pct = (cfg.volumeTotal > 0) ? (volumeRestante / cfg.volumeTotal * 100.0f) : 0;
  StaticJsonDocument<512> doc;
  doc["peso_g"]          = String(pesoAtual, 2);
  doc["volume_ml"]       = String(volumeRestante, 1);
  doc["gotas_min"]       = String(taxaGotasAtual, 1);
  doc["gotas_total"]     = (unsigned long)contadorGotas;
  doc["tempo_rest"]      = String(tempoRestante, 0);
  doc["pct"]             = pct;
  doc["vol_total"]       = (int)cfg.volumeTotal;
  doc["alarme"]          = alarmeAtivo;
  doc["alarme_msg"]      = alarmeAtivo ? "VERIFICAR SORO" : "";
  doc["soro_confirmado"] = soroVazioConfirmado;
  doc["mqtt"]            = mqttConectado;
  doc["wifi_ok"]         = (WiFi.status() == WL_CONNECTED);
  doc["ip"]              = WiFi.localIP().toString();
  doc["rssi"]            = WiFi.RSSI();
  doc["uptime"]          = millis() / 1000;
  doc["heap"]            = ESP.getFreeHeap();
  doc["fator"]           = String(cfg.fatorCalibracao, 4);
  char buf[512];
  serializeJson(doc, buf, sizeof(buf));
  server.send(200, "application/json", buf);
}

// ════════════════════════════════════════════════════════════
//  API /api/tare — agenda tare no loop (salva offset na NVS)
// ════════════════════════════════════════════════════════════
void handleTare() {
  tararePendente = true;
  server.send(200, "application/json",
    "{\"ok\":true,\"msg\":\"Zerando... aguarde 3s — offset sera salvo automaticamente\"}");
}

// ════════════════════════════════════════════════════════════
//  API /api/confirmar/soro — equivale ao botão físico via web
// ════════════════════════════════════════════════════════════
void handleConfirmarSoro() {
  silenciarAlarme("WEB_BUTTON");
  server.send(200, "application/json",
    "{\"ok\":true,\"msg\":\"Presenca confirmada — alarme silenciado\"}");
}

void handleAlarmeOff() {
  silenciarAlarme("WEB_API");
  server.send(200, "application/json",
    "{\"ok\":true,\"msg\":\"Alarme silenciado\"}");
}

void handleReset() {
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Reiniciando...\"}");
  delay(300);
  ESP.restart();
}

// ════════════════════════════════════════════════════════════
//  API /api/calibrar (protegida) — salva fator E offset na NVS
// ════════════════════════════════════════════════════════════
void handleCalibrar() {
  if (!estaAutenticado()) {
    server.send(403, "application/json", "{\"ok\":false,\"msg\":\"Sem permissao\"}");
    return;
  }
  if (!server.hasArg("peso")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Falta o peso\"}");
    return;
  }
  float pesoReal = server.arg("peso").toFloat();
  if (pesoReal <= 0) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Peso invalido\"}");
    return;
  }
  if (!balanca.is_ready()) {
    server.send(500, "application/json", "{\"ok\":false,\"msg\":\"HX711 nao pronto\"}");
    return;
  }
  balanca.set_scale(1.0f);
  float raw = balanca.get_value(20);
  cfg.fatorCalibracao = raw / pesoReal;
  balanca.set_scale(cfg.fatorCalibracao);

  Serial.printf("[CAL] peso=%.1fg  raw=%.1f  fator=%.4f  offset=%ld\n",
                pesoReal, raw, cfg.fatorCalibracao, balanca.get_offset());

  // Salva fator E offset na NVS
  salvarCalibracaoNVS();

  long offset = balanca.get_offset();
  String resp = "{\"ok\":true,\"msg\":\"Calibrado e salvo na NVS! Fator=" +
                String(cfg.fatorCalibracao, 4) + "\",\"fator\":\"" +
                String(cfg.fatorCalibracao, 4) + "\",\"offset\":" +
                String(offset) + "}";
  server.send(200, "application/json", resp);
}

// ════════════════════════════════════════════════════════════
//  POST /config (protegida)
// ════════════════════════════════════════════════════════════
void handleSaveConfig() {
  if (!estaAutenticado()) { redirecionarLogin(); return; }
  if (server.hasArg("nomeEmpresa"))  strlcpy(cfg.nomeEmpresa,  server.arg("nomeEmpresa").c_str(),  64);
  if (server.hasArg("logoUrl"))      strlcpy(cfg.logoUrl,      server.arg("logoUrl").c_str(),      128);
  if (server.hasArg("ssid"))         strlcpy(cfg.ssid,         server.arg("ssid").c_str(),         64);
  if (server.hasArg("senha") && server.arg("senha").length() > 0)
    strlcpy(cfg.senha, server.arg("senha").c_str(), 64);
  if (server.hasArg("mqttHost"))     strlcpy(cfg.mqttHost,     server.arg("mqttHost").c_str(),     64);
  if (server.hasArg("mqttPort"))     cfg.mqttPort   = server.arg("mqttPort").toInt();
  if (server.hasArg("mqttUser"))     strlcpy(cfg.mqttUser,     server.arg("mqttUser").c_str(),     32);
  if (server.hasArg("mqttPass") && server.arg("mqttPass").length() > 0)
    strlcpy(cfg.mqttPass, server.arg("mqttPass").c_str(), 32);
  if (server.hasArg("mqttTopico"))   strlcpy(cfg.mqttTopico,   server.arg("mqttTopico").c_str(),   64);
  if (server.hasArg("nomePaciente")) strlcpy(cfg.nomePaciente, server.arg("nomePaciente").c_str(), 64);
  if (server.hasArg("nomeLeito"))    strlcpy(cfg.nomeLeito,    server.arg("nomeLeito").c_str(),    32);
  if (server.hasArg("volumeTotal"))  cfg.volumeTotal = server.arg("volumeTotal").toFloat();
  if (server.hasArg("taxaGotas"))    cfg.taxaGotas   = server.arg("taxaGotas").toFloat();
  if (server.hasArg("pesoVazio"))    cfg.pesoVazio   = server.arg("pesoVazio").toFloat();
  if (server.hasArg("pesoAlerta"))   cfg.pesoAlerta  = server.arg("pesoAlerta").toFloat();
  if (server.hasArg("adminUser") && server.arg("adminUser").length() > 0)
    strlcpy(cfg.adminUser, server.arg("adminUser").c_str(), 32);
  if (server.hasArg("adminPass") && server.arg("adminPass").length() > 0)
    strlcpy(cfg.adminPass, server.arg("adminPass").c_str(), 32);
  Serial.println("\n===== DADOS RECEBIDOS =====");
  Serial.print("SSID: ");
  Serial.println(cfg.ssid);
  Serial.print("Senha: ");
  Serial.println(cfg.senha);
  Serial.println("===========================\n");
  salvarConfig();
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
  delay(200);
  ESP.restart();
}

// ════════════════════════════════════════════════════════════
//  PERSISTÊNCIA NVS
// ════════════════════════════════════════════════════════════

// Salva fator de calibração E offset do tare — chamado após calibrar ou tarar
void salvarCalibracaoNVS() {
  prefs.begin("sorocare", false);
  prefs.putFloat("fatorCal", cfg.fatorCalibracao);
  prefs.putLong ("tareOffset", balanca.get_offset());
  prefs.end();
  Serial.printf("[NVS] Calibracao salva: fator=%.4f  offset=%ld\n",
                cfg.fatorCalibracao, balanca.get_offset());
}

// Restaura fator E offset do tare — chamado no setup
void lerCalibracaoNVS() {
  prefs.begin("sorocare", true);
  float fator  = prefs.getFloat("fatorCal",   -420.0f);
  long  offset = prefs.getLong ("tareOffset",  0);
  prefs.end();

  cfg.fatorCalibracao = fator;
  balanca.set_scale(fator);
  balanca.set_offset(offset);
  Serial.printf("[NVS] Restaurado: fator=%.4f  offset=%ld\n", fator, offset);
}

void salvarConfig() {
  prefs.begin("sorocare", false);
  prefs.putString("ssid",         cfg.ssid);
  prefs.putString("senha",        cfg.senha);
  Serial.println("\n===== NVS GRAVADA =====");
  Serial.print("SSID: ");
  Serial.println(cfg.ssid);
  Serial.print("Senha: ");
  Serial.println(cfg.senha);
  Serial.println("=======================\n");
  prefs.putString("mqttHost",     cfg.mqttHost);
  prefs.putInt   ("mqttPort",     cfg.mqttPort);
  prefs.putString("mqttUser",     cfg.mqttUser);
  prefs.putString("mqttPass",     cfg.mqttPass);
  prefs.putString("mqttTopico",   cfg.mqttTopico);
  prefs.putString("nomePaciente", cfg.nomePaciente);
  prefs.putString("nomeLeito",    cfg.nomeLeito);
  prefs.putFloat ("volumeTotal",  cfg.volumeTotal);
  prefs.putFloat ("taxaGotas",    cfg.taxaGotas);
  prefs.putFloat ("pesoVazio",    cfg.pesoVazio);
  prefs.putFloat ("pesoAlerta",   cfg.pesoAlerta);
  prefs.putFloat ("fatorCal",     cfg.fatorCalibracao);
  prefs.putLong  ("tareOffset",   balanca.get_offset()); // garante sincronismo
  prefs.putString("nomeEmpresa",  cfg.nomeEmpresa);
  prefs.putString("logoUrl",      cfg.logoUrl);
  prefs.putString("adminUser",    cfg.adminUser);
  prefs.putString("adminPass",    cfg.adminPass);
  prefs.end();
  Serial.println("[NVS] Config completa salva");
}

void carregarConfig() {
  prefs.begin("sorocare", true);
  if (prefs.isKey("ssid")) {
    strlcpy(cfg.ssid,         prefs.getString("ssid",         "").c_str(),              64);
    strlcpy(cfg.senha,        prefs.getString("senha",        "").c_str(),              64);
    strlcpy(cfg.mqttHost,     prefs.getString("mqttHost",     "179.125.32.238").c_str(),64);
    cfg.mqttPort            = prefs.getInt   ("mqttPort",     MQTT_PORT_DEFAULT);
    strlcpy(cfg.mqttUser,     prefs.getString("mqttUser",     "").c_str(),              32);
    strlcpy(cfg.mqttPass,     prefs.getString("mqttPass",     "").c_str(),              32);
    strlcpy(cfg.mqttTopico,   prefs.getString("mqttTopico",   "soro/leito01").c_str(),  64);
    strlcpy(cfg.nomePaciente, prefs.getString("nomePaciente", "Paciente").c_str(),      64);
    strlcpy(cfg.nomeLeito,    prefs.getString("nomeLeito",    "Leito 01").c_str(),      32);
    cfg.volumeTotal         = prefs.getFloat ("volumeTotal",  500.0f);
    cfg.taxaGotas           = prefs.getFloat ("taxaGotas",    60.0f);
    cfg.pesoVazio           = prefs.getFloat ("pesoVazio",    PESO_VAZIO_DEFAULT);
    cfg.pesoAlerta          = prefs.getFloat ("pesoAlerta",   PESO_ALERTA_DEFAULT);
    cfg.fatorCalibracao     = prefs.getFloat ("fatorCal",     -420.0f);
    strlcpy(cfg.nomeEmpresa, prefs.getString("nomeEmpresa",  "NexulTech").c_str(),       64);
    strlcpy(cfg.logoUrl,     prefs.getString("logoUrl",      "").c_str(),               128);
    strlcpy(cfg.adminUser,   prefs.getString("adminUser",    ADMIN_USER_DEFAULT).c_str(), 32);
    strlcpy(cfg.adminPass,   prefs.getString("adminPass",    ADMIN_PASS_DEFAULT).c_str(), 32);
    Serial.println("[NVS] Config carregada");
    Serial.println("===== NVS LIDA =====");
    Serial.print("SSID: ");
    Serial.println(cfg.ssid);
    Serial.print("Senha: ");
    Serial.println(cfg.senha);
    Serial.println("====================");  
    } else {
  Serial.println("[NVS] Primeira execucao — usando valores padrao");

  // Wi-Fi padrão
  strlcpy(cfg.ssid,  "CLARO_2G600601", sizeof(cfg.ssid));
  strlcpy(cfg.senha, "96600601", sizeof(cfg.senha));

  // Salva na NVS para as próximas inicializações
  salvarConfig();
}
  prefs.end();
  gerarToken();
}

// ════════════════════════════════════════════════════════════
//  UTILITÁRIOS
// ════════════════════════════════════════════════════════════
void bipCurto(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(80);
    digitalWrite(BUZZER_PIN, LOW);  delay(80);
  }
}

void piscaLed(int pin, unsigned long intervalo) {
  unsigned long agora = millis();
  if (agora - tUltBlink > intervalo) {
    tUltBlink  = agora;
    blinkState = !blinkState;
    digitalWrite(pin, blinkState);
  }
}
