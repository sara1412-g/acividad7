/*
  Enjambre de 3 carritos con ACO en ESP32
  ---------------------------------------
  Cada ESP32 corre su propio algoritmo de colonia de hormigas sobre el laberinto,
  manda su mejor ruta por UDP (broadcast) y refuerza sus feromonas con las rutas
  que reciben de los otros nodos.

  - NODE_ID 0: crea la red WiFi (modo AP).
  - NODE_ID 1 y 2: se conectan a esa red como estacion.

  Mensaje UDP (texto):  "id,iteracion,largo,ruta"   ej: "1,12,20,RRDDLL..."

  Los parametros ALPHA, BETA, RHO, Q, ANTS y el laberinto deben coincidir con
  aco.py y maze.py.
*/
#include <WiFi.h>
#include <WiFiUdp.h>

// ------------------------------------------------------------ configuracion
#define NODE_ID 0                  // 0, 1 o 2 (cambia antes de cargar cada placa)

const char* SSID = "ACO_SWARM";
const char* PASS = "aco12345";     // minimo 8 caracteres
const uint16_t UDP_PORT = 4210;
const IPAddress BCAST(192, 168, 4, 255);

// ------------------------------------------------------------ laberinto
const int R = 11;
const int C = 11;
const char* MAZE[R] = {
  "###########",
  "#S........#",
  "#.##.####.#",
  "#....#....#",
  "#.####.##.#",
  "#.#......##",
  "#.#.####..#",
  "#...#..#.##",
  "#.###.##..#",
  "#.....#..G#",
  "###########"
};

// ------------------------------------------------------------ parametros ACO
const float ALPHA = 1.0;
const float BETA = 1.0;
const float RHO = 0.10;
const float Q = 10.0;
const int ANTS = 4;
const float TAU_MIN = 0.05;
const float TAU_MAX = 20.0;
const int MAX_STEPS = 60;

const char DIRS[4] = {'U', 'D', 'L', 'R'};
const int DR[4] = {-1, 1, 0, 0};
const int DC[4] = {0, 0, -1, 1};

// ------------------------------------------------------------ estado
float tau[R][C];
int startR = 0, startC = 0, goalR = 0, goalC = 0;
char bestPath[MAX_STEPS + 1] = "";
int bestLen = 1000000;
unsigned long iteration = 0;
WiFiUDP udp;

// ------------------------------------------------------------ utilidades
bool isOpen(int r, int c) {
  return r >= 0 && r < R && c >= 0 && c < C && MAZE[r][c] != '#';
}

float frand() {
  return (float)(esp_random() % 100000) / 100000.0f;
}

void findPoints() {
  for (int r = 0; r < R; r++)
    for (int c = 0; c < C; c++) {
      if (MAZE[r][c] == 'S') { startR = r; startC = c; }
      if (MAZE[r][c] == 'G') { goalR = r; goalC = c; }
    }
}

// Deposita feromona a lo largo de una ruta. Devuelve false si la ruta no es valida.
bool deposit(const char* path, int len, float amount) {
  int r = startR, c = startC;
  // primero validar
  for (int i = 0; i < len; i++) {
    int d = -1;
    for (int k = 0; k < 4; k++) if (DIRS[k] == path[i]) d = k;
    if (d < 0) return false;
    r += DR[d]; c += DC[d];
    if (!isOpen(r, c)) return false;
  }
  r = startR; c = startC;
  for (int i = 0; i < len; i++) {
    int d = 0;
    for (int k = 0; k < 4; k++) if (DIRS[k] == path[i]) d = k;
    r += DR[d]; c += DC[d];
    tau[r][c] = min(TAU_MAX, tau[r][c] + amount);
  }
  return true;
}

// ------------------------------------------------------------ una hormiga
bool runAnt(char* out, int& len) {
  bool visited[R][C];
  memset(visited, 0, sizeof(visited));
  int r = startR, c = startC;
  visited[r][c] = true;
  len = 0;
  while (!(r == goalR && c == goalC)) {
    if (len >= MAX_STEPS) return false;
    float w[4];
    float total = 0;
    for (int d = 0; d < 4; d++) {
      int nr = r + DR[d], nc = c + DC[d];
      w[d] = 0;
      if (isOpen(nr, nc) && !visited[nr][nc]) {
        float eta = 1.0f / (1 + abs(nr - goalR) + abs(nc - goalC));
        w[d] = powf(tau[nr][nc], ALPHA) * powf(eta, BETA);
        total += w[d];
      }
    }
    if (total <= 0) return false;            // hormiga atrapada
    float pick = frand() * total, acc = 0;
    int chosen = 0;
    for (int d = 0; d < 4; d++) {
      if (w[d] <= 0) continue;
      chosen = d;
      acc += w[d];
      if (pick <= acc) break;
    }
    out[len++] = DIRS[chosen];
    r += DR[chosen]; c += DC[chosen];
    visited[r][c] = true;
  }
  out[len] = '\0';
  return true;
}

// ------------------------------------------------------------ una iteracion
void iterate() {
  iteration++;
  char found[ANTS][MAX_STEPS + 1];
  int lens[ANTS];
  int n = 0;
  for (int a = 0; a < ANTS; a++) {
    int len;
    if (runAnt(found[n], len)) { lens[n] = len; n++; }
  }
  // evaporacion
  for (int r = 0; r < R; r++)
    for (int c = 0; c < C; c++)
      tau[r][c] = max(TAU_MIN, tau[r][c] * (1.0f - RHO));
  // deposito
  for (int i = 0; i < n; i++) {
    deposit(found[i], lens[i], Q / lens[i]);
    if (lens[i] < bestLen) {
      bestLen = lens[i];
      strcpy(bestPath, found[i]);
    }
  }
  // refuerzo elitista
  if (bestLen < 1000000) deposit(bestPath, bestLen, Q / bestLen);
}

// ------------------------------------------------------------ red
void sendBest() {
  if (bestLen >= 1000000) return;
  char msg[128];
  snprintf(msg, sizeof(msg), "%d,%lu,%d,%s", NODE_ID, iteration, bestLen, bestPath);
  udp.beginPacket(BCAST, UDP_PORT);
  udp.print(msg);
  udp.endPacket();
}

void receiveOthers() {
  int size;
  while ((size = udp.parsePacket()) > 0) {
    char buf[128];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = '\0';
    // formato: id,iteracion,largo,ruta
    char* p1 = strchr(buf, ',');           if (!p1) continue;
    char* p2 = strchr(p1 + 1, ',');        if (!p2) continue;
    char* p3 = strchr(p2 + 1, ',');        if (!p3) continue;
    int id = atoi(buf);
    int len = atoi(p2 + 1);
    char* path = p3 + 1;
    if (id == NODE_ID || len <= 0 || len > MAX_STEPS || (int)strlen(path) != len) continue;
    if (deposit(path, len, Q / len) && len < bestLen) {
      bestLen = len;
      strcpy(bestPath, path);
    }
  }
}

void startNetwork() {
  if (NODE_ID == 0) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASS);
    Serial.print("AP creado. IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    Serial.print("Conectando a " + String(SSID));
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print("\nConectado. IP: ");
    Serial.println(WiFi.localIP());
  }
  udp.begin(UDP_PORT);
}

// ------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(500);
  findPoints();
  for (int r = 0; r < R; r++)
    for (int c = 0; c < C; c++) tau[r][c] = 1.0f;
  startNetwork();
  Serial.printf("Nodo %d listo\n", NODE_ID);
}

void loop() {
  iterate();
  receiveOthers();
  sendBest();
  if (iteration % 10 == 0)
    Serial.printf("iter %lu | mejor ruta: %d pasos\n", iteration, bestLen);
  delay(250);
}
#include <WiFi.h>
#include <WiFiUdp.h>

#define NODE_ID 0

const char*    WIFI_SSID = "ACO_SWARM";
const char*    WIFI_PASS = "aco12345";
const uint16_t PORT      = 4210;
const IPAddress BCAST(192, 168, 4, 255);

const int R = 10, C = 10, N = R * C;
const char* MAZE[R] = {
  "S.........",
  "..#...#.#.",
  "#.#.....#.",
  "#.........",
  "...#..#...",
  "..#..#....",
  "...#.#....",
  ".##.....#.",
  ".....#..#.",
  ".........G"
};
const int DR[4] = {-1, 0, 1, 0};
const int DC[4] = {0, 1, 0, -1};
int START = 0, GOAL = 0;

const float ALPHA = 1.0, BETA = 1.0, RHO = 0.10, Q = 10.0;
const int   ANTS = 10;
const unsigned long PERIOD_MS = 500;

float tau[N][4];
int   bestPath[N + 1], bestLen = 9999;
unsigned long iterN = 0, lastT = 0;
WiFiUDP udp;

int neighbor(int i, int d) {
  int r = i / C + DR[d], c = i % C + DC[d];
  if (r < 0 || r >= R || c < 0 || c >= C || MAZE[r][c] == '#') return -1;
  return r * C + c;
}

int runAnt(int* path) {
  bool vis[N] = {false};
  int cur = START, n = 0;
  path[n++] = cur; vis[cur] = true;
  int gr = GOAL / C, gc = GOAL % C;
  while (cur != GOAL) {
    int nb[4]; float w[4]; int k = 0; float sum = 0;
    for (int d = 0; d < 4; d++) {
      int x = neighbor(cur, d);
      if (x < 0 || vis[x]) continue;
      float eta = 1.0f / (1 + abs(x / C - gr) + abs(x % C - gc));
      nb[k] = x; w[k] = powf(tau[cur][d], ALPHA) * powf(eta, BETA);
      sum += w[k]; k++;
    }
    if (k == 0) return -1;
    float r = (random(0, 10000) / 10000.0f) * sum, acc = 0; int pick = k - 1;
    for (int i = 0; i < k; i++) { acc += w[i]; if (r <= acc) { pick = i; break; } }
    cur = nb[pick]; path[n++] = cur; vis[cur] = true;
  }
  return n - 1;
}

void deposit(const int* path, int len, float amt) {
  for (int i = 0; i < len; i++) {
    int a = path[i], b = path[i + 1];
    for (int d = 0; d < 4; d++) if (neighbor(a, d) == b) {
      tau[a][d] = min(50.0f, tau[a][d] + amt);
      tau[b][(d + 2) % 4] = min(50.0f, tau[b][(d + 2) % 4] + amt);
      break;
    }
  }
}

void evaporate() {
  for (int i = 0; i < N; i++) for (int d = 0; d < 4; d++)
    tau[i][d] = max(0.05f, tau[i][d] * (1 - RHO));
}

void netBegin() {
#if NODE_ID == 0
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("AP listo, IP: "); Serial.println(WiFi.softAPIP());
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando al AP");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print('.'); }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());
#endif
  udp.begin(PORT);
}

void netTx(const int* path, int len) {
  char buf[600];
  int n = snprintf(buf, sizeof(buf), "P;%d;%lu;%d;", NODE_ID, iterN, len);
  for (int i = 0; i <= len && n < (int)sizeof(buf) - 6; i++)
    n += (i == 0) ? snprintf(buf + n, sizeof(buf) - n, "%d", path[i])
                  : snprintf(buf + n, sizeof(buf) - n, ",%d", path[i]);
  udp.beginPacket(BCAST, PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
}

void netRx() {
  while (udp.parsePacket() > 0) {
    char buf[600];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    if (buf[0] != 'P') continue;
    char *f[5], *sv1, *sv2; int k = 0;
    for (char* t = strtok_r(buf, ";", &sv1); t && k < 5; t = strtok_r(NULL, ";", &sv1)) f[k++] = t;
    if (k < 5) continue;
    int id = atoi(f[1]), len = atoi(f[3]);
    if (id == NODE_ID || len <= 0 || len >= N) continue;
    int path[N + 1], m = 0;
    for (char* t = strtok_r(f[4], ",", &sv2); t && m <= N; t = strtok_r(NULL, ",", &sv2)) {
      int v = atoi(t); if (v < 0 || v >= N) { m = -1; break; } path[m++] = v;
    }
    if (m == len + 1) deposit(path, len, Q / len);
  }
}

void onBestPath(const int* path, int len) {
  Serial.printf("[nodo %d] mejor ruta: %d pasos\n", NODE_ID, len);
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  randomSeed(esp_random());
  for (int i = 0; i < R * C; i++) {
    if (MAZE[i / C][i % C] == 'S') START = i;
    if (MAZE[i / C][i % C] == 'G') GOAL = i;
    for (int d = 0; d < 4; d++) tau[i][d] = 1.0f;
  }
  netBegin();
}

void loop() {
#if NODE_ID != 0
  if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
#endif
  netRx();
  if (millis() - lastT < PERIOD_MS) return;
  lastT = millis();

  int iterBest[N + 1], iterLen = -1, tmp[N + 1];
  for (int a = 0; a < ANTS; a++) {
    int len = runAnt(tmp);
    if (len > 0 && (iterLen < 0 || len < iterLen)) { iterLen = len; memcpy(iterBest, tmp, sizeof(int) * (len + 1)); }
  }
  evaporate();
  if (iterLen > 0) {
    deposit(iterBest, iterLen, Q / iterLen);
    netTx(iterBest, iterLen);
    digitalWrite(2, !digitalRead(2));
    if (iterLen < bestLen) { bestLen = iterLen; memcpy(bestPath, iterBest, sizeof(int) * (iterLen + 1)); onBestPath(bestPath, bestLen); }
  }
  iterN++;
}
