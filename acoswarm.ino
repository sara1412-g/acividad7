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
