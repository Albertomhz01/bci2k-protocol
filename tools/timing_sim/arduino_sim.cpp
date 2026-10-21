// Native (Linux/macOS) build of arduino_pressure_controller.ino for timing
// tests WITHOUT hardware or participant. The .ino is compiled unchanged; this
// file supplies a minimal Arduino API and a physical model of pump + cuff +
// valve + MPX5050DP.
//
// Serial goes through stdin/stdout (the test harness connects them to a
// pseudo-terminal that pyserial opens like a COM port).
//
// Build:  g++ -O2 -std=c++17 -o arduino_sim arduino_sim.cpp
// Usage:  arduino_sim <truth_log.csv> [clock_error_ppm] [seed]
//
// Model (fitted to P001S001R02-R05: HIGH 111 mmHg ~7 s, LOW 46 mmHg ~3.6 s,
// NONE 8 mmHg ~0.3-0.6 s at PWM 170; HIGH deflation ~4.5 s):
//   pressure(V)  = 4*V + 0.0606*V^2           (cuff gets stiffer as it fills)
//   pump flow    = Q0 * speed * (1 - P/450)    (weaker against back pressure)
//   speed        = clamp((pwm - 75)/(170 - 75), 0, 1.6), 1st-order lag 60 ms
//                  (motor stalls below PWM 75 in this model)
//   vent flow    = 0.672 * sqrt(P)             (valve open)
//   leak         = 0.2 % of P per second       (valve closed)
//   Q0 varies ±4 % trial to trial (fit of the cuff on the arm, tubing).
// The model is NOT the real pump; it only gives a nonlinear, noisy plant with
// the right time scales so the control and timing logic can be exercised.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <poll.h>
#include <unistd.h>

// ------------------------------------------------------------------ clock
static const auto T_START = std::chrono::steady_clock::now();
static double g_clock_error_ppm = 0.0;   // Arduino resonator error (millis runs fast if > 0)

static double mono_s()   // same clock as Python's time.perf_counter() on Linux
{
  timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static double since_start_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - T_START).count();
}
unsigned long millis() { return (unsigned long)(since_start_s() * 1000.0 * (1.0 + g_clock_error_ppm * 1e-6)); }
void delay(unsigned long ms) { std::this_thread::sleep_for(std::chrono::microseconds(ms * 1000)); }

// ------------------------------------------------------------------ pins
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define A4 18
static int g_pin[32];
static int g_pwm9 = 0;
void pinMode(int, int) {}
void digitalWrite(int pin, int v) { g_pin[pin] = v; }
void analogWrite(int pin, int v) { if (pin == 9) g_pwm9 = v; }

// ------------------------------------------------------------------ physics
static std::mt19937 g_rng(1234);
static std::normal_distribution<double> g_n01(0.0, 1.0);
static double V = 0.0, speed = 0.0, Q0 = 3.41, Q0_nominal = 3.41;
static double last_phys = -1.0;
static int    last_valve = LOW;
static FILE  *g_truth = nullptr;
static double last_truth = 0.0;
static const double SENSOR_OFFSET_V = 0.012;   // real sensor offset differs from V_OFFSET (auto-zero test)

static double P_of_V(double v) { return v <= 0 ? 0.0 : 4.0 * v + 0.0606 * v * v; }

static void physics_step()
{
  double now = since_start_s();
  if (last_phys < 0) { last_phys = now; return; }
  int valve = g_pin[2];
  if (valve == HIGH && last_valve == LOW) {          // new trial: valve closes
    Q0 = Q0_nominal * (1.0 + 0.04 * g_n01(g_rng));
  }
  last_valve = valve;
  while (last_phys < now) {
    double dt = std::min(0.0005, now - last_phys);
    double target_speed = std::max(0.0, std::min(1.6, (g_pwm9 - 75.0) / (170.0 - 75.0)));
    speed += (target_speed - speed) * dt / 0.060;
    double P = P_of_V(V);
    double dV = Q0 * speed * std::max(0.0, 1.0 - P / 450.0);
    if (valve == LOW) dV -= 0.672 * std::sqrt(P) + 0.05 * (V > 0);   // vent (never retains)
    else              dV -= 0.002 * P / std::max(1e-6, (4.0 + 0.1212 * V));
    V = std::max(0.0, V + dV * dt);
    last_phys += dt;
  }
  if (g_truth && now - last_truth >= 0.005) {
    fprintf(g_truth, "T,%.6f,%.3f,%d,%d\n", mono_s(), P_of_V(V), g_pwm9, valve);
    last_truth = now;
  }
}

int analogRead(int)
{
  physics_step();
  double kpa  = P_of_V(V) / 7.50062;
  double volt = 0.190 + SENSOR_OFFSET_V + 0.085 * kpa + 0.003 * g_n01(g_rng);
  int adc = (int) std::lround(volt / 5.0 * 1023.0);
  if (adc < 0) adc = 0;
  if (adc > 1023) adc = 1023;
  // a real analogRead takes ~112 us; 32 of them set the loop period
  std::this_thread::sleep_for(std::chrono::microseconds(100));
  return adc;
}

// ------------------------------------------------------------------ String
class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String(const std::string &x) : s(x) {}
  bool startsWith(const char *p) const { return s.rfind(p, 0) == 0; }
  int indexOf(char c, int from = 0) const { auto i = s.find(c, from); return i == std::string::npos ? -1 : (int) i; }
  String substring(int a) const { return String(s.substr(a)); }
  String substring(int a, int b) const { return String(s.substr(a, b - a)); }
  float toFloat() const { return (float) atof(s.c_str()); }
  void trim() {
    size_t a = s.find_first_not_of(" \r\n\t"), b = s.find_last_not_of(" \r\n\t");
    s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
  }
  bool operator==(const char *o) const { return s == o; }
};

// ------------------------------------------------------------------ Serial
class SerialSim {
  std::string in, out;
  unsigned long timeout_ms = 1000;
  void pump_in() {
    pollfd p{0, POLLIN, 0};
    while (poll(&p, 1, 0) > 0 && (p.revents & POLLIN)) {
      char b[256]; ssize_t n = read(0, b, sizeof b);
      if (n <= 0) { if (n == 0) exit(0); break; }
      in.append(b, n);
    }
  }
  void emit(const std::string &x) {
    out += x;
    size_t nl;
    while ((nl = out.find('\n')) != std::string::npos) {
      std::string line = out.substr(0, nl + 1);
      out.erase(0, nl + 1);
      if (g_truth && line.rfind("P,", 0) != 0)
        fprintf(g_truth, "E,%.6f,%s", mono_s(), line.c_str());
      // 115200 baud: ~87 us per byte
      std::this_thread::sleep_for(std::chrono::microseconds(87 * line.size()));
      ssize_t r = write(1, line.data(), line.size()); (void) r;
    }
  }
 public:
  void begin(long) {}
  void setTimeout(unsigned long ms) { timeout_ms = ms; }
  int available() { pump_in(); return (int) in.size(); }
  String readStringUntil(char term) {
    double t0 = since_start_s();
    while (true) {
      pump_in();
      auto i = in.find(term);
      if (i != std::string::npos) { std::string r = in.substr(0, i); in.erase(0, i + 1); return String(r); }
      if ((since_start_s() - t0) * 1000.0 >= timeout_ms) { std::string r = in; in.clear(); return String(r); }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  }
  void print(const char *s) { emit(s); }
  void print(float v, int d) { char b[32]; snprintf(b, sizeof b, "%.*f", d, v); emit(b); }
  void print(double v, int d) { print((float) v, d); }
  void print(float v) { print(v, 2); }
  void print(int v) { emit(std::to_string(v)); }
  void print(long v) { emit(std::to_string(v)); }
  void print(unsigned long v) { emit(std::to_string(v)); }
  template <class T> void println(T v) { print(v); emit("\r\n"); }
  template <class T> void println(T v, int d) { print(v, d); emit("\r\n"); }
  void println() { emit("\r\n"); }
};
static SerialSim Serial;

// ------------------------------------------------------------------ sketch
#include "../../arduino_pressure_controller/arduino_pressure_controller.ino"

int main(int argc, char **argv)
{
  if (argc > 1) { g_truth = fopen(argv[1], "w"); setvbuf(g_truth, nullptr, _IOLBF, 0); }
  if (argc > 2) g_clock_error_ppm = atof(argv[2]);
  if (argc > 3) g_rng.seed(atoi(argv[3]));
  setvbuf(stdout, nullptr, _IONBF, 0);
  setup();
  while (true) loop();
}
