#include <Arduino.h>
#include <WiFi.h>
#include "AsyncUDP.h"
#include "M5Atom.h"

/*
 * The VISCA binary protocol is exactly the same over UDP as it is over wired connections.
 * However there are exceptions to this rule. When using VISCA over IP, Sony cameras employ
 * a header and footer surrounding each datagram. However, over a wired connection, they
 * do not use such a header and footer, and only send the VISCA packet directly.
 * 
 * This code works more like the PTZ Optics implementation. 
 * 
 * This code connects to WiFi and starts a UDP server.
 * This code also connects to a VISCA camera over RS-232.
 * On bootup, it will immediately send to the camera an ADDR_SET and POWER_ON command.
 * 
 * From that point on, the code will simply proxy all data packets between RS-232 and UDP
 * with very little error checking or validation. Whatever packet is received over UDP
 * will be sent to the camera without modification, and whatever packet is received from
 * the camera over RS-232 will be sent to the most recent UDP client without modification.
 * 
 * WITH A FEW EXCEPTIONS:
 * Cameras and controllers vary in their implementation of speed, and therefore this code
 * allows the programmer to customize various speed curves for the PTZ operations.
 * 
 * They are commented below.
 * 
 * USAGE:
 * Change the settings below, compile and upload!
 * 
 * 
 * FINALLY:
 * This code should work with minor modifications on any device that supports WiFi, RS-232,
 * but it was designed for the M5Stack Atom series devices with the RS-232 attachment.
 * 
 * As an added bonus, the Atom and Atom Matrix devices have RGB LEDs which allow us to
 * use them as tally lights. This code employs that using the vMix tally protocol. If it
 * successfully connects to vMix, it will by default make itself tally whatever input is
 * live when it booted. You can change the tally by pressing the main button on the Atom
 * or the Matrix. If you hold down the button, it will automatically set itself to whatever
 * input is PREVIEW.
 * 
 * NOTE: An RS-232 or RS-422 interface is definitely needed. The biggest concern is that
 * Android devices by default use TTL for serial communications, and that protocol uses
 * voltages that are incompatible with the RS standards.
*/

// uncomment this line if you are using an M5Stack Matrix
// comment it if you are using an M5Stack Atom
#define MATRIX


// WIFI SETTINGS
const char *SSID = "YOUR_SSID";
const char *PASSWORD = "your password";

// UDP SETTINGS
const int udp_port = 52381;

// RS232 Serial Settings
const int txpin = 19;
const int rxpin = 22;

// set the baudrate to match that of the VISCA camera
const int baudrate = 38400;

// Use the following constants and functions to modify the speed of PTZ commands
double ZOOMMULT = 0.3;      // speed multiplier for the zoom functions
double ZOOMEXP = 1.5;       // exponential curve for the speed modification
double PTZMULT = 0.3;       // speed multiplier for the pan and tilt functions
double PTZEXP = 1.0;        // exponential curve for the speed modification

// TALLY SETTINGS
const IPAddress VMIX_IP = IPAddress(192, 168, 50, 12);
const int VMIX_PORT = 8099;



// STATE VARIABLES
bool wifi_connected = false;
bool ignore_button = false;

IPAddress ip;
WiFiClient tcp;
AsyncUDP udp;
int lastclientport = 0;
IPAddress lastclientip;
bool pwr_is_on = false;

// memory buffers for VISCA commands
size_t lastudp_len = 0;
uint8_t lastudp_in[16];
size_t lastser_len = 0;
uint8_t lastser_in[16];

// quick use VISCA commands
const uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
const uint8_t pwr_off[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xff};
const uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};            // address set
const uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff};      // if clear

// quick color constants
// yes, on the Atom and Matrix devices
// the red and green LEDs seem to be swapped
const CRGB blue = CRGB(0x00, 0x00, 0xf0);
const CRGB red = CRGB(0x00, 0xf0, 0x00);
const CRGB green = CRGB(0xf0, 0x00, 0x00);
const CRGB yellow = CRGB(0xf0, 0xf0, 0x00);
const CRGB black = CRGB(0x00, 0x00, 0x00);
CRGB ledcolor = yellow;

// tally variables
int total_inputs = 2;          // vMix always loads with at least two inputs defined
int tally_input = 1;           // for mental consistency, we use input numbers based on 1 like vMix does
int tally_status = 0;          // 0: safe, 1: live, 2: preview
const int tally_bufsize = 100; // vMix can have a lot of inputs
uint8_t tb[tally_bufsize + 1]; // so there is always a null byte at the end... for printing


// general helper function definitions
double zoomcurve(int v)
{
  return ZOOMMULT * pow(v, ZOOMEXP);
}

double ptzcurve(int v)
{
  return PTZMULT * pow(v, PTZEXP);
}

void debug(char c)
{
  Serial.print(c);
}

void debug(int n, int base)
{
  Serial.print(n, base);
  Serial.print(' ');
}

void debug(uint8_t *buf, int len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    uint8_t elem = buf[i];
    debug(elem, HEX);
  }
}

// turn on the whole screen if we have a MATRIX
// otherwise, just light the single LED if we are an Atom
// and remember the color just set
void led(CRGB c)
{
#ifdef MATRIX
  for (int i = 0; i < 25; i++)
  {
    M5.dis.drawpix(i, c);
    // Serial.print(i);
  }
  M5.dis.setBrightness(8);
#else
  M5.dis.drawpix(0, c);
#endif
  ledcolor = c;
}


void send_bytes(uint8_t *b, int len)
{
  for (int i = 0; i < len; i++)
  {
    uint8_t elem = b[i];
    debug(elem);
    Serial1.write(elem);
  }
}

void send_visca(uint8_t *c, size_t len)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    Serial1.write(elem);
  } while (i < len && elem != 0xff);
  Serial.println("");
}

// this function assumes you are sending
// a valid visca command that endsend with a 0xff
// This function will just keep sending bytes from
// memory until it sees a 0xff, so be careful!
void send_visca(const uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    Serial1.write(elem);
  } while (elem != 0xff);
  Serial.println("");
}

void visca_power(bool turnon)
{
  if (turnon)
  {
    send_visca(addr_set);
    delay(500);
    send_visca(pwr_on);
    delay(2000);
    send_visca(if_clear);
  }
  else
  {
    send_visca(if_clear);
    delay(2000);
    send_visca(pwr_off);
  }
  pwr_is_on = turnon;
}

// for debugging purposes only
// prints the current status to the debug serial port
void status()
{
  if (wifi_connected)
  {
    Serial.println("-- UDP LISTENING --");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(udp_port);
    Serial.print("tx: G");
    Serial.print(txpin);
    Serial.print(" | rx: G");
    Serial.println(rxpin);
  }
  else
  {
    Serial.println("CONNECTING ...");
    Serial.println(SSID);
    Serial.println(PASSWORD);
  }
}



// FUNCTIONS TO RECEIVE VISCA OVER UDP =============

// this processes visca commands received over UDP
// and decides if they need to be modified before
// passing through to the camera.
void handle_visca(uint8_t *buf, size_t len)
{
  uint8_t modified[16];
  size_t lastelement = 0;
  for (int i = 0; (i < len && i < 16); i++)
  {
    modified[i] = buf[i];
    lastelement = i;
  }

  // is this a PTZ?
  if (modified[1] == 0x01 && modified[2] == 0x06 && modified[3] == 0x01)
  {
    Serial.println("PTZ CONTROL DETECTED... ADJUSTING SPEED");
    modified[4] = (int)ptzcurve(modified[4]);
    modified[5] = (int)ptzcurve(modified[5]);
  }
  if (modified[1] == 0x01 && modified[2] == 0x04 && modified[3] == 0x07)
  {
    Serial.println("ZOOM CONTROL DETECTED, ADJUSTING SPEED");
    int zoomspeed = modified[4] & 0b00001111;
    zoomspeed = (int)zoomcurve(zoomspeed);
    int zoomval = (modified[4] & 0b11110000) + zoomspeed;
    modified[4] = zoomval;
  }

  Serial1.write(modified, lastelement + 1);
}

void start_server()
{
  Serial.print("Starting UDP server on port: ");
  Serial.println(udp_port);
  udp.close(); // will close only if needed
  if (udp.listen(udp_port))
  {
    Serial.println("Server is Running!");
    udp.onPacket([](AsyncUDPPacket packet) {
      CRGB oldc = ledcolor;
      led(yellow);

      // debug(packet);
      lastclientip = packet.remoteIP();
      lastclientport = packet.remotePort();

      Serial.print("Type of UDP datagram: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                             : "Unicast");
      Serial.print(", Sender: ");
      Serial.print(lastclientip);
      Serial.print(":");
      Serial.print(lastclientport);
      Serial.print(", Receiver: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Message length: ");
      Serial.print(packet.length());
      Serial.print(", Payload (hex):");
      debug(packet.data(), packet.length());
      Serial.println();

      handle_visca(packet.data(), packet.length());
      led(oldc);
    });
  }
  else
  {
    Serial.println("Server failed to start");
  }
}


// FUNCTIONS TO RECEIVE TALLY DATA OVER TCP
void start_tally()
{
  Serial.println("Starting Tally Listener");
  tcp.connect(VMIX_IP, VMIX_PORT);
  tcp.write("SUBSCRIBE TALLY\r\n");
}
int read_tally(int input) { return (tb[8 + input]) - 48; }
int read_tally() { return read_tally(tally_input); }

void update_tally()
{
  tally_status = read_tally(); // ASCII NUMBERS 0-9 ARE ENCODED 48-57 (0x30-0x39)

  switch (tally_status)
  {
  case 0:
    led(black);
    break;
  case 1:
    led(red);
    break;
  case 2:
    led(green);
    break;
  }
}

// the TCP socket is read synchronously because I'm not that
// good at Arduino programming yet.
void check_tally()
{
  int avail = tcp.available();
  if (avail == 0)
    return;

  Serial.println("TCP data in buffer!");
  // TALLY DATA ALWAYS LOOKS LIKE THIS:
  // TALLY OK 120001\r\n
  // (encoded as ASCII)
  // input x will be at buffer index 8 + x (because inputs are 1-based)
  // total inputs will be avail - 9
  int inputs = avail - 11; // the data will be terminated with \r\n
  if (inputs > 1 && total_inputs != inputs)
  {
    Serial.print("INPUT COUNT HAS CHANGED TO: ");
    Serial.println(inputs);
    total_inputs = inputs;
  }

  // vMix terminates messages with \r\n
  // readBytesUntil will discard the terminator char
  // but we also want to discard the \r before it
  size_t read = tcp.readBytesUntil('\n', tb, tally_bufsize); // never read more than the tally_bufsize
  if (read == 0)
    return;

  debug(tb, read);
  Serial.println("");
  tb[read - 1] = 0;           // convert the \r to 0 or at least make sure the final char has a null
  Serial.println((char *)tb); // since there's a null at the end, we can print the buffer like a string

  update_tally();
}



// FUNCTIONS TO HANDLE WiFi EVENTS

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  led(green);
  ip = info.got_ip.ip_info.ip.addr;
  Serial.println("WiFi Connected!");
  Serial.println(ip);
  wifi_connected = true;
  start_server(); // will stop any previous servers
  start_tally();
}

void WiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ip = IPAddress(0, 0, 0, 0);
  wifi_connected = false;
}


// FUNCTION TO READ DATA SENT FROM THE CAMERA
// the RS-232 port is read synchronously because I'm not that
// good at Arduino programming yet.
// check if we have received data over the serial port
void check_serial()
{
  int available = Serial1.available();
  while (available > 0)
  {
    led(yellow);
    Serial.println("Data available on Serial1");
    int actual = Serial1.readBytesUntil(0xff, lastser_in, available); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);
    Serial.println("");
    available = Serial1.available();
    update_tally();
  }
}



// put your setup code here, to run once:
void setup()
{
  M5.begin(true, true, true);
  Serial.begin(baudrate);

  led(yellow);

  Serial.println("started...");
  Serial.println("connecting to WiFi...");

  // connect to WiFi
  WiFi.setAutoConnect(true);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiDisconnected, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);
  WiFi.begin(SSID, PASSWORD, 0, NULL, true);

  // start the visca serial connection
  Serial.println("connecting to camera...");
  Serial1.begin(baudrate, SERIAL_8N1, rxpin, txpin, false, 200);

  visca_power(true);
}

void loop()
{
  // HANDLE VARIOUS BUTTON EVENTS
  if (M5.Btn.wasPressed())
  {
    tally_input = (tally_input % total_inputs) + 1; // mod before add yields 1-indexed values
    Serial.println("button!");
    Serial.print("WATCHING INPUT: ");
    Serial.println(tally_input);
    update_tally();
  }
  else if (M5.Btn.pressedFor(750) && !ignore_button)
  {
    ignore_button = true;
    Serial.println("long press button!");
    // find the next active tally by cycling through all the inputs to see
    // if any of them are preview
    for (int i = 0; i < total_inputs; i++)
    {
      tally_input = (tally_input % total_inputs) + 1; // mod before add yields 1-indexed values
      if (read_tally() == 2)
        break;
    }
    Serial.print("WATCHING INPUT: ");
    Serial.println(tally_input);
    update_tally();
    start_tally();
  }
  else if (M5.Btn.wasReleased())
  {
    ignore_button = false;
  }

  // blink if no wifi connection
  if (!wifi_connected)
  {
    led(black);
    delay(100);
    led(yellow);
    delay(100);
    return;
  }

  check_serial();
  check_tally();

  // don't overload the CPU! take a breather
  delay(100);
  Serial.print('.');
  M5.update(); // reads the button again among other things.
}
