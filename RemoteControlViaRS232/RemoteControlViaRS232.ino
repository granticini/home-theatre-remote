/*
  Home Theatre Remote Control via RS-232
  Grant Phillips
  23 January 2021
  -------------------------

  Control my Pioneer A/V Amplifier and Panasonic Projector
  using RS-232 connections on both devices.
  
 The circuit:
 * Pionner Amp
 * -----------
 * RX is digital pin 2 (connect to TX of other device)
 * TX is digital pin 3 (connect to RX of other device)
 * Button On is digital pin 6
 * Button Off is digital pin 7
 * Button Apple TV is digital pin 8
 * Button Playstation is digital pin 9
 * Button Radio is digital pin 10
 * Button TV is digital pin 11

 * Panasonic Projector
 * -----------
 * RX is digital pin 4 (connect to TX of other device)
 * TX is digital pin 5 (connect to RX of other device)
 * Button On is digital pin 12
 * Button Off is digital pin 14
 * 
*/
// #include <SoftwareSerial.h>

#define APP_NAME     "Home Theatre Remote"
#define APP_VERSION  "0.0.1"
#define APP_AUTHOR   "Grant Phillips"
#define APP_CREATED  "23 January 2021"
#define APP_BUILT    "31 January 2021"

#define ON               HIGH
#define OFF              LOW
#define BUTTON_DOWN      LOW
#define BUTTON_UP        HIGH

// Modes
#define MODE_None              0
#define MODE_Normal_Start      100
#define MODE_Normal            101
int mode = MODE_None;

unsigned long previousTime;
int previousSeconds = -1;

#define SECONDS_PER_MINUTE          60
#define DELAY_ONE_SECOND          1000
#define DELAY_BUTTON_DEBOUNCE       50
#define DELAY_AFTER_BUTTON_PRESS   200
#define DELAY_mainloop              20
#define DELAY_SETUP_MAIN_SERIAL     10

#define BAUD_MAIN 115200
bool DEBUG_ON = true;

bool ledState = false;

#define AMP_BAUD 9600
#define AMP_DATA_BITS 8
#define AMP_STOP_BITS 1
#define AMP_PARITY_BITS 0

#define AMP_PIN_RX 8
#define AMP_PIN_TX 9
#define AMP_PIN_BUTTON_ON 2
#define AMP_PIN_BUTTON_OFF 3
#define AMP_PIN_BUTTON_APPLE_TV 4
#define AMP_PIN_BUTTON_PLAYSTATION 5 
#define AMP_PIN_BUTTON_RADIO 6
#define AMP_PIN_BUTTON_TV 7
 
#define PROJECTOR_BAUD 9600
#define PROJECTOR_DATA_BITS 8
#define PROJECTOR_STOP_BITS 1
#define PROJECTOR_PARITY_BITS 0

#define PROJECTOR_PIN_RX 10
#define PROJECTOR_PIN_TX 11
#define PROJECTOR_PIN_BUTTON_ON 12
#define PROJECTOR_PIN_BUTTON_OFF 14

byte BYTE_STX = 0x02;
byte BYTE_ETX = 0x03;
byte BYTE_CR  = 0x0D;
byte BYTE_LF  = 0x0A;

// SoftwareSerial projectorSerial(PROJECTOR_PIN_RX, PROJECTOR_PIN_TX);
byte PROJECTOR_COMMAND_POWER_ON[]  = {BYTE_STX, 'P', 'O', 'N', BYTE_ETX};
byte PROJECTOR_COMMAND_POWER_OFF[] = {BYTE_STX, 'P', 'O', 'F', BYTE_ETX};
int PROJECTOR_DELAY_AFTER_POWER_ON = 10000;

// SoftwareSerial ampSerial(AMP_PIN_RX, AMP_PIN_TX);
byte AMP_COMMAND_POWER_ON[]    = {'P', 'O', BYTE_CR};
byte AMP_COMMAND_POWER_OFF[]   = {'P', 'F', BYTE_CR};
byte AMP_COMMAND_VOLUME_UP[]   = {'V', 'U', BYTE_CR};
byte AMP_COMMAND_VOLUME_DOWN[] = {'V', 'D', BYTE_CR};
byte AMP_COMMAND_MUTE_ON[]     = {'M', 'O', BYTE_CR};
byte AMP_COMMAND_MUTE_OFF[]    = {'M', 'F', BYTE_CR};

// following commands have a parameter that is sent first, e.g.
// Example to set Apple TV as the selected input...
// 25FN<CR>
byte AMP_COMMAND_VOLUME_SET_MAXIMUM[]    = {'1', '8', '5', 'V', 'L', BYTE_CR};
byte AMP_COMMAND_VOLUME_SET_0DB[]        = {'1', '6', '1', 'V', 'L', BYTE_CR};
byte AMP_COMMAND_VOLUME_SET_MINUS_30DB[] = {'1', '0', '1', 'V', 'L', BYTE_CR};
byte AMP_COMMAND_VOLUME_SET_MINIMUM[]    = {'0', '0', '1', 'V', 'L', BYTE_CR};
byte AMP_COMMAND_VOLUME_SET_MUTE[]       = {'0', '0', '0', 'V', 'L', BYTE_CR};

byte AMP_COMMAND_FUNCTION_MODE_SET[] = {'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_PLAYSTATION[] = {'0', '4', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_APPLE_TV[]    = {'2', '5', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_PHONO[]       = {'0', '0', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_CD[]          = {'0', '1', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_TUNER[]       = {'0', '2', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_CDR[]         = {'0', '3', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_DVD[]         = {'0', '4', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_TV[]          = {'0', '5', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_VIDEO1[]      = {'1', '0', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_VIDEO2[]      = {'1', '4', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_VIDEO3[]      = {'3', '2', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_DVR1[]        = {'1', '5', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_DVR2[]        = {'1', '6', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_HDMI1[]       = {'1', '9', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_HDMI2[]       = {'2', '0', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_HDMI3[]       = {'2', '1', 'F', 'N', BYTE_CR};
byte AMP_FUNCTION_MODE_BDP[]         = {'2', '5', 'F', 'N', BYTE_CR};

int AMP_DELAY_AFTER_POWER_ON = 100;

#define MAX_PIN 24
int lastButtonStates[MAX_PIN];
int buttonStates[MAX_PIN];
unsigned long lastDebounceTimes[MAX_PIN]; // last time the pins were toggled, used to keep track of time
unsigned long DELAY_DEBOUNCE = 50;   // the debounce time which user sets prior to run

int mainloopCounter = -1;

void projectorSendCommand(byte* command, int len)
{
  // projectorSerial.write(command, len);
}

void ampSendCommand(byte* command, int len)
{
  // ampSerial.write(command, len);
}


void setup()
{
  DEBUG_ON = false;

  // Open serial communications and wait for port to open:
  Serial.begin(BAUD_MAIN);
  while (!Serial)
  {
    // wait for serial port to connect. Needed for Native USB only
    delay(DELAY_SETUP_MAIN_SERIAL);
  }
  DEBUG_ON = true;
  DBG("******");
  DBG("setup()");

  setupLEDs();
  setupButtons();

  setupSerialAmp();
  setupSerialProjector();
      
  previousTime = millis();
  mainloopCounter = 0;

  DBG("Mainloop delay is ", DELAY_mainloop, " milliseconds");
  DBG("Success - startup complete.");
  DBG("===========================");

//  digitalWrite(LED_BUILTIN, HIGH);
//  delay(500);
//  digitalWrite(LED_BUILTIN, LOW);
}

// Called last from the variadic template function
void DBG()
{
  if (!DEBUG_ON) return;
  Serial.println();
}

template <typename T, typename... Types>
void DBG(T first, Types... other)
{
  if (!DEBUG_ON) return;
  Serial.print(first);
  DBG(other...);
}


void setupSerialAmp()
{
  // setup the amp and projector the serial comms
  DBG("Amplifier Serial Config:");
  DBG("Baud ........ ", AMP_BAUD);
  DBG("Data bits ... ", AMP_DATA_BITS);
  DBG("Stop bits ... ", AMP_STOP_BITS);
  DBG("Parity ...... ", AMP_PARITY_BITS);
  
  // ampSerial.begin(AMP_BAUD);
  
  DBG("Serial connection to Amp completed.");
}

void setupSerialProjector()
{
  DBG("Projector Serial Config:");
  DBG("Baud ........ ", PROJECTOR_BAUD);
  DBG("Data bits ... ", PROJECTOR_DATA_BITS);
  DBG("Stop bits ... ", PROJECTOR_STOP_BITS);
  DBG("Parity ...... ", PROJECTOR_PARITY_BITS);
  
  // projectorSerial.begin(PROJECTOR_BAUD);

  DBG();
  DBG("Serial connection to Projector completed.");
}

void setupLEDs()
{
  DBG("Setting up LEDs ...");
  pinMode(LED_BUILTIN, OUTPUT);
  
//  digitalWrite(LED_BUILTIN, HIGH);
//  delay(100);
//  digitalWrite(LED_BUILTIN, LOW);

  DBG("LED setup completed ok");
}

void setupButtons()
{
  DBG("Setting up Buttons ...");

  // initialise the array that tracks the Button states
  unsigned long currentTime = millis();
  int pin;
  for (pin = 0; pin < MAX_PIN; pin++)
  {
    lastButtonStates[pin]  = BUTTON_UP;
    buttonStates[pin]      = BUTTON_UP;
    lastDebounceTimes[pin] = currentTime;
  }

  pinMode(AMP_PIN_BUTTON_ON,          INPUT_PULLUP);
  pinMode(AMP_PIN_BUTTON_OFF,         INPUT_PULLUP);
  pinMode(AMP_PIN_BUTTON_APPLE_TV,    INPUT_PULLUP);
  pinMode(AMP_PIN_BUTTON_PLAYSTATION, INPUT_PULLUP);
  pinMode(AMP_PIN_BUTTON_RADIO,       INPUT_PULLUP);
  pinMode(AMP_PIN_BUTTON_TV,          INPUT_PULLUP);

  pinMode(PROJECTOR_PIN_BUTTON_ON,    INPUT_PULLUP);
  pinMode(PROJECTOR_PIN_BUTTON_OFF,   INPUT_PULLUP);
  
  DBG("Buttons setup ok");
}

bool readButtonState(int buttonPin)
{
  bool pressed = false;
  
  // read the button pin, if pressed will be high, if not pressed will be low
  int reading = digitalRead(buttonPin);
  
  // in the case that the reading is not equal to the last state set the last debounce time to current millis time
  if (reading != lastButtonStates[buttonPin])  
  {
    // button state has changed
    lastDebounceTimes[buttonPin] = millis();
  }

  unsigned long currentTime = millis();
  if (currentTime < lastDebounceTimes[buttonPin])
  {
    // time must have overflowed and gone back to 0
    DBG("current time has wrapped overflowed!");
    lastDebounceTimes[buttonPin] = 0;
  }
  
  // check the difference between current time and last registered button press time, 
  // if it's greater than user defined delay then change the LED state as it's not a bounce
  if ((currentTime - lastDebounceTimes[buttonPin]) > DELAY_BUTTON_DEBOUNCE) 
  {
    if (reading != buttonStates[buttonPin])
    {
      buttonStates[buttonPin] = reading;
      if (buttonStates[buttonPin] == LOW) 
      {
        // button has been pressed
        DBG("Pressed Button on pin # ", buttonPin);
        pressed = true;
//        DBG("Snoozing for ", DELAY_AFTER_BUTTON_PRESS, " milliseconds");
        delay(DELAY_AFTER_BUTTON_PRESS);
//        DBG();
      }
    }
  }

  // save the reading.
  // next time through the loop the state of the reading will be known as the lastButtonState
  lastButtonStates[buttonPin] = reading;

  return pressed;
}

void loop() // run over and over
{
  unsigned long currentTime = millis();
  
  if ((currentTime < previousTime) or 
      (currentTime - previousTime >= DELAY_mainloop))
  {
    int currentSeconds = (currentTime / DELAY_ONE_SECOND) % SECONDS_PER_MINUTE;
    if (currentSeconds != previousSeconds)
    {
      // sleepytime has been reached
      ledState = !ledState;
      previousTime = currentTime;
      previousSeconds = currentSeconds;
    }
  }
  
  mainloopCounter += 1;
  if (DEBUG_ON && mainloopCounter > 10000)
  {
    DBG("Turning off DEBUG mode!");
    DEBUG_ON = false;
  }

  // detect button press
  if (readButtonState(AMP_PIN_BUTTON_ON))
  {
    DBG("Button Pressed: Amp ON");
    ampSendCommand(AMP_COMMAND_POWER_ON, sizeof(AMP_COMMAND_POWER_ON));
    delay(AMP_DELAY_AFTER_POWER_ON);
    ampSendCommand(AMP_COMMAND_POWER_ON, sizeof(AMP_COMMAND_POWER_ON));
  }
  else if (readButtonState(AMP_PIN_BUTTON_OFF))
  {
    DBG("Button Pressed: Amp OFF");
    ampSendCommand(AMP_COMMAND_POWER_OFF, sizeof(AMP_COMMAND_POWER_OFF));
  }

// probably don't need this
// if (ampSerial.available())
// {
//   receiveBuffer = ampSerial.read();
// }


// if (projectorSerial.available())
// {
//   receiveBuffer = projectorSerial.read();
// }

  //DBG("... ", mainloopCounter);
  delay(DELAY_mainloop);
}
