/*
Arduino nano every
*/

unsigned long currentMillis;
unsigned long msg_prev_millis = 0;
unsigned long alarm_prev_millis = 0;
unsigned long built_prev_millis = 0;

unsigned long alarm_buzzer1_prev_millis = 0;
unsigned long alarm_buzzer2_prev_millis = 0;

unsigned long msg_duration = 10000;
unsigned long alarm_duration = 10000;

unsigned long alarm_buzzer1_duration = 500;
unsigned long alarm_buzzer2_duration = 100;

unsigned long msg_start = 0;
unsigned long alarm_start = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

bool alarm_started = false;
bool msg_started = false;

// Led messaggio: 3
// Led luci: 4
// Led allarme: 2
int msg_led_pin = 3;
int lights_led_pin = 4;
int alarm_led_pin = 2;

int alarm_led_blink = 240;
int msg_led_blink = 1000;
int built_led_blink = 5000;

// Buttons
// lights
int alarm_btn_pin = 11;
int lights_btn_pin = 10;
int msg_btn_pin = 12;


int lights_led_state = LOW;         // the current state of the output pin
int lights_button_state = LOW;             // the current reading from the input pin
int lights_button_last_state = 1; // 1 is OFF

// Alarm
int alarm_led_state = LOW;         // the current state of the output pin
int alarm_button_state = LOW;             // the current reading from the input pin
int alarm_button_last_state = 1; // 1 is OFF

// Msg
int msg_led_state = LOW;         // the current state of the output pin
int msg_button_state = LOW;             // the current reading from the input pin
int msg_button_last_state = 1; // 1 is OFF

// Builtin led
int built_led_state = LOW;

// Buzzers
int buzzer1_pin = 6;
int buzzer2_pin = 7;


void setup() {
  Serial.begin( 9600 );

  // Leds
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (msg_led_pin, OUTPUT);
  pinMode (lights_led_pin, OUTPUT);
  pinMode (alarm_led_pin, OUTPUT);

  // Buttons
  pinMode(alarm_btn_pin, INPUT_PULLUP );
  pinMode(msg_btn_pin, INPUT_PULLUP );
  pinMode(lights_btn_pin, INPUT_PULLUP );

  // Buzzers
  //pinMode(buzzer1_pin, OUTPUT );
  //pinMode(buzzer2_pin, OUTPUT );
}

// the loop function runs over and over again forever
void loop() {

  currentMillis = millis();

  built_led();
  alarm(); 
  msg();
  lights();
}

/*
* Builtin led routine
*/
void built_led() {
  if (currentMillis - built_prev_millis >= built_led_blink) {
    built_prev_millis = currentMillis;

    built_led_state = !built_led_state;
    digitalWrite(LED_BUILTIN, built_led_state);
  }
}

/*
* Alarm routine
*/
void alarm() {
  int reading = digitalRead(alarm_btn_pin);

   if (reading != alarm_button_last_state) {
    Serial.println("alarm btn pressed");
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    if (reading != alarm_button_state) {
      alarm_button_state = reading;
      // only toggle the LED if the new button state is LOW - pressed
      if (alarm_button_state == LOW) {
        alarm_started = !alarm_started;
        alarm_start = millis();
      }
    }
  }

  if (alarm_started == true) {
    if (currentMillis - alarm_start <= alarm_duration) {

      if ((currentMillis - alarm_prev_millis) >= alarm_led_blink) {
        alarm_prev_millis = currentMillis;

        alarm_led_state = !alarm_led_state;
        digitalWrite(alarm_led_pin, alarm_led_state);
      }

      // Buzzer
      /*
      if (currentMillis - alarm_buzzer1_prev_millis <= alarm_buzzer1_duration) {
        tone(buzzer1_pin, 1000);
      } else {
        alarm_buzzer2_prev_millis = currentMillis;
      }
      
      
      if (currentMillis - alarm_buzzer2_prev_millis <= alarm_buzzer2_duration) {
        tone(buzzer2_pin, 500);
      } else {
        alarm_buzzer1_prev_millis = currentMillis;
      }
      */
      
    } else {
        alarm_started = false;
    }
  } else {
    digitalWrite(alarm_led_pin, LOW);
  }

  alarm_button_last_state = reading;
}

/*
* Msg routine
*/
void msg() {

  int reading = digitalRead(msg_btn_pin);

   if (reading != msg_button_last_state) {
    Serial.println("msg btn pressed");
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    if (reading != msg_button_state) {
      msg_button_state = reading;
      // only toggle the LED if the new button state is LOW - pressed
      if (msg_button_state == LOW) {
        msg_started = !msg_started;
        msg_start = millis();
      }
    }
  }

  if (msg_started == true) {
    if (currentMillis - msg_start <= msg_duration) {

      if ((currentMillis - msg_prev_millis) >= msg_led_blink) {
        msg_prev_millis = currentMillis;

        msg_led_state = !msg_led_state;
        digitalWrite(msg_led_pin, msg_led_state);
      }
    } else {
        msg_started = false;
    }
  } else {
    digitalWrite(msg_led_pin, LOW);
  }

  msg_button_last_state = reading;
}

/*
* Lights
*/
void lights() {

  int reading = digitalRead(lights_btn_pin);

  // If the switch changed, due to noise or pressing:
  if (reading != lights_button_last_state) {
    Serial.println("lights btn pressed");
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    if (reading != lights_button_state) {
      lights_button_state = reading;

      // only toggle the LED if the new button state is LOW - pressed
      if (lights_button_state == LOW) {
        lights_led_state = !lights_led_state;
      }
    }
  }

  // set the LED:
  digitalWrite(lights_led_pin, lights_led_state);
  lights_button_last_state = reading;
}