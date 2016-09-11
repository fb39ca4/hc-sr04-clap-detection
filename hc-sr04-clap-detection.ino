/*
MIT License

Copyright (c) 2016 fb39ca4

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Servo.h>

/*
This Arduino sketch is to be used with an HC-SR04 ultrasonic distance sensor to 
detect hand claps and other percussive sounds.

The sensor is operated by sending a pulse on US_TRIG_PIN. At this point, the 
sensor sends an ultrasonic pulse and sets US_ECHO_PIN to high. Upon detecting an
echo, the sensor will set US_ECHO_PIN to low, and the time elapsed between 
these two events is used to compute distance. However, if the echo otherwise 
does not arrive, the sensor times out after 130-150 ms, varying between 
individual sensors.

The functioning of this sensor can be exploited to detect percussive sounds, by 
blocking the transmitting side. Now, no echos will arrive, so the pulse duration
from US_ECHO_PIN will always be at the timeout length. However, if the pulse is 
shorter, below a treshold enough to differentiate from the timeout length, it 
can be assumed to be from an external sound. This method will fail if a sound is
detected right as the sensor times out, but will still detect most sounds.

Also included is code to match the timings of recieved claps to a pattern. It 
will match a correct pattern even if there are unrelated noises in between, so
it would not be the best for environments with sustained noise, because any
stream of closely spaced detections that is long enough would match any pattern.
*/

//Input configuration
#define US_GND_PIN 12
#define US_ECHO_PIN 11
#define US_TRIG_PIN 10
#define US_VCC_PIN 9
#define LED_PIN 4
#define SERVO_PIN 3
#define BROWNOUT_DETECT_PIN 0
#define SERVO_MIN_ANGLE 10
#define SERVO_MAX_ANGLE 150
#define ULTRASONIC_TIMEOUT 133000 //Set this to 1000-2000ms below the observed timeout value.
#define BROWNOUT 720 //Treshold for 5V brownout detection

template <typename T, unsigned int buffer_size>
class circular_buffer {
  public:
    unsigned int head = 0;
    unsigned int tail = 0;
    T data[buffer_size] = {0};
    void clear(T value) {
      for (unsigned int i = 0; i < buffer_size; i++) data[i] = value;
    }
    void insert(T element) {
      data[head] = element;
      head++;
      if (head >= buffer_size) head = 0;
    }
    T &operator[](unsigned int index) {
      return data[(head - index - 1) & (buffer_size - 1)];
    }
};
//Stores the history of recieved claps, as the time since the next most recent one
circular_buffer<unsigned long, 128> clap;

//Configures the clap sequence.
const unsigned int clap_sequence_timings[] = {1000, 1000};
const unsigned int clap_sequence_size = 3;
const unsigned int max_clap_deviation = 200;
//Currently set to detect a sequence of three claps, each 1000+-200 milliseconds apart.

//Recursively searches through the clap buffer for a pattern that matches.
bool detect_clap_sequence(unsigned int sequence_position, unsigned int buffer_position, unsigned int can_skip) {
  unsigned int current_delay = 0;
  while (true) {
    if (buffer_position > 15) return false;
    current_delay += clap[buffer_position];
    if (current_delay < clap_sequence_timings[sequence_position] - max_clap_deviation) {
      buffer_position++;
      continue;
    }
    else if (current_delay > clap_sequence_timings[sequence_position] + max_clap_deviation) break;
    else {
      if (sequence_position == 0) {
        clap.clear(0);
        return true;
      }
      else return detect_clap_sequence(sequence_position - 1, buffer_position + 1, can_skip);
    }
  }
  return false;
}

Servo servo;

//This code runs after each sound is detected.
void on_sound_detected() {
  //Add to timing history, log to Serial.
  static unsigned long last_clap_time = millis();
  unsigned long now = millis();
  clap.insert(now - last_clap_time);
  last_clap_time = now;
  for (unsigned int i = 0; i < 4; i++) {
    Serial.print(clap[i]);
    Serial.write(' ');
  }
  //Compare against sequence
  bool detected_sequence = detect_clap_sequence(clap_sequence_size - 2,0,0);
  Serial.println(detected_sequence);
  //Take action if the sequence matches
  if (detected_sequence) on_sequence_detected();
  //Blink LED
  digitalWrite(LED_PIN, HIGH);
  delay(60);
  digitalWrite(LED_PIN, LOW);
}

void on_sequence_detected() {
  static bool claw_state = 0;
  claw_state = !claw_state;
  servo.write(claw_state ? SERVO_MIN_ANGLE : SERVO_MAX_ANGLE);
}

void setup() {
  pinMode(US_GND_PIN, OUTPUT);
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(US_VCC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  digitalWrite(US_GND_PIN, LOW);
  digitalWrite(US_VCC_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  servo.write(0);
  Serial.begin(115200);
  on_sequence_detected();
}

inline void trigger() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
}

void loop() {
  trigger();
  unsigned long pulse_duration = pulseIn(US_ECHO_PIN, HIGH);
  Serial.println(pulse_duration);
  Serial.println(analogRead(ULTRASONIC_DETECT_PIN));
  if (pulse_duration < ULTRASONIC_TIMEOUT && analogRead(BROWNOUT_DETECT_PIN) < BROWNOUT) { 
    on_sound_detected();
  }
}
