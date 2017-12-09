#include "pitches.h"
#include <stdint.h>


// [Config] Basic settings
#define BUTTON 5        // Button count
#define DUR_AFTER 3000  // Duration after buzzing before a new button press permitted
#define ERR 40          // Error in analog input value
#define DEBUG 0         // wether to enable playMelody or not

// The components active high
const uint8_t pin_button = A0;  // Using just one pin method arranged in voltage divider
const uint8_t pin_output[BUTTON] = {3, 5, 3, 3, 3};

uint8_t button[BUTTON] = {0};
uint8_t prev_button[BUTTON] = {0};
const int button_th[BUTTON] = {1000, 830, 614, 393, 279};  // [Config] Threshold for A0 to be identified as certain button, tuning necessary
                                 // 10k, 22k, 39k, 82k, 150k

//Notes in the melody: (note durations: 4 = quarter note, 8 = eighth note, etc.)
typedef struct note {
  int key; float dur;
};

note melody[] = {
  //{NOTE_C4, 4}, {NOTE_G3, 8}, {NOTE_G3, 8}, {NOTE_A3, 4}, {NOTE_G3, 4}, {0, 4}, {NOTE_B3, 4}, {NOTE_C4, 4}
  //{NOTE_C4, 4}, {NOTE_D4, 4}, {NOTE_E4, 4}, {NOTE_F4, 4}, {NOTE_G4, 4}, {NOTE_A4, 4}, {NOTE_B4, 4}, {NOTE_C5, 4}, {NOTE_D5, 4}, {NOTE_E5, 4}, {NOTE_F5, 4}
  //{NOTE_C5, 12}, {0, 12}, {NOTE_D5, 12}, {0, 12}, {NOTE_E5, 0.5}
  {NOTE_E5, 1}, {0, 12}, {NOTE_E5, 12}, {0, 12}, {NOTE_E5, 12}
  //{NOTE_E5, 0.5}
  //{NOTE_C4, 1}
};

int duration;

void playMelody(int repeat, int pin) {
  //Iterate over the notes of the melody:
  for (; repeat >= 0; repeat--) {
    for (int i = 0; i < sizeof(melody)/sizeof(note); i++) {
      //To calculate the note    n       , take one second divided by the note type.
      //e.g. quarter note = 1000/4, eighth note = 1000/8, etc.
      duration = 1000 / melody[i].dur;
      tone(pin, melody[i].key, duration);
  
      //To distinguish the notes, set a minn n imum time between them.
      //the note's duration + 30% seems to work well:
      //int pause = duration * 1.30;
      int pause = duration * 1.10;
      delay(pause);
      
      //Stop the tone playing:
      noTone(pin);
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Pin mode
  for (int i = 0; i < BUTTON; i++){
    pinMode(pin_output[i], OUTPUT);
  }
}

void loop() {
  int input = analogRead(pin_button);
  Serial.print(input); Serial.print(' ');

  for (int i = 0; i < BUTTON; i++){
    // Map analog input to button
    if ((input > button_th[i] - ERR) && (input <= button_th[i] + ERR)) button[i] = 1;
    else button[i] = 0;

    Serial.print(button[i]); Serial.print(' ');

    // If triggered
    if (!DEBUG && button[i] && prev_button[i]) {
      playMelody(0, pin_output[i]);
      digitalWrite(pin_output[i], HIGH);  // Stays on until a period of time
      delay(DUR_AFTER);
      digitalWrite(pin_output[i], LOW);  // Stays on until a period of time
    }

    // To prevent noise. Just one look-back already removed almost all false-alarms
    prev_button[i] = button[i];
  }
  Serial.println();
}
