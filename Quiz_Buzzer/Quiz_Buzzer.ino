#include "pitches.h"
#include <stdint.h>


// [Config] Basic settings
#define BUTTON 5        // Button count
#define DUR_AFTER 3000  // Duration after buzzing before a new button press permitted
#define TH_BUTTON 256   // analogRead threshold for
#define DEBUG 0         // wether to enable playMelody or not

// The components are all active LOW
const uint8_t pin_button[BUTTON] = {A0, A1, A0, A0, A0};
const uint8_t pin_output[BUTTON] = {3, 5, 3, 3, 3};

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

  for (int i = 0; i < BUTTON; i++){
    pinMode(pin_button[i], INPUT_PULLUP);
    pinMode(pin_output[i], OUTPUT);
  }
}

void loop() {
  for (int i = 0; i < BUTTON; i++){
    Serial.print(analogRead(pin_button[i]));
    Serial.print(' ');
    Serial.print(digitalRead(pin_button[i])*1000);
    Serial.print(' ');
    if (!DEBUG && (analogRead(pin_button[i]) < TH_BUTTON)) {
      playMelody(0, pin_output[i]);
      digitalWrite(pin_output[i], HIGH);  // Stays on until a period of time
      delay(DUR_AFTER);
      digitalWrite(pin_output[i], LOW);  // Stays on until a period of time
    }
  }
  Serial.println();
}
