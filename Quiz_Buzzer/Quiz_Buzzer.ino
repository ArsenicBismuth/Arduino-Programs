#include "pitches.h"

//Notes in the melody: (note durations: 4 = quarter note, 8 = eighth note, etc.)
typedef struct note {
  int key; int dur;
} melody[] = {
  //{NOTE_C4, 4}, {NOTE_G3, 8}, {NOTE_G3, 8}, {NOTE_A3, 4}, {NOTE_G3, 4}, {0, 4}, {NOTE_B3, 4}, {NOTE_C4, 4}
  //{NOTE_C4, 4}, {NOTE_D4, 4}, {NOTE_E4, 4}, {NOTE_F4, 4}, {NOTE_G4, 4}, {NOTE_A4, 4}, {NOTE_B4, 4}, {NOTE_C5, 4}, {NOTE_D5, 4}, {NOTE_E5, 4}, {NOTE_F5, 4}
  {NOTE_E5, 1}
};

int pin_buzzer = 3;

void playmelody(int repeat) {
  //Iterate over the notes of the melody:
  for (; repeat >= 0; repeat--) {
    for (int i = 0; i < sizeof(melody) - 1; i++) {
      //To calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000/4, eighth note = 1000/8, etc.
      duration = 1000 / melody[i].dur;
      tone(pin_buzzer, melody[i].key, duration);
  
      //To distinguish the notes, set a minimum time between them.
      //the note's duration + 30% seems to work well:
      int pause = duration * 1.30;
      delay(pause);
      
      //Stop the tone playing:
      noTone(pin_buzzer);
    }
  }
}

void setup() {
  
}

void loop() {
  
}
