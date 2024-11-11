#include <Arduino.h>
#include <TitanBot.h>
#include <KB_music.h>




KB_music music;

typedef int Number;
typedef int Boolean;

${EXTINC}

${VARIABLE}

${FUNCTION}

void setup()
{

  TitanBot_setup();
  buzzer(1000,100);
  setSensorPins((const int[]) {0, 1, 2 , 3,4,5,6,7}, 8);
  setSensorPins_B((const int[]) {0, 1, 2 , 3,4,5,6,7}, 8);
  music.begin();
 

  ${SETUP_CODE}
  ${BLOCKSETUP}
}
// void setup1()
// {
//   // TitanBot_setup();
//   // setSensorPins((const int[]) {0, 1, 2 , 3,4,5,6,7}, 8);
//   // setSensorPins_B((const int[]) {0, 1, 2 , 3,4,5,6,7}, 8);
//   // music.begin();


// }
void loop()
{
  ${LOOP_CODE}
  ${LOOP_EXT_CODE}
}
// void loop1(){

// }