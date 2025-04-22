#include <OneButton.h>

OneButton downButton(27, true, true);
OneButton upButton(14, true, true);
OneButton selectButton(12, true, true);

void setup()
{
    Serial.begin(9600);

    downButton.attachClick([]()
                           { Serial.println("Down Pressed!"); });

    upButton.attachClick([]()
                         { Serial.println("Up Pressed!"); });

    selectButton.attachClick([]()
                             { Serial.println("Select Pressed!"); });
}

void loop()
{
    downButton.tick();
    upButton.tick();
    selectButton.tick();
}
