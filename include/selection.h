#pragma once

#include <string>

//selector configuration
#define HUE 180
#define DEFAULT 1
#define AUTONS "_Nothing", "_Half", "_Roller"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
