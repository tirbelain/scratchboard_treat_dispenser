#ifndef __LOGGING_H__
#define __LOGGING_H__

#ifdef DEBUG_OUTPUT

#define debugLog(x) Serial.print(x)
#define debugLogln(x) Serial.println(x)
#define debugLogF(x) Serial.print(F(x))
#define debugLoglnF(x) Serial.println(F(x))

#else // DEBUG_OUTPUT

#define debugLog(x)
#define debugLogln(x)
#define debugLogF(x)
#define debugLoglnF(x)

#endif // DEBUG_OUTPUT

#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#define logF(x) Serial.print(F(x))
#define loglnF(x) Serial.println(F(x))

#endif // __LOGGING_H__