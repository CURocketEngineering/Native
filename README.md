# Native

This is a native PlatformIO environment meaning that it runs on a laptop/desktop computer rather than our embedded hardware. 
To support a native environment, a 'hal' or hardware abstraction layer was added to **Avionics**. When you want to 
do `#include "Arduino.h"` instead, you will `#include "ArduinoHAL.h"` which will replace the normal Arduino methods with
mocks when the software isn't running on an Arduino.

You should no longer clone **Avionics** directly. Instead, clone this repo and then use the submodule to work on **Avionics**.

## When to use this repo
- You are developing on the Avionics repo and need an environment to run/test your code

## How to use this repo
- Clone the repo and initialize the submodules
- From your terminal, stay in the repo's root directory for the git commands to affect **Native**, and cd into `lib/Avionics` for your git commands to affect **Avionics**.
- I.e. you can contribute to both **Native** and **Avionics** using this repo depending on your terminal's directory. 