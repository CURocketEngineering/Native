# Native

Think of native as a phantom flight computer used to run tests on any of our custom libraries used across multiple flight computers. Native was created because it's messy to setup a PlatformIO environment within our submodules and allows us to keep all of our non-hardware unit testing in one place. 

You should no longer clone **Avionics** directly. Instead, clone this repo and then use the submodule to work on **Avionics**.

## When to use this repo
- You are developing on the Avionics repo and need an environment to run/test your code

## How to use this repo
- Clone the repo and initialize the submodules
- From your terminal, stay in the repo's root directory for the git commands to affect **Native**, and cd into `lib/Avionics` for your git commands to affect **Avionics**.
- I.e. you can contribute to both **Native** and **Avionics** using this repo. 