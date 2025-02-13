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

## Repo + PlatformIO Setup

### Downloading the needed programs
1. Download vscode: https://code.visualstudio.com/  
   - VS Code is our IDE of choice because of integration with PlatformIO IDE, popularity,
    and widespread usage in industry. 
2. Download Git: https://git-scm.com/downloads
	- After you finish the download and setup, type `git` in your terminal to see if it's recognized
	- We use GitHub to version control all of our code on the cloud
	- Git is the program to copy the remote code locally and push edits up to the cloud

### Creating a project folder 
1. Open up your terminal/command prompt (Search "CMD" if on Windows)
2. Make a rocketry directory (Creates a folder called "rocketry")
    ```bash
    mkdir rocketry
    ``` 
3. Go into the directory

	```bash
	PS C:\Users\ethan> cd rocketry
	PS C:\Users\ethan\rocketry>
	```
	
4. Clone the repo (Copies the code from the cloud into this folder using Git)
    ```bash
    git clone https://github.com/CURocketEngineering/Native.git
    ```
    ```bash 
    PS C:\Users\ethan\rocketry> git clone https://github.com/CURocketEngineering/Native.git
    Cloning into 'Native'...
    remote: Enumerating objects: 53, done.
    remote: Counting objects: 100% (53/53), done.
    remote: Compressing objects: 100% (33/33), done.
    remote: Total 53 (delta 15), reused 47 (delta 12), pack-reused 0 (from 0)
    Receiving objects: 100% (53/53), 9.84 KiB | 9.84 MiB/s, done.
    Resolving deltas: 100% (15/15), done.
    PS C:\Users\ethan\rocketry>
    ```

### Setting up the submodules
Within the Native repo are some sub repos. The primary sub repo
is called "Avionics" and contains all our generic flight computer code.

1. Go into the Native directory we pulled using Git
    ```
    cd Native
    ```
2. Initialize submodules (Clones the sub repos)
    ```bash
    git submodule init
    ```
3. Update submodules (Gets the latest version of the sub repos)
    ```bash
    git submodule update
    ```

### PlatformIO Integration with VS Code
#### Opening the Native folder with VS Code
With VS Code, you open a folder and work out of it...
1.  Open VS Code 
2. Click file in the top left
3. Click open folder and navigate to the `rocketry/Native` folder and click "select folder"

#### Downloading PlatformIO IDE
1. Click on "extensions" on the far left side within VS Code
4. Search "PlatformIO IDE"
5. Click on "PlatformIO IDE" and click install
6. Wait for PlatformIO (pio) to install all of the dependencies for the Native project
7. Click on the little flask icon on the bottom blue ribbon to run tests
8. You should see all the tests pass

## Running Tests

### Running Standard Tests
1. Click the PIO icon
2. Under Native>Advanced click "Test" to run all the unit tests

### Running CSV Tests
1. Naviage to the Flight Logs Folder in the Microsoft Teams Drive (CURE>Engineering Division>Software>Flight-Logs)
2. Download the CSV files in the Active Aero Feb 18 - 2023 Launch Folder
3. Place the CSV files in the Native/data folder (if one doesn't exist create it)
4. Click the PIO icon
5. Under Native>Advanced click "Test" to run all the unit tests