# qmd - ESP-IDF MCPWM Motor Control Component

**qmd** is an ESP-IDF component designed to simplify the control of DC motors and servo motors using the MCPWM peripheral. It can operate 12 PWM channels parallely.

## Features

- Easy integration with ESP-IDF  
- Supports DC motor speed and direction control  
- Supports servo motor position control  
- Optimized for ESP32’s MCPWM module  

## Installation

1. Clone this repository into your ESP-IDF project’s `components` directory:
   ```sh
   git clone https://github.com/shvass/qmd.git components/qmd
   ```
2. Build and flash your ESP-IDF project.

## Usage

Include the header in your code:
```c
#include "qmd.hpp"
```

Let me know if you need any modifications! 🚀