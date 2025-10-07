# TEENSY VCU System

This repository contains the Vehicle Control Unit (VCU) software for the UC Davis Formula SAE team, designed to run on the Teensy 4.1 microcontroller.

## Overview

This is the new repository for our VCU system. We are migrating to the Teensy 4.1 platform for improved performance and capabilities. The Teensy 4.1 provides:
- 600 MHz ARM Cortex-M7 processor
- 8 MB Flash memory
- 1 MB RAM
- Multiple CAN bus interfaces
- High-speed serial communication
- Built-in Ethernet support

## Project Structure

```
TEENSY/
├── src/              # Source code files
│   └── main.cpp      # Main application entry point
├── include/          # Header files
├── lib/              # Project-specific libraries
├── test/             # Unit tests
├── platformio.ini    # PlatformIO configuration
└── README.md         # This file
```

## Requirements

- [PlatformIO](https://platformio.org/) - Cross-platform build system
- [Visual Studio Code](https://code.visualstudio.com/) (recommended) with PlatformIO IDE extension
- Teensy 4.1 board

## Getting Started

### Installation

1. Install Visual Studio Code
2. Install the PlatformIO IDE extension
3. Clone this repository:
   ```bash
   git clone https://github.com/UCDFS/TEENSY.git
   cd TEENSY
   ```

### Building the Project

1. Open the project folder in Visual Studio Code
2. PlatformIO will automatically detect the project and install dependencies
3. Build the project using one of these methods:
   - Click the checkmark (✓) icon in the PlatformIO toolbar
   - Use the command palette: `PlatformIO: Build`
   - Run in terminal: `pio run`

### Uploading to Teensy 4.1

1. Connect your Teensy 4.1 board via USB
2. Upload using one of these methods:
   - Click the right arrow (→) icon in the PlatformIO toolbar
   - Use the command palette: `PlatformIO: Upload`
   - Run in terminal: `pio run --target upload`

### Serial Monitor

To view serial output from the Teensy:
- Click the plug icon in the PlatformIO toolbar
- Use the command palette: `PlatformIO: Serial Monitor`
- Run in terminal: `pio device monitor`

## Development

### Adding Libraries

Add library dependencies to `platformio.ini` under the `lib_deps` section:

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
    # Add libraries here
```

### Testing

Run unit tests with:
```bash
pio test
```

## Configuration

The Teensy 4.1 configuration is specified in `platformio.ini`. Modify this file to change:
- Build flags
- Upload settings
- Library dependencies
- Serial monitor settings

## Contributing

1. Create a feature branch
2. Make your changes
3. Test thoroughly
4. Submit a pull request

## Resources

- [Teensy 4.1 Documentation](https://www.pjrc.com/store/teensy41.html)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [Arduino Reference](https://www.arduino.cc/reference/en/)

## Migration Notes

This repository replaces the previous VCU system. See `vcu_software.ino` in the old repository for legacy implementation details.

## License

Copyright UC Davis Formula SAE Team

## Support

For questions or issues, please contact the VCU software team or open an issue on GitHub.