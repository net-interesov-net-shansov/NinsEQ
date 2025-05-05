# NinsEQ - Dynamic Equalizer VST3 Plugin

A dynamic equalizer VST3 plugin built with JUCE framework. Features three filter bands (highpass, mid bell, lowpass) with dynamic processing capabilities on the mid bell band.

## Features

- Three-band EQ with highpass, mid bell, and lowpass filters
- Dynamic processing on the mid bell band with threshold, ratio, attack, and release controls
- Real-time spectrum analyzer
- Interactive frequency response curve display
- Available as VST3 plugin and standalone application

## Requirements

- CMake 3.22 or newer
- C++ compiler with C++20 support
- VS Code (for build instructions below)

## Building the Project with VS Code

### Prerequisites

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install the following VS Code extensions:
   - [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
   - [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)

### Build Steps

1. Clone the repository and open it in VS Code
2. Configure CMake:
   - Press `Ctrl+Shift+P` to open the command palette
   - Type "CMake: Configure" and select it
   - Choose the compiler you want to use (e.g., Visual Studio, GCC, Clang)

3. Build the project:
   - Press `Ctrl+Shift+P` again
   - Type "CMake: Build" and select it
   - Alternatively, click on the "Build" button in the CMake Tools status bar at the bottom of VS Code

4. To run the standalone application:
   - Press `Ctrl+Shift+P`
   - Type "CMake: Run Without Debugging" and select it
   - Choose the "NinsEQ_Standalone" target

### Plugin Installation

After building, you'll find the VST3 plugin in the build folder. To install it:

- Windows: Copy the `.vst3` file to `C:\Program Files\Common Files\VST3`
- macOS: Copy the `.vst3` file to `/Library/Audio/Plug-Ins/VST3`
- Linux: Copy the `.vst3` file to `/usr/local/lib/vst3`

## Using the Plugin

1. **Highpass Filter**:
   - Use the "HP Freq" knob to set the cutoff frequency
   - Use the "HP Q" knob to adjust the resonance

2. **Mid Band**:
   - Use "Mid Freq", "Mid Q", and "Mid Gain" to shape the bell filter
   - Use "Threshold", "Ratio", "Attack", and "Release" to control the dynamics:
     - When the input signal exceeds the threshold, the mid band gain is dynamically reduced
     - Higher ratio values result in more gain reduction
     - Attack and release control how quickly the dynamic processing responds

3. **Lowpass Filter**:
   - Use the "LP Freq" knob to set the cutoff frequency
   - Use the "LP Q" knob to adjust the resonance

## License

See [LICENSE.md](LICENSE.md) file for details.
