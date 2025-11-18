# STM32WB55_BLE_Example

Minimal BLE example for the STM32WB55 series demonstrating a small custom GATT server.

This repository contains an STM32 example that advertises as "LedBlink" and exposes a small
custom service that allows a connected client to write commands which control an LED and to
enable notifications for status updates.

## Quick facts

- Target MCU: STM32WB55 series
- Advertised device name: `LedBlink`
- Exposes a custom service with a write characteristic (to control LEDs) and a notify characteristic
- Example server code: `STM32_WPAN/App/custom_app.c` and `STM32_WPAN/App/custom_stm.c`

## Prerequisites

- STM32CubeIDE (recommended) or a GCC Arm toolchain + Makefile
- ST-Link programmer/debugger
- STM32CubeProgrammer (for flashing if not using the IDE debugger)
- A BLE client app on your phone: nRF Connect (recommended) or nRF Toolbox (limited for custom services)

## Build

Recommended: open the `BLE_Example.ioc` project in STM32CubeIDE and build from the IDE.

Command-line (uses the provided Debug makefile):

1. Open a terminal and change to the repository root.
2. Build the project using the makefile in `Debug/`:

	 make -C Debug

After a successful build you will have the output files in the `Debug` folder (ELF/HEX depending on the makefile).

## Flashing

Option A — STM32CubeIDE:

- Use the built-in debugger (Run/Debug) to flash and start the application via ST-Link.

Option B — STM32CubeProgrammer:

- Launch STM32CubeProgrammer, connect to your ST-Link, and load the generated `.elf` or `.hex` file from the `Debug/` folder. Flash and reset the MCU.

After flashing, the example will start advertising automatically as `LedBlink`.

## What the example does

- The device advertises a custom service. The example implements two main behaviors:
	- A write characteristic that accepts simple commands to control an LED.
	- A notify characteristic that can be enabled by the client to receive notifications.

In the example code the advertised local name is set by the `a_AdvData` array in `STM32_WPAN/App/app_ble.c` (value: `LedBlink`).

The custom application logic is implemented in `STM32_WPAN/App/custom_app.c`. The handler for write events is
`CUSTOM_STM_BLINKLED_WRITE_EVT` and is implemented to interpret a small command value to control GPIOs.

Likely commands (as implemented in the example code):
- `0` — LED OFF
- `1` — LED ON
- `2` — TOGGLE LED

Note: If you connect and write to the write-characteristic the MCU will perform the corresponding GPIO action.

## Using nRF Toolbox / nRF Connect

nRF Toolbox is a small collection of Nordic profiles (UART, DFU, etc.). Because this example exposes a custom GATT
service (not the Nordic UART Service), nRF Toolbox may not be able to directly talk to the custom characteristic.

Recommendation: use nRF Connect (Android / iOS) for exploring and interacting with custom services and characteristics.

Short guide — using nRF Connect (recommended):

1. Install and open the nRF Connect app.
2. Tap "Scan" and wait for the device named `LedBlink` to appear.
3. Tap the device name to connect.
4. Expand the listed services to find the custom service (it will be shown by UUID). Locate the writable characteristic (the example registers it as the Blink/Write characteristic) and the notify characteristic.
5. To control the LED: use the "Write" control for the write-characteristic and enter a small byte array. Try the values `00`, `01`, `02` (hex) corresponding to OFF, ON, TOGGLE respectively. If the app requires a full payload, try sending `00 00`, `00 01`, `00 02` (some stacks include an initial length byte).
6. To receive notifications: tap the notify characteristic and enable "Notifications". The device will send notify packets when available.

Short guide — using nRF Toolbox (if you insist):

- If you modify this firmware to expose the Nordic UART Service (NUS), you can use the UART profile in nRF Toolbox to exchange ASCII/bytes easily.
- For the current custom-service firmware, nRF Toolbox will not show the arbitrary custom GATT characteristics — use nRF Connect instead.

## Where to look in the code

- GAP/GATT & advertising setup: `STM32_WPAN/App/app_ble.c`
- Custom service: `STM32_WPAN/App/custom_stm.c` / `STM32_WPAN/App/custom_stm.h`
- Application behavior (write handler / LED control): `STM32_WPAN/App/custom_app.c`

## Troubleshooting

- Device does not appear when scanning:
	- Ensure MCU is powered and ST-Link is disconnected (if ST-Link debugger holds the MCU in reset), or press reset on the board after flashing.
	- Make sure radio is enabled in the firmware (default in this example).

- Writes have no visible effect:
	- Confirm you write to the correct characteristic (use nRF Connect to inspect UUIDs and properties).
	- Try different payload formats as described above.

## License

This project contains code derived from the STM32 WB examples from STMicroelectronics. See the repository LICENSE or the headers in source files for licensing details.

## Contact / Next steps

If you want, I can add a small client example that implements the Nordic UART Service (NUS) so the project works directly with nRF Toolbox UART, or add clear UUIDs and human-friendly names to the GATT table for easier testing with mobile apps.
