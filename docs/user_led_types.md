# User-defined LED types (`/userLEDtypes.json`)

Prototype: LED types that are not built into the firmware can be described in a JSON file on the controller's
file system. Each entry is a named preset of the **Customize bus** settings (channel map, 16-bit, gap bits,
inversion, timing). Selecting it on the LED settings page fills those settings; the bus is then saved as a
customized bus of its `base` type, so configs stay valid without the file and no firmware change is needed per chip.

## Installing

Upload a file named `userLEDtypes.json` with the file editor (`http://<wled-ip>/edit`).
The types appear at the end of every LED type dropdown under **User types**.
See `tools/userLEDtypes.example.json` for a working example (Govee G1151).

## Format (`"v": 1`)

| Field | Required | Meaning |
|---|---|---|
| `name` | yes | Name shown in the dropdown |
| `base` | yes | Built-in single-wire digital type ID the bus is stored as (driver and default timing), e.g. `22` = WS281x |
| `channels` | yes | 1-6 wire channels in transmit order, each one of `R`, `G`, `B`, `W`, `WW`, `CW` |
| `bits` | no | `8` (default) or `16` bits per channel |
| `gaps` | no | Zero bits sent before each channel, one 0-16 value per channel (missing = 0). When any is > 0 pixels are bit-packed (not byte aligned). Govee G1151: `[2, 2, 2, 2, 2]` |
| `invert` | no | Channel indexes (0-based) whose value is inverted |
| `invertOutput` | no | Invert the data signal polarity |
| `timing` | no | `t0h`, `t0l`, `t1h`, `t1l` in ns and `reset` in µs; missing values come from `base` |
| `notes` | no | Free text, ignored |

Invalid entries are skipped (see the browser console for the reason).

## Notes

- Bit-packed pixels (`gaps` > 0) are supported on the ESP32 RMT, I2S/LCD, PARLIO and parallel SPI drivers;
  BitBang and ESP8266 drivers refuse such buses.
- A customized bus whose settings match a user type exactly is shown with the user type's name when the page loads.
