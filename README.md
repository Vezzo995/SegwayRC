# SegwayRC

Firmware for **SegwayRC**, a two-wheel self-balancing RC robot.
Arduino Nano · MPU-6050 · 2× NEMA 17 driven by TB6600 drivers.

📦 **3D model, bill of materials and assembly instructions:** [SegwayRC on MakerWorld]([https://makerworld.com/it/models/3152173-segwayrc-arduino-nano-self-balancing-rc-robot#profileId-3560848](https://makerworld.com/en/models/3152173-segwayrc-arduino-nano-self-balancing-rc-robot))

<img width="467" height="491" alt="image" src="https://github.com/user-attachments/assets/95a7a747-e1b1-4eee-8bab-7a10ec5e3d10" />

---

## What it does

Balances on two wheels, drives forward and backward, and spins 360° on the spot, controlled from a standard 2.4 GHz RC transmitter. The angle estimate comes from an MPU-6050 through a complementary filter; a PID controller converts the tilt error into a step frequency for the two steppers.

## Requirements

Arduino IDE, and nothing else — **no external libraries needed**. The sketch only uses `Wire.h` and `util/atomic.h`, both bundled with the IDE.

Keep the `SegwayRC_v1.ino` file inside a folder named `SegwayRC_v1`: the Arduino IDE requires the sketch and its folder to share the same name.

## Pinout

| Pin | Function |
|---|---|
| D2 | PUL — left driver |
| D3 | DIR — left driver |
| D4 | PUL — right driver |
| D5 | DIR — right driver |
| D7 | RC Ch3 — arm / disarm |
| D8 | RC Ch1 — throttle |
| D9 | RC Ch2 — steering |
| A4 / SDA | MPU-6050 SDA |
| A5 / SCL | MPU-6050 SCL |
| D13 | Status LED |

RC inputs are read directly with pin-change interrupts (`PCINT0` and `PCINT2` groups), not with a library.

## Configuration

Everything tunable sits in the `#define` block at the top of the sketch.

| Constant | Default | Meaning |
|---|---|---|
| `MICROPASSI` | 8 | **Must match the TB6600 dip-switches** |
| `PASSI_GIRO` | 200 | Motor steps per revolution (1.8°) |
| `DIAM_RUOTA_M` | 0.105 | Wheel diameter, metres |
| `FREQ_MAX_HZ` | 12500 | Max step frequency per motor |
| `DT_S` | 0.002 | Loop period — 500 Hz |
| `Equilibrio` | −5.0 | Accelerometer mounting offset, degrees |
| `Kp` / `Ki` / `Kd` | 21 / 1 / 0.01 | PID gains |
| `K_RETRO_VELOCITA` | 0.015 | Speed feedback into the angle error |
| `ANGOLO_CADUTA` | 30 | Tilt angle at which the motors cut out |

The IMU runs at ±500 dps and ±4 g, with the DLPF set to roughly 43 Hz.

## First run

1. Set both TB6600 dip-switches to **1/8 microstepping and ~2 A**. If you change the microstepping, change `MICROPASSI` to match — otherwise the relationship between PID output and real speed is wrong and the gains no longer apply.
2. Flash the sketch with the **robot lifted off the ground and the wheels free**.
3. Power up with the robot lying flat and completely still: the gyro offsets are measured during startup.
4. Stand it upright, arm it with Ch3, and let go.

## Tuning

Start with `Kd` at zero. Raise `Kp` until the robot oscillates around vertical, then back it off about 20%. Add `Kd` until the oscillation damps out. Add `Ki` last and keep it small — too much integral makes the robot drift away instead of holding position.

If the robot balances but creeps steadily forward or backward, that's `Equilibrio`, not the gains: it's the mounting angle of the accelerometer, in degrees.

Serial debug output runs at 115200 baud and prints angle, setpoint, PID output and left step frequency every 200 ms. It is rate-limited and non-blocking on purpose — printing on every cycle saturates the transmit buffer and stalls the control loop.

## Known limitations

**The steppers run open loop.** The controller has no way of knowing whether the motors actually executed the steps it commanded. Under hard braking from high speed the motors can lose steps, the position estimate drifts away from reality, and the robot falls. It balances and drives well at moderate speed; it is not a racer.

A closed-loop version with encoder-equipped DC motors is in the works, and will fix exactly this.

**MPU-6050 DLPF latency.** The current filter setting introduces roughly 4.8 ms of delay — more than a full control loop period. Lowering it would improve phase margin at the cost of noise; it's on the list.

## Credits

The balancing algorithm derives from **[Joop Brokking's YABR project](http://www.brokking.net/yabr_main.html)**, adapted and largely rewritten for this hardware. The step generator, the RC failsafe, the acceleration ramp and the non-blocking serial output are new. Thanks to Joop for making his work public.

## Licence

Released under **[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)** — same as the 3D model. You may share and adapt it, with attribution, for non-commercial purposes, provided derivatives keep the same licence.

*Note: Creative Commons licences aren't designed for software. This one is chosen for consistency with the 3D model rather than for legal precision.*
