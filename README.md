# Marble Lift Control

Servo control for the [ViralBallTrack](https://github.com/Makers-Im-Zigerschlitz/ViralBallTrack)
marble lift.

Uses a SM-S4303 continuous rotation servo and a STM32F103 blue pill board.

The potentiometer controls the rotation speed and direction.

## Wiring

- `PA1` - Potentiometer (middle pin, the other two pins are connected to 3V3 and GND)
- `PB9` - Servo data (white wire)
