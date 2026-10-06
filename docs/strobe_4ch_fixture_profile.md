# Strobe 4CH Fixture Profile

This file documents the DMX personalities and macro channel mapping for the `strobe_4ch` firmware.

## Personalities

### 1) Simple (2ch)
- CH1: Master dimmer (8-bit)
- CH2: Strobe (0=open, 1..255=slow..fast)

### 2) Simple Fine (3ch)
- CH1: Master dimmer coarse
- CH2: Master dimmer fine
- CH3: Strobe

### 3) Simple Macro (3ch)
- CH1: Master dimmer (8-bit)
- CH2: Strobe
- CH3: Macro

### 4) Compatibility (5ch)
- CH1: Dimmer 1 (8-bit)
- CH2: Dimmer 2 (8-bit)
- CH3: Dimmer 3 (8-bit)
- CH4: Dimmer 4 (8-bit)
- CH5: Strobe

### 5) Compatibility Macro (6ch)
- CH1: Dimmer 1 (8-bit)
- CH2: Dimmer 2 (8-bit)
- CH3: Dimmer 3 (8-bit)
- CH4: Dimmer 4 (8-bit)
- CH5: Strobe
- CH6: Macro

### 6) Advanced (10ch)
- CH1: Dimmer 1 coarse
- CH2: Dimmer 1 fine
- CH3: Dimmer 2 coarse
- CH4: Dimmer 2 fine
- CH5: Dimmer 3 coarse
- CH6: Dimmer 3 fine
- CH7: Dimmer 4 coarse
- CH8: Dimmer 4 fine
- CH9: Strobe
- CH10: Macro

## Strobe Channel
- 0 = open (no strobe gating)
- 1..255 = 1..20 Hz mapping

## Macro Channel Mapping (10 speed steps)
- 0 = no macro
- 1..9 = Effect 1 (All On)
- 10..19 = Effect 2, speed 1..10
- 20..29 = Effect 3, speed 1..10
- 30..39 = Effect 4, speed 1..10
- 40..49 = Effect 5, speed 1..10
- 50..59 = Effect 6, speed 1..10
- 60..69 = Effect 7, speed 1..10
- 70..79 = Effect 8, speed 1..10
- 80..89 = Effect 9, speed 1..10
- 90..99 = Effect 10, speed 1..10
- 100..109 = Effect 11, speed 1..10
- 110..119 = Effect 12, speed 1..10
- 120..129 = Effect 13, speed 1..10
- 130..139 = Effect 14, speed 1..10
- 140..149 = Effect 15, speed 1..10
- 150..159 = Effect 16, speed 1..10
- 160..169 = Effect 17, speed 1..10
- 170..179 = Effect 18, speed 1..10
- 180..189 = Effect 19, speed 1..10
- 190..199 = Effect 20, speed 1..10
- 200..209 = Effect 21, speed 1..10
- 210..219 = Effect 22, speed 1..10
- 220..229 = Effect 23, speed 1..10
- 230..239 = Effect 24, speed 1..10
- 240..249 = Effect 25, speed 1..10
- 250..255 = clamped to Effect 25 speed 10

## Macro Effects List
1. All On
2. All Fade
3. Wave 1→4
4. Wave 4→1
5. Single Chase Forward
6. Single Chase Reverse
7. Pair Chase (1+2 / 3+4)
8. Pair Chase (1+4 / 2+3)
9. Bounce Forward
10. Bounce Reverse
11. Theater Chase
12. Random On/Off Mask
13. Random Levels
14. Random Sparkle
15. Random Single Channel
16. Saw Up
17. Saw Down
18. Triangle Fade
19. Breath Halves Opposite
20. Circular Phase Wave
21. Stair Up
22. Stair Down
23. Two-Channel Chase
24. Smooth Random Drift
25. Random Global Flicker

## Notes
- Macros are rendered first, then strobe is applied.
- Thermal/CPU protection can still limit output.
- Advanced personality keeps 16-bit dimmer control path before PWM quantization.
