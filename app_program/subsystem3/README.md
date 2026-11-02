Subsystem 3 (Line + Barcode) — Integration Handover

What to change in a fresh repo to add my subsystem.

My folder: app_program/subsystem3/
- Add: line_follow.c, barcode.c, task_barcode.c
- Replace: subsystem3.c, subsystem3.h, line_barcode.h
- Replace with no-op stub: subsystem3_test.c
- Delete: task_consumer.c

Kernel config (shared): device/adc/sysdepend/rp2040/adc_cnf_sysdep.h
- DEVCONF_ADC_PIN_INIT_0 → TRUE (GP26 centre)
- DEVCONF_ADC_PIN_INIT_1 → TRUE (GP27 right)
- DEVCONF_ADC_PIN_INIT_2 → TRUE only if barcode analog (GP28)

TEAM.md
- Claim GP26, GP27 (+GP28 if analog barcode).

Switches — top of line_barcode.h
- LINE_SELF_DRIVE: 1 = my task drives (standalone test). 0 = Navigation drives (full system — set 0 when integrated, or we fight for Motion).
- LINE_DEBUG, BARCODE_DEBUG: 1 = serial prints, 0 = silent.
- BARCODE_ANALOG: 1 = GP28 analog, 0 = GP4 digital.
- LINE_CALIBRATE (in line_follow.c): must be 0 to drive (1 = sensor-readout only, no steering).

Don’t touch
- subsystem1/2/4/5, navigation, app_main, kernel, build files.

Depends on
- Glenn’s motion (subsystem2): motion_set_turn_level, motion_send. Must work + both encoders counting (GP5 left, GP7 right) or Motion stall-faults.

Pins
- GP26 = centre line (ADC0), GP27 = right line (ADC1), GP28/GP4 = barcode.
