; touch probe: measure a wax block (top at 8 mm) and the bare plate (0 mm)
; the reported heights show the machine's own errors
; @stock wax 24 24 8 -30 0
G28
M6 T2              ; pick up the touch probe
G90
G0 X-30 Y0 Z14
G38.2 Z4 F120      ; block centre
G0 Z14
G0 X-36 Y-6
G38.2 Z4 F120      ; block corner
G0 Z14
G0 X-24 Y6
G38.2 Z4 F120      ; opposite corner
G0 Z14
G0 X25 Y0
G38.2 Z-4 F120     ; bare plate
G0 Z14
G0 X0 Y0 Z40
M114
