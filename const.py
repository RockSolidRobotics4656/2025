janky = False

enc_offsets = () #drivetrain wheel offsets

if janky:
    enc_offsets = (-250, -127, -185, 0)
if not janky:
    enc_offsets = (180, 170, -85, 160)