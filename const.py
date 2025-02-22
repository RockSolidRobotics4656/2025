janky = False

enc_offsets = () #drivetrain wheel offsets
l1_ext = 0
l2_ext = 0.075
l3_ext = 0.475
l4_ext = 0.8

if janky:
    enc_offsets = (-250, -127, -185, 0)
if not janky:
    enc_offsets = (180, 170, -85, 160)
    l1_ext = 0
    l2_ext = 0.1
    l3_ext = 0.37
    l4_ext = 0.8

    l2_ext = 0.065
    l3_ext = 0.36