janky = False

l_wrist_home_angle = 0
l_wrist_angle = 230
enc_offsets = () #drivetrain wheel offsets
l1_ext = 0
l2_ext = 0.075
l3_ext = 0.475
l4_ext = 0.825

if janky:
    enc_offsets = (-250, -127, -185, 0)
if not janky:
    enc_offsets = (180, 170, -85, 160)
    l1_ext = 0
    l2_ext = 0.075
    l3_ext = 0.365
    l4_ext = 0.825

    #l2_ext = 0.072
    #l3_ext = 0.358

    l1_ext = 0
    l2_ext = 0.075
    l3_ext = 0.365
    l4_ext = 0.825
# We are extending the wrist when we receive, goto the seagul maneuver, and when we score