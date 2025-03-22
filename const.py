janky = False

l_wrist_angle = 230
l_wrist_algae_angle = 120

enc_offsets = (-250, -127, -185, 0)
l1_ext = 0
l2_ext = 0.075
l3_ext = 0.475
l4_ext = 0.825
al_ext = 0.0
ah_ext = 0.3

if not janky:
    enc_offsets = (180, 170, -85, 160)
    l1_ext = 0
    l2_ext = 0.075
    l3_ext = 0.365
    l4_ext = 0.825

    # Last Working Setpoints
    l1_ext = 0
    l2_ext = 0.075
    l3_ext = 0.365
    l4_ext = 0.835

    # Babbit Settings
    l1_ext = 0
    l2_ext = 0.075
    l3_ext = 0.365
    l4_ext = 0.835

    al_ext = 0.0
    ah_ext = 0.3
# We are extending the wrist when we receive, goto the seagul maneuver, and when we score