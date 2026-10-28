-- Runs Qualcomm's qrb_amr_mapping_2d.lua, kept byte-identical, on upstream Cartographer. Qualcomm's fork adds the
-- save_pose_to_scv option; upstream rejects any option it does not read ("Key 'save_pose_to_scv' was used the wrong
-- number of times"), so it is removed here. Qualcomm's fork loads qrb_amr_mapping_2d.lua directly.
include "qrb_amr_mapping_2d.lua"
options.save_pose_to_scv = nil
return options
