-- Runs Qualcomm's qrb_amr_localization_2d.lua, kept byte-identical, on upstream Cartographer, as
-- qrb_amr_mapping_2d_upstream.lua does for the mapping file it includes.
include "qrb_amr_localization_2d.lua"
options.save_pose_to_scv = nil
return options
