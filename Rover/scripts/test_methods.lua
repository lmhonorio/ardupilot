--@param control_output integer - CONTROLE MODE 4
--| '1' # Roll
--| '2' # Pitch
--| '3' # Throttle
--| '4' # Yaw
--| '5' # Lateral
--| '6' # MainSail
--| '7' # WingSail
--| '8' # Walking_Height
--@return number|nil
-- External modules
package.path = package.path .. ';./scripts/modules/?.lua'
local funcs = require("functions")
-- Mission control logic - waypoints XY coordinates to calculate bearing error and setpoints
local last_wp_x, last_wp_y = 0, 0
local current_wp_x, current_wp_y = 5, 5

vh_x, vh_y = -2, 2
vh_yaw = 0
vh_velocity_norm = 2

print("last_wp_x:", last_wp_x, "last_wp_y:", last_wp_y)
print("current_wp_x:", current_wp_x, "current_wp_y:", current_wp_y)
print("vh_x:", vh_x, "vh_y:", vh_y, "vh_yaw:", vh_yaw, "vh_velocity_norm:", vh_velocity_norm)
print("-------------------------------------------------------------------")
-- Get the projected point with some lookahead so we keep pursuing the target waypoint direction
local line_point_x, line_point_y = funcs:lineProjectionPoint(vh_x, vh_y, last_wp_x, last_wp_y, current_wp_x, current_wp_y)
print("line_point_x:", line_point_x, "line_point_y:", line_point_y)

-- The bearing angle between the last and current waypoints
local wp_line_bearing = funcs:calculateBearingBetweenPoints(last_wp_x, last_wp_y, current_wp_x, current_wp_y)
print("wp_line_bearing:", wp_line_bearing)

local heading_error = funcs:mapErrorToRange(wp_line_bearing - vh_yaw)
print("heading_error:", heading_error)

-- Calculate the cross track error from the vehicle to the line between waypoints
local cross_track_error_gain = 1
print("cross_track_error_gain:", cross_track_error_gain)

local cross_track_error = funcs:crossTrackError(vh_velocity_norm, cross_track_error_gain, line_point_x, line_point_y, vh_x, vh_y)
print("cross_track_error:", cross_track_error)

local cross_track_error_sign = funcs:lineSideSignal(last_wp_x, last_wp_y, current_wp_x, current_wp_y, vh_x, vh_y)
print("cross_track_error_sign:", cross_track_error_sign)

-- Return the steering error as the sum of both errors
local steering_error = funcs:mapErrorToRange(heading_error + cross_track_error_sign * cross_track_error)
print("steering_error:", steering_error)