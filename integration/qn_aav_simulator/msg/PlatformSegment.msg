# A finite local segment. Path is in map. For a stationary SURFACE_PATH,
# positive duration is a bounded native Otter trim wait before the next segment;
# moving marine segments retain a nominal duration (native completion remains
# state based), and qn conversion segments retain reference duration semantics.
string operation
nav_msgs/Path path
duration duration
# 0 uses the endpoint's configured native effort; positive values override
# only this segment. REMUS interprets the value as propeller rpm, Otter as
# native surge force. The same value is used by preflight and execution.
float64 propulsion_effort
