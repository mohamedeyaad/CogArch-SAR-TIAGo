# TriageReport.msg
# This message contains the result of a triage process for a detected victim.

# Standard message header
std_msgs/Header header

# Consciousness state of the victim
# Values: "conscious", "unconscious"
string consciousness

# Responsiveness of the victim
# Values: "yes", "no"
string responsive

# Estimated injuries detected via image analysis
# Example: ["bleeding", "suspected_fracture"]
string[] injuries

# Triage priority based on victim condition
# Values: "URGENT", "STABLE"
string priority

# Victim's estimated position in the environment
geometry_msgs/Pose location
