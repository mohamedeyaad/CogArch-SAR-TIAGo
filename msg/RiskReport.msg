# RiskReport.msg
# This message contains the risk assessment data for structural analysis.

# Risk status (e.g., LOW, MEDIUM, HIGH)
string status

# Number of cracks detected in the structure
int32 crack_count

# Number of anomalies detected in the walls
int32 wall_anomalies

# Number of hollow spaces detected within the structure
int32 hollow_spaces

# Magnitude of the force applied during the assessment
float32 force_magnitude

# Calculated risk score representing the overall risk level
float32 risk_score