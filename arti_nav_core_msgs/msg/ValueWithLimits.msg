# A value (coordinate, angle, linear or angular velocity) with optional relative limits.

# The intended value. A non-finite value (NaN or +-Inf) means that the value is not relevant at all. Because the limits
# are relative to this value, they are also not relevant in this case, and the value is unbounded.
float64 value

# Whether `upper_limit` and `lower_limit` should be taken into consideration. Setting this to false means that the
# value should be reached exactly.
bool has_limits

# Upper limit relative to `value`. Must be greater than or equal to zero. Unused if `has_limits` is false.
float64 upper_limit

# Lower limit relative to `value`. Must be less than or equal to zero. Unused if `has_limits` is false.
float64 lower_limit
