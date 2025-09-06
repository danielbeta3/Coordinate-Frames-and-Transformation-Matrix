from src.aircraft_util import alpha_beta_from_body, build_vectors, aircraft_state, plot_aircraft_with_vectors

# Example: velocity components in BODY axes
alpha, beta, Vhat_b, V_air_b = alpha_beta_from_body(100, 0, 10)

# Build reference vectors
vecs = build_vectors(Vhat_b, alpha, beta)

# Generate aircraft state dictionary
state = aircraft_state(alpha, beta, 100, 0, 10, 0, 0, 0, 0, 0.1, 0)
print(state["angles"])

# Plot with STL model
plot_aircraft_with_vectors(r"plots/aircraft.stl", alpha, beta, vecs)