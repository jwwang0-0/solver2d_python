import solver2d_py as s2

w = s2.create_world()
print("world:", w)

s2.step_tgs_soft(w, dt=1.0/60.0, iterations=10, warm_start=True)
print("stepped (tgs_soft)")

s2.destroy_world(w)
print("destroyed")