import solver2d_py as s2

w = s2.create_world()
print("world:", w)
print(type(w)) 

# ground
s2.add_ground(w)
b = s2.add_box(w, 0.0, 1, 2, 1)

# # a box above
# b = s2.add_box(w, 0.3, 3.0, 0.5, 0.5, density=1.0)

for i in range(60):
    s2.step_tgs_soft(w, 1/60, iterations=4, pos_iterations=2, warm_start=True)
    print("box  pose:", s2.get_body_pose(b).x,"", s2.get_body_pose(b).y,"", s2.get_body_pose(b).angle)

 


s2.destroy_world(w)
print("destroyed")