import solver2d_py as s2

w = s2.create_world()
print("world:", w)
print(type(w)) 

# ground
s2.add_ground(w)
# b = s2.add_box(w, 0.0, 2, 2, 1)

# a trapezoid (convex CCW)
top_w, bottom_w, h = 0.6, 1.6, 1.0
trap = s2.add_4pt_polygon(
    w, 0.0, 0.5,
    [[-bottom_w/2, -h/2],
     [ bottom_w/2, -h/2],
     [ top_w/2,     h/2],
     [-top_w/2,     h/2]],
)

for i in range(60):
    s2.step_tgs_soft(w, 1/60, iterations=4, pos_iterations=2, warm_start=True)
    #print("box  pose:", s2.get_body_pose(b).x,"", s2.get_body_pose(b).y,"", s2.get_body_pose(b).angle)
    #print("box velocity:", s2.get_body_velocity(b).vx,"",s2.get_body_velocity(b).vy,"",s2.get_body_velocity(b).omega,"",)
    print("box  pose:", s2.get_body_pose(trap).x,"", s2.get_body_pose(trap).y,"", s2.get_body_pose(trap).angle)
    print("box velocity:", s2.get_body_velocity(trap).vx,"",s2.get_body_velocity(trap).vy,"",s2.get_body_velocity(trap).omega,"",)

s2.destroy_world(w)
print("destroyed")