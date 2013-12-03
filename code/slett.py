from visual import *
from visual.filedialog import save_file

floor = box (pos=(0,0,0), length=4, height=0.5, width=4, color=color.blue)
ball = sphere (pos=(0,4,0), radius=1, color=color.red,material=materials.plastic)
ball.velocity = vector(0,-1,0)
dt = 0.01

save_file(file_extensions='.py', x=100, y=100,
          title="Save", mode='w', maxfiles=20)

while 1:
    rate (100)
    ball.pos = ball.pos + ball.velocity*dt
    if ball.y < ball.radius:
        ball.velocity.y = abs(ball.velocity.y)
    else:
        ball.velocity.y = ball.velocity.y - 9.8*dt
