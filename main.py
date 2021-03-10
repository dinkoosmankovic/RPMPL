import time
import tests.rrt_test as rt
import tests.drrt_test as drt
import tests.ekf_rrt_test as ekf

_, _, _, t = ekf.run(animate=True, choice=0, pred_step=2)
while t == -2:
    time.sleep(2.0)
    _, _, _, t = ekf.run(animate=False, choice=0, pred_step=2)

# for i in range(0, 50):
#     try:
#         dt.run(run_number=i, animate=False, choice=0)
#     except ValueError as ve:
#         print(ve.args)