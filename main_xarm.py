import time
import csv
import tests.rrt_test as rt
import tests.drrt_test as drt
import tests.ekf_rrt_test as ekf
import logging
import random

# rt.run(animate=True, choice=0)

# a, b, c, d = [], [], [], []
# i = 0
# # for i in range(0, 50):
# while i < 50:
#     try:
#         logging.info("easy xarm - rrt")
#         replan_counter, total_time, initial_planning, counter = \
#             rt.run(run_number=i, animate=False, choice=2)
#         a.append(replan_counter)
#         b.append(total_time)
#         c.append(initial_planning)
#         d.append(counter)
#         i += 1
#     except ValueError:
#         continue
# with open('results_new/rrt_xarm_easy.csv', mode = 'w') as result_file:
#     result_writer = csv.writer(result_file, delimiter=',')
#     for i in range(0, len(a)):
#         result_writer.writerow([a[i], b[i], c[i], d[i]])
#
#
# a, b, c, d = [], [], [], []
# i = 0
# # for i in range(0, 50):
# while i < 50:
#     try:
#         logging.info("hard xarm - rrt")
#         replan_counter, total_time, initial_planning, counter = \
#             rt.run(run_number=i, animate=False, choice=3)
#         a.append(replan_counter)
#         b.append(total_time)
#         c.append(initial_planning)
#         d.append(counter)
#         i += 1
#     except ValueError:
#         continue
# with open('results_new/rrt_xarm_hard.csv', mode = 'w') as result_file:
#     result_writer = csv.writer(result_file, delimiter=',')
#     for i in range(0, len(a)):
#         result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
i = 0
# for i in range(0, 50):
while i < 30:
    try:
        logging.info("easy xarm - drrt")
        random.seed()
        replan_counter, total_time, initial_planning, counter = \
            drt.run(run_number=i, animate=False, choice=2)
        if counter == -1 or counter == -2:
            continue
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
        i += 1
    except ValueError:
        continue
with open('results_new/drrt_xarm_easy.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
i = 0
# for i in range(0, 50):
while i < 30:
    try:
        logging.info("hard xarm - drrt")
        replan_counter, total_time, initial_planning, counter = \
            drt.run(run_number=i, animate=False, choice=3)
        if counter == -1 or counter == -2:
            continue
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
        i += 1
    except ValueError:
        continue
with open('results_new/drrt_xarm_hard.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
i = 0
# for i in range(0, 50):
while i < 30:
    try:
        logging.info("easy xarm - ekf")
        replan_counter, total_time, initial_planning, counter = \
            ekf.run(run_number=i, animate=False, choice=2)
        if counter == -1 or counter == -2:
            continue
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
        i += 1
    except ValueError:
        continue
with open('results_new/ekf_xarm_easy.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])


a, b, c, d = [], [], [], []
i = 0
# for i in range(0, 50):
while i < 30:
    try:
        logging.info("hard xarm - ekf")
        replan_counter, total_time, initial_planning, counter = \
            ekf.run(run_number=i, animate=False, choice=3)
        if counter == -1 or counter == -2:
            continue
        a.append(replan_counter)
        b.append(total_time)
        c.append(initial_planning)
        d.append(counter)
        i += 1
    except ValueError:
        continue
with open('results_new/ekf_xarm_hard.csv', mode = 'w') as result_file:
    result_writer = csv.writer(result_file, delimiter=',')
    for i in range(0, len(a)):
        result_writer.writerow([a[i], b[i], c[i], d[i]])
