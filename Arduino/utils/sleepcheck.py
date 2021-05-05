from datetime import datetime
import time

def check_sleep(amount):
    start = datetime.now()
    time.sleep(amount)
    end = datetime.now()
    delta = end-start
    return delta.seconds + delta.microseconds/1000000.

def p_wait(wait_time):
    """Wait x seconds (precise)."""
    t_0 = time.perf_counter()
    t_1 = t_0
    while t_1 - t_0 < wait_time:
        t_1 = time.perf_counter()
    return t_1 - t_0

def p_sleep(wait_time):
    """Wait x seconds (precise)."""
    t_0 = time.perf_counter()
    time.sleep(wait_time)
    t_1 = time.perf_counter()
    return t_1 - t_0

def p_timer(t_0, wait_time):
    """Wait x seconds (precise)."""
    t_1 = t_0
    while t_1 - t_0 < wait_time:
        t_1 = time.perf_counter()
    return t_1 - t_0

def p_sleep_wait(wait_time, min_sleep_time=0.002):
    """Wait x seconds (precise)."""
    t_0 = time.perf_counter()
    if wait_time > min_sleep_time:
        time.sleep(wait_time - min_sleep_time)
    t_1 = time.perf_counter()
    t_2 = t_1
    while t_2 - t_0 < wait_time:
        t_2 = time.perf_counter()
    return t_2 - t_0, t_1 - t_0

def p_sleep_wait_timer(t_0, wait_time):
    """Wait x seconds (precise)."""
    t_1 = time.perf_counter()
    if wait_time - (t_1 - t_0) > 0.016:
        time.sleep(wait_time - 0.016)
    t_1 = time.perf_counter()
    while t_1 - t_0 < wait_time:
        t_1 = time.perf_counter()
    return t_1 - t_0

sleep_period = 0.050
iterations = 1000
print('Time Estimate = ', sleep_period * iterations)

for j in range(1,21):
    sum_abs = 0
    sum_sleep = 0
    t_0 = time.perf_counter()
    for i in range(iterations):
        measured_period, sleep_period = p_sleep_wait(0.001 * j, 0.002)
        error = 0.001 * j - measured_period
        sum_abs += abs(error)
        sum_sleep += abs(sleep_period)
    avg_abs = (sum_abs / iterations) * 1000
    avg_sleep = (sum_sleep / iterations) * 1000
    t_1 = time.perf_counter()
    print(j, "Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000),      'Total Time = %0.6fs' % (t_1 - t_0),  'Sleep Period = %0.6fms' % (avg_sleep * 1000))

# sum_abs = 0
# for i in range(iterations):
    # measured_period = check_sleep(sleep_period)
    # error = sleep_period - measured_period
    # sum_abs += abs(error)
# avg_abs = (sum_abs / iterations) * 1000
# print("Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000) )

# sum_abs = 0
# for i in range(iterations):
    # measured_period = p_sleep(sleep_period)
    # error = sleep_period - measured_period
    # sum_abs += abs(error)
# avg_abs = (sum_abs / iterations) * 1000
# print("Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000) )

# sum_abs = 0
# for i in range(iterations):
    # measured_period = p_wait(sleep_period)
    # error = sleep_period - measured_period
    # sum_abs += abs(error)
# avg_abs = (sum_abs / iterations) * 1000
# print("Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000) )

# sum_abs = 0
# for i in range(iterations):
    # t_0 = time.perf_counter()
    # measured_period = p_timer(t_0, sleep_period)
    # error = sleep_period - measured_period
    # sum_abs += abs(error)
# avg_abs = (sum_abs / iterations) * 1000
# print("Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000) )

# sum_abs = 0
# for i in range(iterations):
    # measured_period = p_sleep_wait(sleep_period)
    # error = sleep_period - measured_period
    # sum_abs += abs(error)
# avg_abs = (sum_abs / iterations) * 1000
# print("Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000) )

# sum_abs = 0
# for i in range(iterations):
    # t_0 = time.perf_counter()
    # measured_period = p_sleep_wait_timer(t_0, sleep_period)
    # error = sleep_period - measured_period
    # sum_abs += abs(error)
# avg_abs = (sum_abs / iterations) * 1000
# print("Average error is %0.6fms, total error = " % avg_abs, '%0.6fms' % (sum_abs * 1000) )
