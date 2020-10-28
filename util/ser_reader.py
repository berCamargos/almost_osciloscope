import serial
import time
import matplotlib
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue
import numpy as np

def read_serial(q):
    ser_name = '/dev/ttyACM0'
    
    done = False
    while not done:
        try:
            ser = serial.Serial(ser_name, baudrate=115200)
            done = True
        except Exception as e:
            print(e)
            time.sleep(1)
    
    msg_size = 256*2
    n_loops = 1000
    start = time.time()
    values = []
    

    while True:
        raws = ser.read(msg_size)
        raws = list(zip(raws[::2], raws[1::2]))
        values = [3.3*int.from_bytes(raw, byteorder='little', signed=False)/4096.0 for raw in raws]
        q.put(values)

    print(len(values))
    print(sum(values)/len(values))
    print(time.time() - start)
    print((time.time() - start)/n_loops)
    print((time.time() - start)/(n_loops*msg_size))
    print(1/((time.time() - start)/(n_loops*msg_size)))
    ser.close()
    print('done')

def plot_signal(q):
    freq = 20000
    plot_time_s = 3
    max_expected_voltage = 1
    min_expected_voltage = -1
    max_len = freq*plot_time_s
    fig = plt.figure()
    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2)

    time.sleep(0.1)
    values = []
    new_values = [0]
    x_values = [float(i)/freq for i in range(max_len)]
    frequencies = [float(i)/plot_time_s for i in range(int(max_len/2))]
    while len(new_values) > 0: 
        new_values = []
        while not q.empty():
            new_values.extend(q.get(False))
        values.extend(new_values)
        if len(values) > max_len:
            values = values[len(values) - max_len:]
        else:
            values = [0]*(max_len - len(values)) + values
        ax1.clear()
        ax1.plot(x_values, values)
        ax2.clear()
        fft_values = abs(np.fft.fft(values - np.mean(values)).real)
        fft_values = fft_values[:int(len(fft_values)/2)]
        ax2.plot(frequencies, fft_values)
        if max(values) > max_expected_voltage:
            max_voltage = 3.5
        else:
            max_voltage = max_expected_voltage
        ax1.set_ylim(0, max_voltage)
        ax1.set_xlim(0, plot_time_s)

        ax2.set_xlim(-10, freq/2)
        #ax2.set_ylim(0, 20000)
        plt.pause(0.1)


q = Queue()
p = Process(target=read_serial, args=(q,))
p.start()
plot_signal(q)
p.join()
