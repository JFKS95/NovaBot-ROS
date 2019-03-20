#!/usr/bin/python3
from scipy.signal import butter, lfilter, find_peaks, peak_prominences
import pyaudio
from scipy.fftpack import fft, fftfreq, fftshift
import wave
import numpy as np
import time
import datetime
import heapq
#import rospy
#from std_msgs.msg import String

width = 2
channels = 1
RATE = 44100
record_seconds = 5
chunk = RATE*record_seconds
file2write=open(str("Datalog_Audio.txt"),'w')
window = np.blackman(chunk)
p = pyaudio.PyAudio()

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y



stream = p.open(format =
                p.get_format_from_width(width),
                channels = channels,
                rate = RATE,
                input = True,
                output = True,
                frames_per_buffer=chunk)

print("Start Reccording")
data = stream.read(chunk, False)
#start = time.time()
print("Recording complete")
#while (time.time() - start) <= record_seconds:
stream.close()

indata = np.array(wave.struct.unpack("%dh"%(len(data)/width),\
                                     data))



#data = stream.read(chunk, False)
#if data:
#    stream.write(data)

#stream.close()


def run():
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.signal import freqz

    # Sample rate and desired cutoff frequencies (in Hz).
    fs = float(RATE)
    lowcut = 150.0
    highcut = 950.0


    # Plot the frequency response for a few different orders of bandpass filters
    '''
    plt.figure(1)
    plt.clf()
    
    for order in [3, 4, 5]:
        b, a = butter_bandpass(lowcut, highcut, fs, order=order)
        w, h = freqz(b, a, worN=2000)
        plt.plot((fs * 0.5 / np.pi) * w, abs(h), label="order = %d" % order)

    plt.plot([0, 0.5 * fs], [np.sqrt(0.5), np.sqrt(0.5)],
             '--', label='sqrt(0.5)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain')
    plt.grid(True)
    plt.legend(loc='best')
    '''

    #Pass the recorded sound signal through a butterworth bandpass filter and plot
    T = record_seconds
    
    nsamples = T * fs
    
    t = np.linspace(0, T, nsamples, endpoint=False)

    x = indata # the unfiltered audio

    y = butter_bandpass_filter(x, lowcut, highcut, fs, order=4) # filtered audio, limiting frequencies to 200-1000Hz

    '''
    plt.figure(2)
    plt.clf()
    plt.plot(t, indata, label='Unfiltered sound signal')
    plt.plot(t, y, label='Filtered sound signal (Hz)')
    plt.xlabel('time (seconds)')
    plt.hlines([-a, a], 0, T, linestyles='--')
    plt.grid(True)
    plt.axis('tight')
    plt.legend(loc='upper left')
    '''
    
    #Get the FFt of the filtered sound signal and plot

    fftdata = np.fft.rfft(y*window)
    xfftdata = np.fft.rfft(x*window)

    which = abs(fftdata)**2
    
    which2 = which[1:].argmax() + 1 

    y0,y1,y2 = np.log(fftdata[which2-1:which2+2:])
    x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
    # find the frequency and output it
    thefreq = (which2+x1)*RATE/chunk
    print("The frequency component with the highest gain is %f Hz." % (abs(thefreq)))

    xf = np.linspace(0, 1.0/(2.0*(1/fs)), nsamples//2, endpoint=False)

    ffthz = 2.0/nsamples*abs(fftdata[1:])
    '''
    plt.figure(3)
    plt.clf()
    plt.semilogy(xf, 2.0/nsamples*abs(fftdata[1:]), '-b')
    plt.grid()
    plt.xlim(0,1500)
    #plt.show()
    '''

    # Get the frequency peak prominences and plot

    peaks, _ = find_peaks(ffthz, distance=50, height=30)
    #peaks, _ = find_peaks(ffthz)
    
    freqArray = xf[peaks] 
    gainArray = ffthz[peaks]

    intfreqArray = freqArray.astype(int)
    intgainArray = gainArray.astype(int)


    maxIndiceArray = intgainArray.argsort()[-5:][::-1]
    '''
    print(maxIndiceArray)
    print(maxIndiceArray[0])
    print(maxIndiceArray[1])
    print(maxIndiceArray[2])

    print('1st highest gain freq: ', freqArray[maxIndiceArray[0]])
    print('2nd highest gain freq: ', freqArray[maxIndiceArray[1]])
    print('3rd highest gain freq: ', freqArray[maxIndiceArray[2]])
    '''
    
    print('Peaks indices: ', peaks)
    
    print('Frequencies: ', freqArray)

    print('Gains: ', gainArray)


    
    ind = np.argpartition(ffthz[peaks], -len(xf[peaks])) #[-2:]


    ordfreq = np.flip(intfreqArray[ind])
    ordgain = np.flip(intgainArray[ind])

    ordfreq2 = intfreqArray[maxIndiceArray]
    ordfreq2 = intgainArray[maxIndiceArray]

    
    print('Ordered frequencies: ', ordfreq)

    print('Ordered gains: ',ordgain)
    
    print('Ordered frequencies2: ', ordfreq2)

    print('Ordered gains2: ', ordfreq2)

    if(len(ordfreq) == 1):

        file2write.write("OK")
        print("OK")

    if(len(ordfreq) == 0):

        file2write.write("NODATA")
        print("NODATA")
    if(len(ordfreq) >= 2):
        
        if(ordfreq[0] < ordfreq[1]):
            if(ordgain[0] > ordgain[1]):
                file2write.write("A001")
                print("A001")
            if(ordgain[1] > ordgain[0]):
                file2write.write("FAIL")
                print("FAIL")
        if(ordfreq[0] > ordfreq[1]):
            if(ordgain[0] > ordgain[1]):
                file2write.write("A002")
                print("A002")
            if(ordgain[1] > ordgain[0]):
                file2write.write("FAIL")
                print("FAIL")


    plt.figure(1)
    #plt.figure(4)
    plt.clf()
    #plt.semilogy(xf, ffthz, '-r')
    #plt.semilogy(xf[peaks], ffthz[peaks], "x")

    plt.plot(xf, ffthz)
    plt.plot(xf[peaks], ffthz[peaks], "x")
    
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain (dB)')
    #plt.vlines(x=xf[peaks], ymin=contour_heights, ymax=ffthz[peaks])
    plt.grid(True)
    plt.axis('tight')
    plt.xlim(0,1500)

    plt.show()
    time.sleep(10)

#print("Processing data")


run()
file2write.close()

