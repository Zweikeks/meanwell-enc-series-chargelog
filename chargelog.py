#!/usr/bin/env python3

"""ChargeLog
Programs or monitors the Mean Well ENC-series of chargers.

Original development environment:
Python 3.6.0 + pip install pyserial as admin
Windows 7 64 bit
ENC-120-12

Supported:
Python 3.4.4 + pip install pyserial==3.0.1
Windows XP 32 bit
ENC-120-12

Connect the charger to a PC with a USB to 3V3 TTL bridge. For example
FTDI TTL-232R-3V3 works fine (yet needs modification to plug into CN31
of the charger). Or use a module based on Silicon Labs CP210x.

Charger CN31              USB to 3V3 TTL bridge
Pin 2 GND         ------- GND
Pin 3 RX (input)  ---<--- TX (ouput)
Pin 4 TX (output) --->--- RX (input)
(Do NOT connect the 3.3V. This is a power supply output on both ends.)

Version 1.0: Argument checks for ENC-120-12 only.
             No temperature monitoring.
Version 1.1: Minor change in print to file needed by Python 3.4.4.
"""

import signal
import argparse
import sys
import os.path
import serial
import time

version_major = 1
version_minor = 0

mockup = False

#---log------------------------------------------------------------------------

key_vmajor      = 'versionmajor'
key_vminor      = 'versionminor'
key_info        = 'info'
key_comport     = 'comport'
key_logfile     = 'logfile'
key_readinterv  = 'readinterv'
key_graphinterv = 'graphinterv'
key_graphwidth  = 'graphwidth'
key_cname       = 'chargername'
key_cmodel      = 'chargermodel'
key_ctype       = 'chargertype'
key_constcurr   = 'constcurr'
key_constvolt   = 'constvolt'
key_floatvolt   = 'floatvolt'
key_tapercurr   = 'tapercurr'
key_config      = 'config'
key_startepoch  = 'startepoch'
key_epoch       = 'epoch'
key_time        = 'time'
key_elapsed     = 'elapsed'
key_vout        = 'vout'
key_iout        = 'iout'
key_stage       = 'stage'
key_graph       = 'graph'

m_logfile = None

def log_readold(logfile, x_maxchars):
    result = True
    startepoch  = -1.0
    readinterv  = -1.0
    graphinterv = -1.0
    elapsed     = 0
    read_count  = 0
    filter_vout = []
    filter_iout = []

    try:
        with open(logfile) as m_logfile: #automatically closed when leaving the block
            for line in m_logfile:
                line = line.split(',')
                line = list(map(str.strip, line)) #strip whitespace from all elements

                #read the first startepoch (multiple startepoch keys should have the same value anyway)
                if startepoch < 0:
                    try:
                        sidx = line.index(key_startepoch) + 1
                        startepoch = float(line[sidx])
                    except ValueError:
                        pass

                #read a new pair of intervals (may change throughout the log file)
                try:
                    ridx = line.index(key_readinterv)  + 1
                    gidx = line.index(key_graphinterv) + 1
                    readinterv  = float(line[ridx])
                    graphinterv = float(line[gidx])
                    next_graph_count = read_count + int(graphinterv / readinterv)
                    next_store_count = read_count + int(next_graph_count / x_maxchars)
                except ValueError:
                    pass

                #read a set of values and their time stamp, mimic what mon_loop() is doing
                if (startepoch >= 0) and (readinterv > 0) and (graphinterv > 0):
                    try:
                        eindex  = line.index(key_epoch) + 1
                        vindex  = line.index(key_vout)  + 1
                        iindex  = line.index(key_iout)  + 1
                        vout    = float(line[vindex])
                        iout    = float(line[iindex])
                        elapsed = float(line[eindex]) - startepoch
                        filter_vout.append(vout)
                        filter_iout.append(iout)

                        if read_count >= next_store_count:
                            mon_store(filter_vout, filter_iout, elapsed)
                            filter_vout = []
                            filter_iout = []
                            next_store_count = read_count + int(next_graph_count / x_maxchars)

                        if read_count >= next_graph_count:
                            next_graph_count = read_count + int(graphinterv / readinterv)

                        read_count  += 1

                    except ValueError:
                        pass

        if startepoch < 0:
            print_error('Couldn\'t read %s from %s.' % (key_startepoch, logfile))
            result = False
        if (readinterv < 0) or (graphinterv < 0):
            print_error('Couldn\'t read %s or %s from %s.' % (key_readinterv, key_graphinterv, logfile))
            result = False
        if read_count == 0:
            print_error('No readings found in %s.' % (logfile))
            result = False
    except (OSError, IOError) as inst:
        print_error('Couldn\'t open %s.' % logfile)
        print_error(inst)
        result = False

    return (result, startepoch, read_count, elapsed)

def log_open(logfile, x_maxchars):
    result = True
    startepoch = -1.0
    read_count = 0
    elapsed = 0
    global m_logfile

    if (os.path.isfile(logfile)):
        print_warning('File exists.')
        answer = input('Append to exisiting logfile (y/n)? ')
        if answer == 'y':
            (rd_result, startepoch, read_count, elapsed) = log_readold(logfile, x_maxchars)
            if rd_result:
                if mockup:
                    epoch = elapsed + 3600 #an hour gap (arbitrary)
                    mockepoch_set(epoch)
                else:
                    epoch = time.time()
                #compare distances on x axis, limit 5%
                if epoch - startepoch > 0:
                    if elapsed / (epoch - startepoch) < 0.05:
                        gap = epoch - startepoch - elapsed
                        if gap > 86400:
                            print_info('Last log entry is %d day(s) old.' % int(gap/86400))
                        elif gap > 3600:
                            print_info('Last log entry is %d hour(s) old.' % int(gap/3600))
                        print_info('Log data will be squeezed to the sides of the graph, the middle beeing a huge void.')
                        answer = input('Continue (y/n)? ')
                        if answer != 'y':
                            result = False
            else:
                result = False
        else:
            result = False

        if result:
            try:
                m_logfile = open(logfile, 'a')
            except (OSError, IOError) as inst:
                print_error('Couldn\'t open %s.' % logfile)
                print_error(inst)
                result = False

        if result:
            log_print()
            log_print('%s, %d, %s, %d' % (key_vmajor, version_major, key_vminor, version_minor))
            log_print('%s, %s' % (key_info, 'all voltages are in Volt; all currents in Ampere; time in seconds'))
    else:
        try:
            m_logfile = open(logfile, 'w') #overwrites
        except (OSError, IOError) as inst:
            print_error('Couldn\'t create %s.' % logfile)
            print_error(inst)
            result = False

    return (result, startepoch, read_count, elapsed)

def log_close():
    if m_logfile is not None:
        m_logfile.close()

def log_print(*args, **kwargs):
    print(file = m_logfile, *args, **kwargs)

#---print----------------------------------------------------------------------

m_alignment = '%-9s'
m_info_prefix    = 'INFO:'
m_warning_prefix = 'WARNING:'
m_error_prefix   = 'ERROR:'

def print_info(*args, **kwargs):
    print(m_alignment % m_info_prefix, end='')
    print(*args, **kwargs)

def print_warning(*args, **kwargs):
    print(m_alignment % m_warning_prefix, end='')
    print(*args, **kwargs)

def print_error(*args, **kwargs):
    print(m_alignment % m_error_prefix, end='')
    print(*args, **kwargs)

#prints out all in \x notation
def print_bytestr(bytestr, end = '\n'):
    print('b\'', end = '')
    for val in bytestr:
        print('\\x%02x' % val, end = '')
    print('\'', end = end)

def cfg2str(config):
    lut = ['disabled', '-3 mV', '-4 mV', '-5 mV']
    if (config >= 0) and (config <= 3):
        return lut[config]
    else:
        return 'error'

def stage2str(stage):
    lut = ['error', '1', '2', 'error', '3']
    if (stage >= 0) and (stage <= 4):
        return lut[stage]
    else:
        return 'error'

def print_settings(csettings):
    print('    constcurr: %.2f A'  % csettings[ckey_cc])
    print('    constvolt: %.1f V'  % csettings[ckey_cv])
    print('    floatvolt: %.1f V'  % csettings[ckey_fv])
    print('    tapercurr: %.2f A'  % csettings[ckey_tc])
    print('    config:    %s'      % cfg2str(csettings[ckey_cfg]))
    print()

#---graph----------------------------------------------------------------------

m_volt_max   = 14.5  #V
m_curr_max   = 8.0   #A
m_sec_max    = 10800 #seconds
m_x_maxchars = 80    #characters
m_y_maxchars = 32    #characters
m_mark_over  = 'x'
m_buffer     = []
m_gkey_volt  = 'volt'
m_gkey_curr  = 'curr'
m_gkey_sec   = 'sec'
m_curve      = []

def gconfig_set(volt_max, curr_max, sec_max, x_maxchars, y_maxchars):
    global m_volt_max, m_curr_max, m_sec_max, m_x_maxchars, m_y_maxchars
    m_volt_max   = volt_max
    m_curr_max   = curr_max
    m_sec_max    = sec_max
    m_x_maxchars = x_maxchars
    m_y_maxchars = y_maxchars

def gconfig_get():
    return (m_volt_max, m_curr_max, m_sec_max, m_x_maxchars, m_y_maxchars)

def gclear():
    global m_buffer
    m_buffer = []
    last_first_line = '+' + '-' * (m_x_maxchars - 1) + '+'
    middle_line     = '|' + ' ' * (m_x_maxchars - 1) + '|'
    m_buffer.append(last_first_line)
    for yidx in range(1, m_y_maxchars):
        m_buffer.append(middle_line)
    m_buffer.append(last_first_line)

def gplot(xc, yc, letter):
    global m_buffer
    xc = round(xc)
    yc = round(yc)
    if xc > m_x_maxchars:
        xc = m_x_maxchars
        letter = m_mark_over
    if xc < 0:
        xc = 0
        letter = m_mark_over
    if yc > m_y_maxchars:
        yc = m_y_maxchars
        letter = m_mark_over
    if yc < 0:
        yc = 0
        letter = m_mark_over
    m_buffer[yc] = m_buffer[yc][0:xc] + letter + m_buffer[yc][xc+1:] #work around immutable strings

def gplot_volt(val, sec):
    xc = sec / m_sec_max * m_x_maxchars
    yc = m_y_maxchars - val / m_volt_max * m_y_maxchars #invert, buffer index 0.. prints top to bottom
    gplot(xc, yc, 'v')

def gplot_curr(val, sec):
    xc = sec / m_sec_max * m_x_maxchars
    yc = m_y_maxchars - val / m_curr_max * m_y_maxchars
    gplot(xc, yc, 'i')

def gcurve_append(vout, iout, sec):
    m_curve.append({m_gkey_volt: vout, m_gkey_curr: iout, m_gkey_sec: sec})

def gplot_curve():
    for valset in m_curve:
        gplot_volt(valset[m_gkey_volt], valset[m_gkey_sec])
        gplot_curr(valset[m_gkey_curr], valset[m_gkey_sec])

def gprint():
    for line in m_buffer:
        print(line)

def glog():
    for line in m_buffer:
        log_print('%s, %s' % (key_graph, line))

#simple sanity check
def gtest():
    gconfig_set(14.5, 2.5, 1200, 72, 29)
    print(gconfig_get())
    gclear()
    for sec in range(0, 1201, 40): #lines from corner to corner
        v = sec * 14.5 / 1200
        i = 2.5 - sec * 2.5 / 1200
        gcurve_append(v, i, sec)
    gplot_curve()
    gplot_volt(15, 600)    #overshoots
    gplot_volt(-0.5, 600)
    gplot_volt(8, 1210)
    gplot_volt(8, -8)
    gplot_curr(2.6, 700)
    gplot_curr(-0.5, 700)
    gplot_curr(1.3, 1210)
    gplot_curr(1.3, -8)
    gprint()

#---mockup---------------------------------------------------------------------

m_mockepoch = 0.0
m_mockend = 10800 #3 hours, suits mockcurve()

def mockepoch_set(epoch):
    global m_mockepoch
    m_mockepoch = epoch

def mockepoch_get():
    return m_mockepoch

def mockepoch_inc(sec):
    global m_mockepoch
    m_mockepoch += sec

def mockend():
    if m_mockepoch > m_mockend:
        return True
    else:
        return False

#in 3 hours through all stages
def mockcurve():
    if m_mockepoch < 3600:
        vout  = 15.4 - 3600 / (m_mockepoch + 234) #simply made up, nothing profound...
        iout  = 8.0
        stage = 1
    elif m_mockepoch < 7200:
        vout = 14.4
        iout = 51840 / m_mockepoch - 6.4
        stage = 2
    else:
        vout  = 13.8
        iout  = 51000 / m_mockepoch - 6.3
        if iout < 0.02:
            iout = 0.02
        stage = 3
    return (vout, iout, stage)

#emulates replies as if charging a battery
def mockreply(expected_first_byte):
    bytestr = b'' #default, no reply (com error)
    (vout, iout, stage) = mockcurve()
    if expected_first_byte == reply_read_id:
        bytestr = reply_read_id + b'\x00\x11' #ENC-120-12
        #bytestr = reply_read_id + b'\x00\x82' #ENC-360-24
    if expected_first_byte == reply_read_constcurr:
        val = 800 #8.0A
        bytestr = reply_read_constcurr + val.to_bytes(2, byteorder='big')
    if expected_first_byte == reply_read_constvolt:
        val = 144 #14.4V
        bytestr = reply_read_constvolt + val.to_bytes(2, byteorder='big')
    if expected_first_byte == reply_read_floatvolt:
        val = 138 #13.8V
        bytestr = reply_read_floatvolt + val.to_bytes(2, byteorder='big')
    if expected_first_byte == reply_read_tapercurr:
        val =  80 #0.8A
        bytestr = reply_read_tapercurr + val.to_bytes(2, byteorder='big')
    if expected_first_byte == reply_read_config:
        bytestr = reply_read_config + b'\x00\x01' #-3 mV temp compensation
    if expected_first_byte == reply_read_vout:
        vout = round(vout * 10)
        bytestr = reply_read_vout + vout.to_bytes(2, byteorder='big')
    if expected_first_byte == reply_read_iout:
        iout = round(iout * 100)
        bytestr = reply_read_iout + iout.to_bytes(2, byteorder='big')
    if expected_first_byte == reply_read_state:
        stage = 2**(stage - 1)
        bytestr = reply_read_state + stage.to_bytes(2, byteorder='big')
    if bytestr != b'':
        bytestr = add_checksum(bytestr)
    return bytestr

#---com------------------------------------------------------------------------

m_serial = None

#commands to the charger are in the form:       reply is:
#cmd_write pload_hi pload_lo csum_hi csum_lo    none
#cmd_read  csum_hi csum_lo                      cmd_read+0x40 pload_hi pload_lo csum_hi csum_lo

#save values to EEPROM of device
cmd_write_constcurr = b'\x01' #CC, constant current setting of stage 1
cmd_write_constvolt = b'\x02' #CV, constant voltage setting of stage 2
cmd_write_floatvolt = b'\x03' #FV, float voltage setting of stage 3
cmd_write_tapercurr = b'\x04' #TC, current level of transition to stage 3
cmd_write_config    = b'\x05' #temperature compensation

#read values
cmd_read_constcurr = b'\x41' #CC
cmd_read_constvolt = b'\x42' #CV
cmd_read_floatvolt = b'\x43' #FV
cmd_read_tapercurr = b'\x44' #TC
cmd_read_config    = b'\x45' #temperature compensation
cmd_read_state     = b'\x46' #status
cmd_read_vout      = b'\x47' #measured voltage at the output terminals
cmd_read_iout      = b'\x48' #measured current through the output terminals
cmd_read_id        = b'\x49' #device identifier
cmd_read_temp      = b'\x4a' #battery temperature, external sensor at CN31, degree Celsius, two's complement

#reply to read commands
reply_read_constcurr = b'\x81'
reply_read_constvolt = b'\x82'
reply_read_floatvolt = b'\x83'
reply_read_tapercurr = b'\x84'
reply_read_config    = b'\x85'
reply_read_state     = b'\x86'
reply_read_vout      = b'\x87'
reply_read_iout      = b'\x88'
reply_read_id        = b'\x89'
reply_read_temp      = b'\x8a'

ckey_cc  = 'constcurr'
ckey_cv  = 'constvolt'
ckey_fv  = 'floatvolt'
ckey_tc  = 'tapercurr'
ckey_cfg = 'config'

def com_open(comport):
    err = False
    global m_serial
    if not mockup:
        try:
            m_serial = serial.Serial(
                port=comport,
                baudrate=4800,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,

                #set timeout to x seconds, read returns immediately when the requested number
                #of bytes are available, otherwise wait until the timeout expires and return
                #all bytes that were received until then
                timeout = 0.1
            )

            if not m_serial.isOpen():
                m_serial.open()
        except serial.SerialException as inst:
            print_error(inst)
            m_serial = None
            err = True
    return err

def com_close():
    if m_serial is not None:
        m_serial.close()

def calc_checksum(bytestr):
    sum = 0
    for val in bytestr:
        sum += val
    return sum

def add_checksum(bytestr):
    checksum = calc_checksum(bytestr)
    bytestr = bytestr + checksum.to_bytes(2, byteorder='big')
    return bytestr

def verify_checksum(bytestr):
    result = False
    len_bytestr = len(bytestr)
    csum1 = calc_checksum(bytestr[0:len_bytestr-2])
    csum2 = bytestr[len_bytestr-2] * 0x100 + bytestr[len_bytestr-1]
    if csum1 == csum2:
        result = True
    return result

def enc_tx(bytestr):
    bytestr = add_checksum(bytestr)
    if not mockup:
        m_serial.reset_input_buffer()
        m_serial.reset_output_buffer()
        m_serial.write(bytestr)

def enc_rx(expected_length, expected_first_byte, payload_length):
    result = (False, 0)
    if mockup:
        bytestr = mockreply(expected_first_byte)
    else:
        bytestr = m_serial.read(expected_length)
    if len(bytestr) == expected_length:
        if bytestr[0] == expected_first_byte[0]:
            if verify_checksum(bytestr):
                result = (True, bytestr[1:payload_length+1])
    return result

#returns model = 0,1,2,3 (power) and type = 0,1,2 (voltage)
def check_charger_id():
    err = False
    cmodels = [1,2,4,8]
    ctypes  = [1,2,4]
    cmodel  = -1
    ctype   = -1
    cmodel_names = ['120', '180', '240', '360']
    ctype_names  = ['12', '24', '48']

    enc_tx(cmd_read_id)
    (result, payload) = enc_rx(5, reply_read_id, 2)

    if result:
        if payload[0] != 0:
            err = True

        cmodel = (payload[1] & 0xf0) >> 4
        ctype  = (payload[1] & 0x0f)

        try:
            cmodel = cmodels.index(cmodel)
            ctype = ctypes.index(ctype)
            cname = 'ENC-' + cmodel_names[cmodel] + '-' + ctype_names[ctype]
        except ValueError:
             err = True

        if err:
            print_error('Unknown ID read from charger: ', end = '')
            print_bytestr(payload)
    else:
        print_error('No (valid) answer from charger.')
        err = True

    if not err:
        if (cmodel != 0) or (ctype != 0):
            print_warning('Detected charger: %s.' % cname)
            print_warning('The software has been tested only with the ENC-120-12')
            print_warning('so far. Parameter checks/limits are only valid for the')
            print_warning('ENC-120-12.')
            answer = input('Continue anyway (y/n)? ')
            if answer != 'y':
                err = True

    if err:
        return (False, -1, -1, 'unknown')
    else:
        return (True, cmodel, ctype, cname)

def read_settings():
    result = True
    csettings = {}
    cmd_sets = ((cmd_read_constcurr, reply_read_constcurr, ckey_cc, 100),
                (cmd_read_constvolt, reply_read_constvolt, ckey_cv,  10),
                (cmd_read_floatvolt, reply_read_floatvolt, ckey_fv,  10),
                (cmd_read_tapercurr, reply_read_tapercurr, ckey_tc, 100),
                (cmd_read_config,    reply_read_config,    ckey_cfg,  1))
    for cmd_set in cmd_sets:
        enc_tx(cmd_set[0])
        (result, payload) = enc_rx(5, cmd_set[1], 2)
        if result:
            csettings[cmd_set[2]] = (payload[0] * 0x100 + payload[1]) / cmd_set[3]
        else:
            csettings[cmd_set[2]] = 'error'
            result = False
        time.sleep(0.01)

    if result:
        csettings[ckey_cfg] = int(csettings[ckey_cfg]) #change type

    return (result, csettings)

#currently only for ENC-120-12 (cmodel 0, ctype 0)
def check_settings(csettings, cmodel, ctype):
    result = True
    if (csettings[ckey_cc] < 2.4) or (csettings[ckey_cc] > 8.0):
        print_warning('constcurr = %.1f, allowed range 2.4 <= constcurr <= 8.0.' % csettings[ckey_cc])
        result = False
    if (csettings[ckey_cv] < 9.0) or (csettings[ckey_cv] > 15.0):
        print_warning('constvolt = %.1f, allowed range 9.0 <= constvolt <= 15.0.' % csettings[ckey_cv])
        result = False
    if (csettings[ckey_fv] < 9.0) or (csettings[ckey_fv] > csettings[ckey_cv]):
        print_warning('floatvolt = %.1f, allowed range 9.0 <= floatvolt <= constvolt(%.1f).' % (csettings[ckey_fv], csettings[ckey_cv]))
        result = False
    if (csettings[ckey_tc] < 0.8) or (csettings[ckey_tc] > 2.4):
        print_warning('tapercurr = %.1f, allowed range 0.8 <= tapercurr <= 2.4.' % csettings[ckey_tc])
        result = False
    if (csettings[ckey_cfg] < 0) or (csettings[ckey_cfg] > 3):
        print_warning('config = %d, allowed range 0 <= config <= 3.' % csettings[ckey_cfg])
        result = False
    return result

def check_charging():
    err = False

    enc_tx(cmd_read_iout)
    (result, payload) = enc_rx(5, reply_read_iout, 2)
    if result:
        iout = (payload[0] * 0x100 + payload[1]) / 100
    else:
        print_error('Read iout from charger failed.')
        err = True
    time.sleep(0.01)

    enc_tx(cmd_read_state)
    (result, payload) = enc_rx(5, reply_read_state, 2)
    if result:
        state = payload[0] * 0x100 + payload[1]
    else:
        print_error('Read state from charger failed.')
        err = True
    time.sleep(0.01)

    if not err:
        if (iout > 0.1) or (state != 4):
            print_warning('It seems a battery is charging (iout>100mA and/or Stage != 3).')
            print_warning('You should disconnect the battery during programming.')
            answer = input('Continue (y/n)? ')
            if answer != 'y':
                err = True

    return err

#---program--------------------------------------------------------------------

def prg_settings(csettings, cnew):
    err = False
    sleeptime_after_prg = 0.05 #charger needs about 20ms for EEPROM access

    if cnew[ckey_cc]:
        val = int(csettings[ckey_cc] * 100)
        payload = val.to_bytes(2, byteorder='big')
        bytestr = cmd_write_constcurr + payload
        enc_tx(bytestr)
        time.sleep(sleeptime_after_prg)

        enc_tx(cmd_read_constcurr)
        (result, payload_rd) = enc_rx(5, reply_read_constcurr, 2)
        if not (result and (payload_rd == payload)):
            print_error('Read back constcurr from charger failed.')
            err = True
        time.sleep(0.01)

    if not err:
        if cnew[ckey_cv]:
            val = int(csettings[ckey_cv] * 10)
            payload = val.to_bytes(2, byteorder='big')
            bytestr = cmd_write_constvolt + payload
            enc_tx(bytestr)
            time.sleep(sleeptime_after_prg)

            enc_tx(cmd_read_constvolt)
            (result, payload_rd) = enc_rx(5, reply_read_constvolt, 2)
            if not (result and (payload_rd == payload)):
                print_error('Read back constvolt from charger failed.')
                err = True
            time.sleep(0.01)

    if not err:
        if cnew[ckey_fv]:
            val = int(csettings[ckey_fv] * 10)
            payload = val.to_bytes(2, byteorder='big')
            bytestr = cmd_write_floatvolt + payload
            enc_tx(bytestr)
            time.sleep(sleeptime_after_prg)

            enc_tx(cmd_read_floatvolt)
            (result, payload_rd) = enc_rx(5, reply_read_floatvolt, 2)
            if not (result and (payload_rd == payload)):
                print_error('Read back floatvolt from charger failed.')
                err = True
            time.sleep(0.01)

    if not err:
        if cnew[ckey_tc]:
            val = int(csettings[ckey_tc] * 100)
            payload = val.to_bytes(2, byteorder='big')
            bytestr = cmd_write_tapercurr + payload
            enc_tx(bytestr)
            time.sleep(sleeptime_after_prg)

            enc_tx(cmd_read_tapercurr)
            (result, payload_rd) = enc_rx(5, reply_read_tapercurr, 2)
            if not (result and (payload_rd == payload)):
                print_error('Read back tapercurr from charger failed.')
                err = True
            time.sleep(0.01)

    if not err:
        if cnew[ckey_cfg]:
            val = csettings[ckey_cfg]
            payload = val.to_bytes(2, byteorder='big')
            bytestr = cmd_write_config + payload
            enc_tx(bytestr)
            time.sleep(sleeptime_after_prg)

            enc_tx(cmd_read_config)
            (result, payload_rd) = enc_rx(5, reply_read_config, 2)
            if not (result and (payload_rd == payload)):
                print_error('Read back config (tcomp) from charger failed.')
                err = True
            time.sleep(0.01)

    if err:
        print_error('Programming failed.')

    return err

def prg(args):
    err  = False
    cnew = {ckey_cc: False, ckey_cv: False, ckey_fv: False, ckey_tc: False, ckey_cfg: False}

    if args.tempcoeff is not None:
        if args.tempcoeff not in [0, 3, 4, 5]:
            print_error('Argument tempcoeff = %d, allowed values 0, 3, 4 and 5.' % args.tempcoeff)
            err = True

    if not err:
        err = com_open(args.comport)

    if not err:
        (result, cmodel, ctype, cname) = check_charger_id()
        print('Detected charger: %s (Model %d, Type %d).' % (cname, cmodel, ctype))
        print()
        err = not result

    if not err:
        (result, csettings) = read_settings()
        if result:
            print('Current settings read from charger:')
            print_settings(csettings)
        else:
            print_error('Couldn\'t read settings from charger.')
            err = True

    if not err:
        if args.constcurr is not None:
            csettings[ckey_cc] = round(args.constcurr * 100) / 100
            cnew[ckey_cc] = True
        if args.constvolt is not None:
            csettings[ckey_cv] = round(args.constvolt * 10) /10
            cnew[ckey_cv] = True
        if args.floatvolt is not None:
            csettings[ckey_fv] = round(args.floatvolt * 10) / 10
            cnew[ckey_fv] = True
        if args.tapercurr is not None:
            csettings[ckey_tc] = round(args.tapercurr * 100) /100
            cnew[ckey_tc] = True
        if args.tempcoeff is not None:
            if args.tempcoeff > 0:
                csettings[ckey_cfg] = args.tempcoeff - 2
            else:
                csettings[ckey_cfg] = 0
            cnew[ckey_cfg] = True
        print('New settings:')
        print_settings(csettings)

        result = check_settings(csettings, cmodel, ctype)
        if not result:
            print_warning('New settings are out of range. This may damage your hardware.')
            answer = input('Continue anyway (y/n)? ')
            if answer != 'y':
                err = True

        if not err:
            err = check_charging()

        if not err:
            err = prg_settings(csettings, cnew)

        if not err:
            print_info('Changes are fully in effect after a power cycle (wait until LED')
            print_info('is off before switching on again).')

#---monitor--------------------------------------------------------------------

m_readinterv_default  = 2.0
m_graphinterv_default = 60.0
m_graphwidth_default  = 80

def mon_read():
    enc_tx(cmd_read_vout)
    (result, payload) = enc_rx(5, reply_read_vout, 2)
    if result:
        vout = (payload[0] * 0x100 + payload[1]) / 10
        print('%4.1f V, ' % vout, end = '')
        log_print('%s, %4.1f, ' % (key_vout, vout), end = '')
        vout_ok = True
    else:
        print('error V, ', end = '')
        log_print('%s, error, ' % key_vout, end = '')
        vout_ok = False
    time.sleep(0.01)

    enc_tx(cmd_read_iout)
    (result, payload) = enc_rx(5, reply_read_iout, 2)
    if result:
        iout = (payload[0] * 0x100 + payload[1]) / 100
        print('%5.2f A, ' % iout, end = '')
        log_print('%s, %5.2f, ' % (key_iout, iout), end = '')
        iout_ok = True
    else:
        print('error A, ', end = '')
        log_print('%s, error, ' % key_iout, end = '')
        iout_ok = False
    time.sleep(0.01)

    enc_tx(cmd_read_state)
    (result, payload) = enc_rx(5, reply_read_state, 2)
    if result:
        stage = payload[0] * 0x100 + payload[1]
        stage_str = stage2str(stage)
        print('Stage ' + stage_str)
        log_print('%s, %s' % (key_stage, stage_str))
    else:
        print('Stage error')
        log_print('%s, error' % key_stage)
    time.sleep(0.01)

    if vout_ok and iout_ok:
        return (True, vout, iout)
    else:
        return (False, 0, 0)

def mon_store(filter_vout, filter_iout, elapsed):
    if filter_vout and filter_iout:
        vout_mean = sum(filter_vout) / len(filter_vout)
        iout_mean = sum(filter_iout) / len(filter_iout)
        gcurve_append(vout_mean, iout_mean, elapsed)

def mon_graph(elapsed):
    (volt_max, curr_max, sec_max, x_maxchars, y_maxchars) = gconfig_get()
    sec_max = elapsed
    gconfig_set(volt_max, curr_max, sec_max, x_maxchars, y_maxchars)
    gclear()
    gplot_curve()
    gprint()
    glog()

def mon_wait_until(startepoch, elapsed):
    while True:
        delta = startepoch + elapsed - time.time()
        if delta > 0:
            time.sleep(delta/2)
        else:
            break

def mon_loop(readinterv, graphinterv, startepoch, read_count):
    filter_vout = []
    filter_iout = []

    (volt_max, curr_max, sec_max, x_maxchars, y_maxchars) = gconfig_get()
    next_graph_count = read_count + int(graphinterv / readinterv)
    next_store_count = read_count + int(next_graph_count / x_maxchars)

    while True:
        if mockup:
            epoch       = mockepoch_get()    #starts from 0 (or elapsed time of logfile to append)
            localtime_t = time.gmtime(epoch) #use gmtime here, ease verification without local variations
        else:
            epoch       = time.time()        #seconds since 1.1.1970
            localtime_t = time.localtime(epoch)

        if startepoch < -0.5: #startepoch invalid
            startepoch = epoch
            elapsed = 0
            print('startepoch: %.1f s' % startepoch)
            print()
            log_print('%s, %.1f' % (key_startepoch, startepoch))
        else:
            elapsed = epoch - startepoch

        localtime_str = time.strftime("%Y-%m-%d %H:%M:%S", localtime_t)
        print(localtime_str + ', ', end = '')
        print('%12.1f s, ' % epoch, end = '')
        min_el, sec_el = divmod(elapsed, 60)
        hour_el, min_el = divmod(min_el, 60)
        print('%02d:%02d:%02d, ' % (hour_el, min_el, sec_el), end = '')
        log_print('%s, %s, '   % (key_time,  localtime_str), end = '')
        log_print('%s, %12.1f, ' % (key_epoch, epoch), end = '')
        log_print('%s, %02d:%02d:%02d, ' % (key_elapsed, hour_el, min_el, sec_el), end = '')

        (result, vout, iout) = mon_read()
        if result:
            filter_vout.append(vout)
            filter_iout.append(iout)

        #filter and store in time resolution of next graph
        if read_count >= next_store_count:
            mon_store(filter_vout, filter_iout, elapsed)
            filter_vout = []
            filter_iout = []
            next_store_count = read_count + int(next_graph_count / x_maxchars)

        if read_count >= next_graph_count:
            mon_graph(elapsed)
            next_graph_count = read_count + int(graphinterv / readinterv)

        read_count += 1

        if mockup:
            mockepoch_inc(readinterv)
            if mockend():
                #mon_graph(elapsed)
                break
        else:
            mon_wait_until(startepoch, readinterv * read_count)

def mon(args):
    err = False

    if args.readinterv is None:
        args.readinterv = m_readinterv_default
    if args.readinterv < 0.1: #avoid probs with calculation, not useful anyway, would be faster than communication to charger
        print_error('Argument readinterv = %g, allowed range 0.1 <= readinterv.' % args.readinterv)
        err = True

    if args.graphinterv is None:
        args.graphinterv = m_graphinterv_default
    if args.graphinterv < args.readinterv:
        print_error('Argument graphinterv = %g, allowed range readinterv(%g) <= graphinterv.' % (args.graphinterv, args.readinterv))
        err = True

    if args.graphwidth is None:
       args.graphwidth = m_graphwidth_default
    if args.graphwidth < 0:
       print_error('Argument graphwidth = %d, allowed range 0 <= graphwidth.' % args.graphwidth)
       err = True

    if not err:
        print('Parameters:')
        print('    comport:     %s'       % args.comport)
        print('    logfile:     %s'       % args.logfile)
        print('    readinterv:  %s s'     % args.readinterv)
        print('    graphinterv: %s s'     % args.graphinterv)
        print('    graphwidth:  %s chars' % args.graphwidth)
        print()

    if not err:
        err = com_open(args.comport)

    if not err:
        (result, cmodel, ctype, cname) = check_charger_id()
        print('Detected charger: %s (Model %d, Type %d).' % (cname, cmodel, ctype))
        print()
        err = not result

    if not err:
        (result, startepoch, read_count, elapsed) = log_open(args.logfile, args.graphwidth)
        err = not result

    if not err:
        if startepoch > -0.5: #startepoch valid, old log read
            if mockup:
                localtime_t = time.gmtime(startepoch)
            else:
                localtime_t = time.localtime(startepoch)
            localtime_str = time.strftime("%Y-%m-%d %H:%M:%S", localtime_t)
            print('Got %d readings from existing log started at %s.' % (read_count, localtime_str))
            print('    startepoch: %.1f s' % startepoch)
            print('    elapsed:    %.1f s' % elapsed)
            print()

        #readinterv and graphinterv must be on same line to be accepted by log_readold()
        log_print('%s, %s, %s, %s, %s, %g, %s, %g, %s, %d' %
            (key_comport,    args.comport,    key_logfile,     args.logfile,
             key_readinterv, args.readinterv, key_graphinterv, args.graphinterv,
             key_graphwidth, args.graphwidth))
        log_print('%s, %s, %s, %d, %s, %d' %
            (key_cname, cname, key_cmodel, cmodel, key_ctype, ctype))
        if startepoch > -0.5: #startepoch valid, old log read
            log_print('%s, %.1f' % (key_startepoch, startepoch))

        (result, csettings) = read_settings()
        if result:
            print('Current settings read from charger:')
            print_settings(csettings)
            log_print('%s, %.1f, %s, %.1f, %s, %.1f, %s, %.1f, %s, %d' %
                (key_constcurr, csettings[ckey_cc], key_constvolt, csettings[ckey_cv],
                 key_floatvolt, csettings[ckey_fv], key_tapercurr, csettings[ckey_tc],
                 key_config,    csettings[ckey_cfg]))
        else:
            print_error('Couldn\'t read settings from charger.')
            err = True

    if not err:
        result = check_settings(csettings, cmodel, ctype)
        if not result:
            print_warning('Charger settings are out of range.')
            answer = input('Continue (y/n)? ')
            if answer != 'y':
                err = True

    if not err:
        #round-half-up with int( + 0.5) required here (round() of Python 3 is round-towards-even)
        gconfig_set(int(2 * csettings[ckey_cv] + 1) / 2, #steps of 0.5 with 0.1...0.5 margin
                    int(2 * csettings[ckey_cc] + 1) / 2,
                    elapsed, args.graphwidth, round(args.graphwidth/2.5))

        if startepoch > -0.5: #startepoch valid, old log read, show graph from old data
            gclear()
            gplot_curve()
            gprint()
            glog()

    if not err:
        mon_loop(args.readinterv, args.graphinterv, startepoch, read_count)

#---debug----------------------------------------------------------------------

def dbg_read_all_raw():
    cmd_sets = ((cmd_read_constcurr, reply_read_constcurr, 'constcurr'),
                (cmd_read_constvolt, reply_read_constvolt, 'constvolt'),
                (cmd_read_floatvolt, reply_read_floatvolt, 'floatvolt'),
                (cmd_read_tapercurr, reply_read_tapercurr, 'tapercurr'),
                (cmd_read_config,    reply_read_config,    'config'),
                (cmd_read_state,     reply_read_state,     'state'),
                (cmd_read_vout,      reply_read_vout,      'vout'),
                (cmd_read_iout,      reply_read_iout,      'iout'),
                (cmd_read_id,        reply_read_id,        'id'),
                (cmd_read_temp,      reply_read_temp,      'temp'))
    for cmd_set in cmd_sets:
        enc_tx(cmd_set[0])
        (result, payload) = enc_rx(5, cmd_set[1], 2)
        print(('%-9s' % cmd_set[2]) + ': ', end = '')
        if result:
            print_bytestr(payload)
        else:
            print('error')
        time.sleep(0.01)

def dbg(args):
    err = com_open(args.comport)

    if not err:
        if args.rawread:
            dbg_read_all_raw()
        else:
            print_error('One of the arguments of the sub-command \'dbg\'must be specified.')

#---parser---------------------------------------------------------------------

def parse_print_help():
    print('Programs or monitors the Mean Well ENC-series of chargers.')
    print()
    print('usage: chargelog prg <comport> [-cc <constcurr>] [-cv <constvolt>]')
    print('          [-tc <tapercurr>] [-fv <floatvolt>] [-tcomp <tempcoeff>]')
    print('       chargelog mon <comport> <logfile> [-ri <readinterv>]')
    print('          [-gi <graphinterv>] [-width <graphwidth>]')
    print('       chargelog dbg <comport> [-rawread]')
    print()
    print('       constcurr: constant current of stage 1 in Ampere')
    print('       constvolt: constant voltage of stage 2 in Volt')
    print('       tapercurr: current level of transition to stage 3 in Ampere')
    print('       floatvolt: constant voltage of stage 3 in Volt')
    print('       tempcoeff: temperature coefficient 0|3|4|5 in Millivolt')
    print('                  (0 disabled, 3 for -3 mV etc.)')
    print()
    print('       readinterv:  one reading every readinterv seconds (float)')
    print('       graphinterv: draw graph all graphinterv seconds (float)')
    print('       graphwidth:  width of the graph in number of characters')
    print()
    print('       rawread: issues all read commands and prints the results')
    print()
    print('example: chargelog prg com3 -cc 5.2 -tcomp 4')
    print('         chargelog mon com1 c:\\temp\\foo.log -ri 60 -gi 3600')
    print('         chargelog mon com1 c:\\temp\\foo.log -ri 0.6 -gi 43.2')
    print('         chargelog dbg com8 -rawread')
    print()
    print('Note that \'chargelog mon\' can be called with an existing log file')
    print('to continue it.')

def parse_cmdline():
    if len(sys.argv) == 1:
        parse_print_help()
        sys.exit(0)

    if len(sys.argv) == 2:
        if (sys.argv[1] == '-h') or (sys.argv[1] == '-H') or (sys.argv[1] == '-?'):
            parse_print_help()
            sys.exit(0)

    main_parser = argparse.ArgumentParser(add_help = False)
    subparsers  = main_parser.add_subparsers(dest='subcmd')

    prg_parser = subparsers.add_parser('prg', add_help = False)
    prg_parser.add_argument('comport')
    prg_parser.add_argument('-cc',    dest = 'constcurr', type = float)
    prg_parser.add_argument('-cv',    dest = 'constvolt', type = float)
    prg_parser.add_argument('-tc',    dest = 'tapercurr', type = float)
    prg_parser.add_argument('-fv',    dest = 'floatvolt', type = float)
    prg_parser.add_argument('-tcomp', dest = 'tempcoeff', type = int)

    mon_parser = subparsers.add_parser('mon', add_help = False)
    mon_parser.add_argument('comport')
    mon_parser.add_argument('logfile')
    mon_parser.add_argument('-ri',    dest = 'readinterv',  type = float)
    mon_parser.add_argument('-gi',    dest = 'graphinterv', type = float)
    mon_parser.add_argument('-width', dest = 'graphwidth',  type = int)

    dbg_parser = subparsers.add_parser('dbg', add_help = False)
    dbg_parser.add_argument('comport')
    dbg_parser.add_argument('-rawread', action='store_true')

    args = main_parser.parse_args() #exits in case of wrong args

    return args

#---exit-----------------------------------------------------------------------

#clean exit with ctrl+c
def signal_handler(signal, frame):
        com_close()
        log_close()
        sys.exit(0)

#---main-----------------------------------------------------------------------

#sys.argv = ['chargelog', 'mon', 'com1', 'c:\\temp\\foo.log', '-ri', '60', '-gi', '3600'] #for IDLE debugging

#gtest()
#sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

print('  _____ _                          _')
print(' / ____| |                        | |')
print('| |    | |__   __ _ _ __ __ _  ___| |     ___   __ _')
print('| |    | \'_ \ / _` | \'__/ _` |/ _ \ |    / _ \ / _` |')
print('| |____| | | | (_| | | | (_| |  __/ |___| (_) | (_| |')
print(' \_____|_| |_|\__,_|_|  \__, |\___|______\___/ \__, |')
print('                         __/ |                  __/ |')
print('                        |___/                  |___/')
print('Version %d.%d' % (version_major, version_minor))
print()
print('press ctrl+c to exit')
print()

args = parse_cmdline() #exits in case of wrong args
if args.subcmd == 'prg':
    prg(args)
elif args.subcmd == 'mon':
    mon(args)
elif args.subcmd == 'dbg':
    dbg(args)
else:
    print_error('Argument subcmd, choose from \'prg\', \'mon\', \'dbg\'.')

com_close()
log_close()
