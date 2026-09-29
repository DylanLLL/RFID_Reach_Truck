# import serial, time
# s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)

# def send(frame, label):
#     s.reset_input_buffer()
#     s.write(bytes.fromhex(frame))
#     time.sleep(0.3)
#     print(label, s.read(256).hex(' '))

# send('BB 00 03 00 01 00 04 7E', 'hw ver:')
# send('BB 00 22 00 00 22 7E',    'poll:  ')

# s.write(bytes.fromhex('BB 00 27 00 03 22 27 10 83 7E'))  # continuous
# for _ in range(30):
#     d = s.read(64)
#     if d: print('tag:', d.hex(' '))
#     time.sleep(0.1)
# s.write(bytes.fromhex('BB 00 28 00 00 28 7E'))  # stop
# s.close()

# import serial, time

# FRAME = bytes.fromhex('BB 00 03 00 01 00 04 7E')

# for baud in (9600, 19200, 38400, 57600, 115200, 230400):
#     try:
#         s = serial.Serial('/dev/ttyUSB0', baud, timeout=0.4)
#         s.reset_input_buffer()
#         s.write(FRAME)
#         time.sleep(0.4)
#         r = s.read(256)
#         print(f'{baud:>7}: {r.hex(" ") if r else "-"}')
#         s.close()
#     except Exception as e:
#         print(f'{baud:>7}: {e}')

import serial, time

FRAME = bytes.fromhex('BB 00 03 00 01 00 04 7E')
RATES = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)

for baud in RATES:
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = '/dev/ttyUSB0', baud, 0.5
    s.dtr = s.rts = False
    s.open()
    time.sleep(0.3)
    idle = s.read(512)                      # anything unprompted?
    print(f'{baud:>7} idle: {idle.hex(" ") or "-"}')
    for i in range(3):
        s.reset_input_buffer()
        s.write(FRAME)
        time.sleep(0.3)
        r = s.read(512)
        print(f'{baud:>7}  #{i}: {r.hex(" ") or "-"}')
    s.close()