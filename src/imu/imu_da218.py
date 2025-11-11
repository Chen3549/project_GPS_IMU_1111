import smbus2
import time

class i2c_device:
    def __init__(self, addr, port=1):
        self.addr = addr
        self.bus = smbus2.SMBus(port)

    # Write a single command
    def write_cmd(self, cmd):
        self.bus.write_byte(self.addr, cmd)
        sleep(0.0001)

    # Write a command and argument
    def write_cmd_arg(self, cmd, data):
        self.bus.write_byte_data(self.addr, cmd, data)
        sleep(0.0001)

    # Write a block of data
    def write_block_data(self, cmd, data):
        self.bus.write_block_data(self.addr, cmd, data)
        sleep(0.0001)

    # Read a single byte
    def read(self):
        return self.bus.read_byte(self.addr)

    # Read
    def read_data(self, cmd):
        return self.bus.read_byte_data(self.addr, cmd)

    # Read a block of data
    def read_block_data(self, cmd):
        return self.bus.read_block_data(self.addr, cmd)

DA218_ADDR = 0x27
ACC_X_LSB_ADDR = 0x02  
ACC_X_MSB_ADDR = 0x03  

da218 = i2c_device(DA218_ADDR)
while (True):
    acc_x_lsb = da218.read_data(ACC_X_LSB_ADDR)
    acc_x_msb = da218.read_data(ACC_X_MSB_ADDR)
    raw_data = (acc_x_msb << 4) | (acc_x_lsb >> 4)
    
    # 处理12位补码：若第11位（最高位）为1，说明是负数，需扩展符号位到16位后再转有符号数
    if raw_data & 0x800:  # 检查第11位是否为1
        raw_data = raw_data - 0x1000  # 12位补码转有符号数（0x1000是2^12）
    print(raw_data)
    time.sleep(0.1)
