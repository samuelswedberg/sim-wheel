from pyftdi.spi import SpiController

# Initialize the SPI controller
spi = SpiController()

# Open the first available FTDI device with the first SPI port
spi.configure('ftdi://ftdi:232h/1')

# Get an SPI port to communicate with the device
slave = spi.get_port(cs=0)  # Chip select at port 0

# Write some data
slave.write([0x01, 0x02, 0x03])

# Read some data
data = slave.exchange([0x00] * 3, 3)  # Send three dummy bytes and read three bytes
print(data)
