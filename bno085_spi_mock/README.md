# Mock for the BNO085 readout library

This is for easily unit testing code using the library.

## Update mocks

Basic mock generation is nothing more than running cmock command on a header file:

```bash
# Ensure you are in the bno085_spi_mock directory. Then run:
ruby $PATH_TO_CMOCK_REPO/lib/cmock.rb -ocmock_config.yml ../lib/bno085_spi/include/BNO085_SPI_Library.h
# Move the artifacts from the mocks/ directory:
mv mocks/* .
```

Then copy the generated files from the generated directory to this directory.
