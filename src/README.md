# i2c_current_monitor

This is a Rust program that reads an ADC (ADS1115) over I2C and reports the current in amps.

## Usage

```
i2c_current_monitor -i <i2c_bus> -a <address> -r <CT ratio> -v -f -s -p -c
```

Default values:
- I2C bus: /dev/i2c-1
- Address: 0x48
- CT ratio: 60

Options:
- -v: Verbose output displays the configuration of the ADC.
- -f: FFT output displays the FFT of the current waveform.
- -s: Statistics output displays the min, max, average and RMS current.
- -p: Plotting creates a plot of the current waveform to the `statistics.png` file in the current working directory.
- -c: Continuous operation reruns sampling and analysis continuously.

