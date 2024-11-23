use linux_embedded_hal::I2cdev;
use std::time::{Duration, Instant};
use embedded_hal::blocking::i2c::{Write, WriteRead};
use rppal::gpio::{Gpio, Trigger};
use std::sync::mpsc;
use plotters::prelude::*;
use rustfft::{FftPlanner, num_complex::Complex};
use clap::Parser;
use chrono::Utc;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Enable verbose output
    #[arg(short, long)]
    verbose: bool,

    /// Number of samples to capture
    #[arg(short = 'n', long, default_value = "120")]
    number: usize,

    /// Run continuously
    #[arg(short, long)]
    continuous: bool,

    /// Output plot filename (if specified, generates plot)
    #[arg(short = 'p', long)]
    plot_filename: Option<String>,

    /// Enable FFT analysis
    #[arg(short = 'f', long)]
    fft: bool,

    /// Enable quiet output format
    #[arg(short, long)]
    quiet: bool,
}

#[derive(Debug)]
struct Reading {
    value: i16,
    timestamp_nanos: u128,
}

fn main() {
    let args = Args::parse();
    
    if args.verbose {
        println!("Verbose mode enabled");
        println!("Sample count: {}", args.number);
        println!("Plot filename: {}", args.plot_filename.as_deref().unwrap_or("none"));
    }

    loop {
        read_series(&args);
        if !args.continuous {
            break;
        }
    }
}

fn set_config(
    i2c: &mut I2cdev,
    address: u8,
    os: Option<bool>,
    mux: Option<u8>,
    pga: Option<u8>,
    mode: Option<bool>,
    data_rate: Option<u8>,
    comp_mode: Option<bool>,
    comp_pol: Option<bool>,
    comp_lat: Option<bool>,
    comp_que: Option<u8>,
    verbose: bool,
) -> bool {
    let config = 
        ((os.unwrap_or(false) as u16) << 15) |
        ((mux.unwrap_or(0) & 0x7) as u16) << 12 |
        ((pga.unwrap_or(2) & 0x7) as u16) << 9 |
        ((mode.unwrap_or(true) as u16) << 8) |
        ((data_rate.unwrap_or(4) & 0x7) as u16) << 5 |
        ((comp_mode.unwrap_or(false) as u16) << 4) |
        ((comp_pol.unwrap_or(false) as u16) << 3) |
        ((comp_lat.unwrap_or(false) as u16) << 2) |
        ((comp_que.unwrap_or(3) & 0x3) as u16);

    if verbose {
        // Print human readable configuration
        println!("Setting ADS1115 Configuration:");
        
        // Operational status
        let os_bit = (config >> 15) & 0x1;
        println!("Operational status: {}", if os_bit == 1 {"Start conversion"} else {"No effect"});
        
        // Input multiplexer
        let mux_val = (config >> 12) & 0x7;
        let mux_setting = match mux_val {
            0 => "AIN0 - AIN1 (default)",
            1 => "AIN0 - AIN3",
            2 => "AIN1 - AIN3",
            3 => "AIN2 - AIN3",
            4 => "AIN0 - GND",
            5 => "AIN1 - GND",
            6 => "AIN2 - GND",
            7 => "AIN3 - GND",
            _ => "Unknown"
        };
        println!("Input multiplexer: {}", mux_setting);
        
        // Programmable gain
        let pga_val = (config >> 9) & 0x7;
        let pga_setting = match pga_val {
            0 => "±6.144V",
            1 => "±4.096V",
            2 => "±2.048V",
            3 => "±1.024V",
            4 => "±0.512V",
            5 => "±0.256V",
            _ => "Unknown"
        };
        println!("Programmable gain: {}", pga_setting);
        
        // Mode
        let mode_bit = (config >> 8) & 0x1;
        println!("Mode: {}", if mode_bit == 0 {"Continuous"} else {"Single-shot"});
        
        // Data rate
        let dr_val = (config >> 5) & 0x7;
        let dr_setting = match dr_val {
            0 => "8 SPS",
            1 => "16 SPS",
            2 => "32 SPS",
            3 => "64 SPS",
            4 => "128 SPS",
            5 => "250 SPS",
            6 => "475 SPS",
            7 => "860 SPS",
            _ => "Unknown"
        };
        println!("Data rate: {}", dr_setting);

        // Comparator mode
        let comp_mode_bit = (config >> 4) & 0x1;
        println!("Comparator mode: {}", if comp_mode_bit == 0 {"Traditional"} else {"Window"});

        // Comparator polarity
        let comp_pol_bit = (config >> 3) & 0x1;
        println!("Comparator polarity: {}", if comp_pol_bit == 0 {"Active-low"} else {"Active-high"});

        // Comparator latch
        let comp_lat_bit = (config >> 2) & 0x1;
        println!("Comparator latch: {}", if comp_lat_bit == 0 {"Non-latching"} else {"Latching"});
        
        // Comparator queue
        let comp_que_val = (config & 0x3) as u8;
        println!("Comparator queue: {}", match comp_que_val {
            3 => String::from("Disabled"),
            n => format!("Assert after {} conversions", n + 1)
        });
    }

    // Write configuration
    let config_bytes = config.to_be_bytes();
    if i2c.write(address, &[0x01, config_bytes[0], config_bytes[1]]).is_err() {
        return false;
    }

    // Set thresholds only if comparator is enabled (comp_que != 3)
    if let Some(que) = comp_que {
        if que != 3 {
            // Set high threshold to +32767 (max positive)
            if i2c.write(address, &[0x02, 0x7F, 0xFF]).is_err() {
                return false;
            }
            // Set low threshold to -32768 (max negative)
            if i2c.write(address, &[0x03, 0x80, 0x00]).is_err() {
                return false;
            }
        }
    }

    true
}

fn turn_off(i2c: &mut I2cdev, address: u8, verbose: bool) {
    let config_word = 
        ((false as u16) << 15) |           // OS: No effect
        ((0u8 & 0x7) as u16) << 12 |      // MUX: default (AIN0/AIN1)
        ((2u8 & 0x7) as u16) << 9 |       // PGA: default (±2.048V)
        ((true as u16) << 8) |            // MODE: default (Single-shot)
        ((4u8 & 0x7) as u16) << 5 |       // DR: default (128 SPS)
        ((false as u16) << 4) |           // COMP_MODE: default (Traditional)
        ((false as u16) << 3) |           // COMP_POL: default (Active-low)
        ((false as u16) << 2) |           // COMP_LAT: default (Non-latching)
        ((3u8 & 0x3) as u16);             // COMP_QUE: default (Disabled)
    
    // Write configuration
    let msb = ((config_word >> 8) & 0xFF) as u8;
    let lsb = (config_word & 0xFF) as u8;
    if let Err(_) = i2c.write(address, &[0x01, msb, lsb]) {
        println!("Failed to write default configuration");
        return;
    }
    
    // Read back and verify
    let mut read_buf = [0u8; 2];
    if let Err(_) = i2c.write_read(address, &[0x01], &mut read_buf) {
        println!("Failed to read back configuration");
        return;
    }
    
    let read_value = ((read_buf[0] as u16) << 8) | (read_buf[1] as u16);
    
    // Mask out the OS bit (bit 15) for comparison
    let config_masked = config_word & 0x7FFF;
    let read_masked = read_value & 0x7FFF;
    
    if read_masked == config_masked {
        if verbose {
            println!("Config returned to defaults");
        }
    } else {
        println!("Failed to reset configuration");
    }
}

fn process_readings(readings: &Vec<Reading>) -> Vec<(f64, f64)> {
    readings.iter().map(|reading| {
        let voltage = reading.value as f64 * 4.096 / 32768.0;
        let processed_value = voltage * 60.0;
        
        ((reading.timestamp_nanos / 1_000_000) as f64, processed_value)
    }).collect()
}

fn calculate_timing_stats(processed_readings: &Vec<(f64, f64)>) -> (f64, f64, f64, f64) {
    let mut intervals = Vec::with_capacity(processed_readings.len() - 1);
    for i in 1..processed_readings.len() {
        let interval = processed_readings[i].0 - processed_readings[i-1].0;
        intervals.push(interval);
    }

    let min_interval = intervals.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let max_interval = intervals.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
    let avg_interval = intervals.iter().sum::<f64>() / intervals.len() as f64;
    
    let first_timestamp = processed_readings[0].0;
    let last_timestamp = processed_readings.last().unwrap().0;
    let total_time_ms = last_timestamp - first_timestamp;
    let actual_sps = if total_time_ms > 0.0 {
        (processed_readings.len() as f64 - 1.0) / (total_time_ms / 1000.0)
    } else {
        0.0
    };

    (min_interval, max_interval, avg_interval, actual_sps)
}

fn plot_readings(processed_readings: &Vec<(f64, f64)>, min_value: f64, max_value: f64, plot_filename: &str, verbose: bool) {
    let root = BitMapBackend::new(plot_filename, (1920, 1080))
        .into_drawing_area();
    root.fill(&WHITE).unwrap();

    let first_timestamp_ms = processed_readings[0].0;
    let last_timestamp_ms = processed_readings.last().unwrap().0;
    
    let unit_label = "A";
    
    let mut chart = ChartBuilder::on(&root)
        .caption("Current Measurements", ("sans-serif", 30))
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(60)
        .build_cartesian_2d(
            first_timestamp_ms..last_timestamp_ms,
            min_value..max_value,
        ).unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            processed_readings.iter().copied(),
            &BLUE,
        )).unwrap()
        .label(format!("Readings ({})", unit_label))
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw().unwrap();

    root.present().unwrap();
    if verbose {
        println!("Plot saved as {}", plot_filename);
    }
}

fn calculate_harmonics(values: &[f64]) -> (Vec<f64>, usize, usize) {
    // Find first zero crossing and the closest point to zero
    let mut start_idx = 0;
    for i in 1..values.len() {
        if values[i-1] < 0.0 && values[i] >= 0.0 {
            // Compare distances to zero
            let dist_prev = values[i-1].abs();
            let dist_curr = values[i].abs();
            start_idx = if dist_prev < dist_curr { i-1 } else { i };
            break;
        }
    }
    
    // Find last zero crossing and the closest point to zero
    let mut end_idx = values.len() - 1;
    for i in (1..values.len()).rev() {
        if values[i-1] < 0.0 && values[i] >= 0.0 {
            // Compare distances to zero
            let dist_prev = values[i-1].abs();
            let dist_curr = values[i].abs();
            end_idx = if dist_prev < dist_curr { i-1 } else { i };
            break;
        }
    }
    
    // Use trimmed slice for analysis
    let trimmed_values = &values[start_idx..=end_idx];
    
    // Apply Hamming window
    let windowed: Vec<Complex<f64>> = trimmed_values.iter()
        .enumerate()
        .map(|(i, &x)| {
            let window = 0.54 - 0.46 * (2.0 * std::f64::consts::PI * i as f64 / (trimmed_values.len() - 1) as f64).cos();
            Complex::new(x * window, 0.0)
        })
        .collect();

    // Perform FFT
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(windowed.len());
    let mut spectrum = windowed;
    fft.process(&mut spectrum);

    // Calculate magnitude spectrum
    let magnitudes: Vec<f64> = spectrum.iter()
        .take(spectrum.len() / 2)  // Only take first half (Nyquist)
        .map(|c| (c.norm() / (values.len() as f64).sqrt()))
        .collect();

    // Find fundamental frequency (largest magnitude after DC)
    let fundamental_idx = (1..magnitudes.len())
        .max_by(|&i, &j| magnitudes[i].partial_cmp(&magnitudes[j]).unwrap())
        .unwrap();
    let fundamental_magnitude = magnitudes[fundamental_idx];

    // Calculate harmonic percentages (up to 9th harmonic instead of 15th)
    let mut harmonics = Vec::new();
    for i in 1..=7 {
        let harmonic_idx = fundamental_idx * i;
        if harmonic_idx < magnitudes.len() {
            let percentage = (magnitudes[harmonic_idx] / fundamental_magnitude) * 100.0;
            harmonics.push(percentage);
        } else {
            harmonics.push(0.0);
        }
    }

    (harmonics, start_idx, end_idx)
}

fn print_harmonics(harmonics: &[f64]) {
    print!("Harmonic: 1");
    for i in 1..harmonics.len() {
        print!(" {}", i + 1);
    }
    println!();
    
    print!("Percent:  100");
    for percentage in harmonics.iter().skip(1) {
        if percentage.fract() > 0.05 {
            print!(" {:.1}", percentage);
        } else {
            print!(" {}", percentage.round());
        }
    }
    println!();
}

fn report_statistics(readings: &Vec<Reading>, args: &Args) {
    if readings.is_empty() {
        println!("No readings collected!");
        return;
    }

    let now = Utc::now();
    if args.quiet {
        print!("{:.3}", now.timestamp() as f64 + now.timestamp_subsec_nanos() as f64 / 1_000_000_000.0);
    } else {
        println!("Timestamp: {:.3} {}", 
            now.timestamp() as f64 + now.timestamp_subsec_nanos() as f64 / 1_000_000_000.0,
            now.to_rfc3339_opts(chrono::SecondsFormat::Millis, true)
        );
    }

    let processed_readings = process_readings(readings);
    let values: Vec<f64> = processed_readings.iter().map(|(_t, v)| *v).collect();
    
    // Calculate harmonics and get trimmed indices
    let (harmonics, start_idx, end_idx) = calculate_harmonics(&values);
    
    // Create trimmed series for plotting
    let trimmed_readings: Vec<(f64, f64)> = processed_readings[start_idx..=end_idx].to_vec();
    
    // Calculate statistics on trimmed data
    let trimmed_values: Vec<f64> = trimmed_readings.iter().map(|(_t, v)| *v).collect();
    let min_value = trimmed_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let max_value = trimmed_values.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
    let avg_value = trimmed_values.iter().sum::<f64>() / trimmed_values.len() as f64;
    
    // Print statistics
    if args.verbose && !args.quiet {
        println!("Average current: {:.2}A", avg_value);
    }
    let rms = (trimmed_values.iter()
        .map(|v| v.powi(2))
        .sum::<f64>() / trimmed_values.len() as f64)
        .sqrt();
    if args.quiet {
        print!(",{:.2}", rms);
    } else {
        println!("RMS Current: {:.2}A", rms);
    }
    
    // Print harmonics
    if args.fft {
        if args.quiet {
            // Skip the first harmonic (100%) and print the rest comma-separated
            print!(",{}", harmonics.iter().skip(1)
                .map(|h| format!("{:.1}", h))
                .collect::<Vec<String>>()
                .join(","));
        } else {
            print_harmonics(&harmonics);
        }
    }
    
    if args.quiet {
        println!();
    }

    if args.verbose && !args.quiet {
        // Calculate and print timing statistics
        let (min_interval, max_interval, avg_interval, actual_sps) = 
            calculate_timing_stats(&trimmed_readings);
        println!("Average interval: {:.2} microseconds", avg_interval);
        println!("Min interval: {:.2} microseconds", min_interval);
        println!("Max interval: {:.2} microseconds", max_interval);
        println!("Actual samples per second: {:.1} SPS", actual_sps);
        println!("Value range: {:.3}A to {:.3}A", min_value, max_value);
    }

    // Create the plot if plotting is enabled
    if let Some(plot_filename) = &args.plot_filename {
        plot_readings(&trimmed_readings, min_value, max_value, plot_filename, args.verbose && !args.quiet);
    }
}

fn read_series(args: &Args) {
    let mut i2c = I2cdev::new("/dev/i2c-1").expect("Failed to open I2C device");
    let address = 0x48u8;
    
    if args.verbose {
        println!("Setting up GPIO and interrupt handler...");
    }
    
    // Setup GPIO4 with pull-down and interrupt
    let gpio = Gpio::new().expect("Failed to initialize GPIO");
    let mut gclk_pin = gpio.get(4).expect("Failed to get GPIO pin")
                          .into_input_pulldown();
    
    // Create a channel for sending interrupt events
    let (tx, rx) = mpsc::channel::<()>();
    
    // Clone tx for the interrupt handler
    let tx_interrupt = tx.clone();
    
    // Configure interrupt for falling edge (when GCLK goes low)
    gclk_pin.set_async_interrupt(Trigger::FallingEdge, move |_level| {
        let _ = tx_interrupt.send(());
    }).expect("Failed to set interrupt");

    if args.verbose {
        println!("Interrupt handler configured. Setting up ADS1115...");
    }
    
    if !set_config(
        &mut i2c,
        address,
        Some(true),     // OS: Start conversion
        Some(0),        // MUX: AIN0 vs AIN1 (differential)
        Some(1),        // PGA: ±4.096V
        Some(false),    // MODE: Continuous
        Some(7),        // DR: 860 SPS
        None,           // COMP_MODE: default (Traditional)
        None,           // COMP_POL: default (Active-low)
        Some(true),     // COMP_LAT: Latching
        Some(0),        // COMP_QUE: Assert after 1 conversion
        args.verbose,   // Add verbose parameter
    ) {
        println!("Failed to set configuration, aborting.");
        return;
    }

    if args.verbose {
        println!("Starting continuous readings from ADS1115 using GCLK interrupts...");
        println!("Initial GCLK state: {:?}", gclk_pin.read());
    }
    
    let mut readings = Vec::with_capacity(args.number);
    let start_time = Instant::now();

    loop {
        match rx.recv_timeout(Duration::from_secs(1)) {
            Ok(_) => {
                // Read raw values directly
                let mut read_buf = [0u8; 2];
                if let Ok(_) = i2c.write_read(address, &[0x00], &mut read_buf) {
                    let raw_value = i16::from_be_bytes([read_buf[0], read_buf[1]]);
                    let timestamp_nanos = start_time.elapsed().as_nanos();
                    readings.push(Reading { 
                        value: raw_value,
                        timestamp_nanos 
                    });

                    // Check if we've reached the sample count
                    if readings.len() >= args.number {
                        report_statistics(&readings, &args);
                        break;
                    }
                }
            },
            Err(mpsc::RecvTimeoutError::Timeout) => {
                println!("Timeout waiting for GCLK interrupt");
                break;
            },
            Err(e) => {
                println!("Error receiving interrupt: {:?}", e);
                break;
            }
        }
    }

    turn_off(&mut i2c, address, args.verbose);
} 