use serialport::SerialPort;
use std::time::{Duration, Instant};

fn main() {
    // sudo apt-get install -y libudev-dev
    let mut port = serialport::new("/dev/ttyAMA0", 115_200)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");
    initialize_lidar(&mut port);
    let mut scan_packets = Vec::new();
    let mut scans = Vec::new();

    loop {
        while port.bytes_to_read().unwrap() >= 132 {
            let scan: Vec<u8> = read(&mut port, 132).unwrap();
            scan_packets.push(deserialize_express_scan_extended(
                scan.as_slice().try_into().unwrap(),
            ));
            if scan_packets.len() > 1 {
                let previous_scan = &scan_packets[scan_packets.len() - 2];
                let current_scan = &scan_packets[scan_packets.len() - 1];
                for i in 0..32 {
                    let mut dist_q2 = [0; 3];
                    // let mut quality = [0; 3];
                    // let mut sync_bit = [false; 3];

                    let combined_x3 = previous_scan.ultra_cabins[i];
                    // unpack
                    let dist_major1 = combined_x3 & 0xFFF;
                    let mut dist_predict1 = ((combined_x3 as i32) << 10) >> 22;
                    let mut dist_predict2 = (combined_x3 as i32) >> 22;

                    let dist_major2 = if i == 31 {
                        current_scan.ultra_cabins[0] & 0xFFF
                    } else {
                        previous_scan.ultra_cabins[i + 1] & 0xFFF
                    };

                    let mut scale_level1 = 0;
                    let mut scale_level2 = 0;

                    let dist_major1 = varbitscale_decode(dist_major1, &mut scale_level1);
                    let dist_major2 = varbitscale_decode(dist_major2, &mut scale_level2);

                    let mut dist_base1 = dist_major1;
                    let dist_base2 = dist_major2;

                    if dist_major1 == 0 && dist_major2 != 0 {
                        dist_base1 = dist_major2;
                        scale_level1 = scale_level2;
                    }

                    dist_q2[0] = dist_major1 << 2;
                    if dist_predict1 as u32 == 0xFFFFFE00 || dist_predict1 == 0x1FF {
                        dist_q2[1] = 0
                    } else {
                        dist_predict1 <<= scale_level1;
                        dist_q2[1] = ((dist_base1 as i32 + dist_predict1) << 2) as u32;
                    }

                    if dist_predict2 as u32 == 0xFFFFFE00 || dist_predict2 == 0x1FF {
                        dist_q2[2] = 0
                    } else {
                        dist_predict2 <<= scale_level2;
                        dist_q2[2] = ((dist_base2 as i32 + dist_predict2) << 2) as u32;
                    }

                    let start_angle_q6 = previous_scan.start_angle_q6 as i32;
                    let angle_diff_q6 = (current_scan.start_angle_q6 as i32
                        - start_angle_q6 as i32)
                        .rem_euclid(360 * 64);

                    if scans.len() == 0 {
                        scans.push(Vec::new());
                    }
                    for j in 0..3 {
                        let point = LidarPoint {
                            angle_q6: start_angle_q6 as u16
                                + (angle_diff_q6 as f64 * (i as f64 / 32.0 + j as f64 / 96.0))
                                    as u16,
                            distance_q2: dist_q2[j],
                        };

                        scans[0].push(point);

                        // // a scan is defined by 360 degrees
                        // let scan = scans.last_mut().unwrap();
                        // let first_point = scan.first();
                        // if let Some(first_point) = first_point {
                        //     let angle_diff =
                        //         (point.angle_q6 as i32 - first_point.angle_q6 as i32).abs();
                        //     if angle_diff > 360 * 64 / 2 {
                        //         scans.push(Vec::new());
                        //     }
                        // } else {
                        //     scan.push(point);
                        // }
                    }
                }
            }
        }
        println!("scan_packets: {}", scan_packets.len());
    }
}

fn initialize_lidar(port: &mut Box<dyn SerialPort>) {
    println!("Initializing lidar");
    write(port, [0xa5, 0x25]);
    // wait
    std::thread::sleep(Duration::from_millis(800));
    // clear buffer
    port.clear(serialport::ClearBuffer::Input).unwrap();
    // wait
    for _ in 0..10 {
        std::thread::sleep(Duration::from_millis(500));
        write(port, [0xa5, 0x52]);
        std::thread::sleep(Duration::from_millis(500));
        if let Ok(response) = read(port, 10) {
            assert_eq!(
                response,
                [0xa5, 0x5a, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00]
            );
            break;
        } else {
            println!("No response, retrying in 1 second");
        }
    }
    write(port, [0xa5, 0x50]);
    // a5 5a 14 00 00 00 04 18 1d 01 07 d9 8a 99 f6 c9 e5 9a d2 c5 e5 9c f7 25 58 34 12
    assert_eq!(
        read(port, 27).unwrap(),
        [
            0xa5, 0x5a, 0x14, 0x00, 0x00, 0x00, 0x04, 0x18, 0x1d, 0x01, 0x07, 0xd9, 0x8a, 0x99,
            0xf6, 0xc9, 0xe5, 0x9a, 0xd2, 0xc5, 0xe5, 0x9c, 0xf7, 0x25, 0x58, 0x34, 0x12
        ]
    );
    println!("made it to here");
    // 	a5 84 06 71 00 00 00 03 00 55
    write(
        port,
        [0xa5, 0x84, 0x06, 0x71, 0x00, 0x00, 0x00, 0x03, 0x00, 0x55],
    );
    // a5 5a 08 00 00 00 20 71 00 00 00 00 7f 00 00
    assert_eq!(
        read(port, 15).unwrap(),
        [
            0xa5, 0x5a, 0x08, 0x00, 0x00, 0x00, 0x20, 0x71, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00,
            0x00
        ]
    );
    // a5 84 06 74 00 00 00 03 00 50
    write(
        port,
        [0xa5, 0x84, 0x06, 0x74, 0x00, 0x00, 0x00, 0x03, 0x00, 0x50],
    );
    // a5 5a 08 00 00 00 20 74 00 00 00 00 0c 00 00
    assert_eq!(
        read(port, 15).unwrap(),
        [
            0xa5, 0x5a, 0x08, 0x00, 0x00, 0x00, 0x20, 0x74, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00,
            0x00
        ]
    );
    println!("made it to there");
    // 	a5 84 06 75 00 00 00 03 00 51
    write(
        port,
        [0xa5, 0x84, 0x06, 0x75, 0x00, 0x00, 0x00, 0x03, 0x00, 0x51],
    );
    // a5 5a 05 00 00 00 20 75 00 00 00 84
    assert_eq!(
        read(port, 12).unwrap(),
        [0xa5, 0x5a, 0x05, 0x00, 0x00, 0x00, 0x20, 0x75, 0x00, 0x00, 0x00, 0x84]
    );
    // 	a5 84 06 7f 00 00 00 03 00 5b
    write(
        port,
        [0xa5, 0x84, 0x06, 0x7f, 0x00, 0x00, 0x00, 0x03, 0x00, 0x5b],
    );
    // a5 5a 10 00 00 00 20 7f 00 00 00 53 65 6e 73 69 74 69 76 69 74 79 00
    assert_eq!(
        read(port, 23).unwrap(),
        [
            0xa5, 0x5a, 0x10, 0x00, 0x00, 0x00, 0x20, 0x7f, 0x00, 0x00, 0x00, 0x53, 0x65, 0x6e,
            0x73, 0x69, 0x74, 0x69, 0x76, 0x69, 0x74, 0x79, 0x00
        ]
    );
    // a5 79
    write(port, [0xa5, 0x79]);
    // a5 5a 0f 00 00 00 14 00 00 61 00 00 00 a0 00 00 0c 00 04 00 28 1d
    assert_eq!(
        read(port, 22).unwrap(),
        [
            0xa5, 0x5a, 0x0f, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x61, 0x00, 0x00, 0x00, 0xa0,
            0x00, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x28, 0x1d
        ]
    );
    // Extended version, scan mode 3
    // a5 82 05 03 00 00 00 00 21
    write(port, [0xa5, 0x82, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x21]);
    // a5 5a 84 00 00 40 84
    assert_eq!(
        read(port, 7).unwrap(),
        [0xa5, 0x5a, 0x84, 0x00, 0x00, 0x40, 0x84]
    );
    deserialize_response_descriptor(&[0xa5, 0x5a, 0x84, 0x00, 0x00, 0x40, 0x84]);
    println!("Initialized lidar");
}

fn write(port: &mut Box<dyn SerialPort>, data: impl AsRef<[u8]>) {
    port.write_all(data.as_ref()).expect("Failed to write");
}

fn read(port: &mut Box<dyn SerialPort>, size: usize) -> Result<Vec<u8>, std::io::Error> {
    let mut buf: Vec<u8> = vec![0; size];
    port.read_exact(&mut buf)?;
    Ok(buf)
}

fn deserialize_response_descriptor(bytes: &[u8; 7]) {
    assert_eq!(bytes[0], 0xa5);
    assert_eq!(bytes[1], 0x5a);
    let packet_length = u32::from_le_bytes(bytes[2..6].try_into().unwrap()) as usize & 0x3fff;
    println!("packet_length: {}", packet_length);
    let send_mode = u8::from_le_bytes([bytes[5]]) & 0x03;
    match send_mode {
        0 => println!("send_mode: single"),
        1 => println!("send_mode: multi"),
        _ => panic!("invalid send_mode"),
    }
    let data_type = bytes[6];
    println!("data_type: {}", data_type);
}

struct ScanPacket {
    timestamp: Instant,
    start_bit: bool,
    start_angle_q6: u16,
    ultra_cabins: [u32; 32],
}

fn deserialize_express_scan_extended(bytes: &[u8; 132]) -> ScanPacket {
    let timestamp = Instant::now();
    let sync = (bytes[0] & 0xF0) | (bytes[1] >> 4);
    assert_eq!(sync, 0xa5);
    let checksum = (bytes[0] & 0xF) | (bytes[1] << 4);
    let mut check_checksum = 0;
    for i in 2..bytes.len() {
        check_checksum ^= bytes[i];
    }
    assert_eq!(checksum, check_checksum);
    let start_bit = bytes[3] & 0b1000_0000 != 0;
    let start_angle_q6 = u16::from_le_bytes([bytes[2], bytes[3] & 0b0111_1111]);

    if start_bit {
        println!("New Scan Started");
    }

    let mut ultra_cabins = Vec::with_capacity(32);
    for i in 0..32 {
        let offset = i * 4;
        let cabin = u32::from_le_bytes(bytes[(4 + offset)..(8 + offset)].try_into().unwrap());
        ultra_cabins.push(cabin);
    }
    ScanPacket {
        timestamp,
        start_bit,
        start_angle_q6,
        ultra_cabins: ultra_cabins.try_into().unwrap(),
    }
}

#[derive(Debug, Copy, Clone)]
struct LidarPoint {
    angle_q6: u16,
    distance_q2: u32,
}

fn varbitscale_decode(scaled: u32, scale_level: &mut u32) -> u32 {
    const VBS_SCALED_BASE: [u32; 5] = [3328, 1792, 1280, 512, 0];
    const VBS_SCALED_LVL: [u32; 5] = [4, 3, 2, 1, 0];
    const VBS_TARGET_BASE: [u32; 5] = [(0x1 << 14), (0x1 << 12), (0x1 << 11), (0x1 << 9), 0];

    for i in 0..VBS_SCALED_BASE.len() {
        let remain = scaled as i32 - VBS_SCALED_BASE[i] as i32;
        if remain >= 0 {
            *scale_level = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << *scale_level) as u32;
        }
    }
    0
}

#[test]
fn test_varbitscale_decode() {
    let mut scale_level = 0;
    assert_eq!(varbitscale_decode(1000, &mut scale_level), 1488);
    assert_eq!(scale_level, 1);
    scale_level = 0;
    assert_eq!(varbitscale_decode(2000, &mut scale_level), 5760);
    assert_eq!(scale_level, 3);
    scale_level = 0;
    assert_eq!(varbitscale_decode(1500, &mut scale_level), 2928);
    assert_eq!(scale_level, 2);
    scale_level = 0;
    assert_eq!(varbitscale_decode(15000, &mut scale_level), 203136);
    assert_eq!(scale_level, 4);
    scale_level = 0;
    assert_eq!(varbitscale_decode(0, &mut scale_level), 0);
    assert_eq!(scale_level, 0);
}
