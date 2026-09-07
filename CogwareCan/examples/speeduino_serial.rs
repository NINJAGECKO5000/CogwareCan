//! Serial ECU in: a Speeduino realtime packet becomes gauge values.
//!
//!     cargo run --example speeduino_serial

use cogware_can::speeduino::{self, Layout};
use cogware_can::*;

/// Stand-in for the ECU. Real firmware writes "n" to the UART and reads back
/// a 3-byte header followed by the payload the header declares.
fn ecu_reply() -> Vec<u8> {
    let mut body = vec![0u8; Layout::LATEST.payload_len()];
    body[0] = 42; // secl, the ECU's second counter
    body[4..6].copy_from_slice(&95u16.to_le_bytes()); // MAP 95 kPa
    body[6] = 25 + 40; // IAT 25 °C, sent with a +40 offset
    body[7] = 87 + 40; // coolant 87 °C
    body[9] = 138; // battery 13.8 V, sent in tenths
    body[10] = 132; // AFR 13.2, sent in tenths
    body[14..16].copy_from_slice(&3500u16.to_le_bytes()); // rpm
    body[24] = (-3i8) as u8; // advance -3°
    body[25] = 100; // TPS, 0.5 % per count
    body[76..78].copy_from_slice(&2750u16.to_le_bytes()); // pulse width 2.75 ms
    body[104..106].copy_from_slice(&88u16.to_le_bytes()); // 88 km/h

    let mut packet = vec![b'n', 0x32, body.len() as u8];
    packet.extend_from_slice(&body);
    packet
}

fn main() {
    clear_all();
    let packet = ecu_reply();

    // Read the header first, then exactly the bytes it declares. Reading a
    // fixed count instead leaves the UART FIFO out of step with the ECU.
    let declared = speeduino::n_payload_len(&packet[..speeduino::N_HEADER_LEN]).unwrap();
    println!("header declares {declared} payload bytes");

    let written = speeduino::parse_n(&packet).unwrap();
    println!("parsed, {written} gauges written\n");

    println!("  rpm      {:>7.0}", RPM.as_f32().unwrap());
    println!("  MAP      {:>7.1} kPa   ({:+.1} psi gauge)", MAP.as_f32().unwrap(), MAP.to(Unit::PSI).unwrap() - 14.7);
    println!("  coolant  {:>7.1} °C    ({:.0} °F)", CLNT.as_f32().unwrap(), CLNT.to(Unit::FAHRENHEIT).unwrap());
    println!("  intake   {:>7.1} °C", IAT.as_f32().unwrap());
    println!("  AFR      {:>7.2}", AFR_PRI.as_f32().unwrap());
    println!("  battery  {:>7.2} V", BAT_VOL.as_f32().unwrap());
    println!("  TPS      {:>7.1} %", TPS.as_f32().unwrap());
    println!("  advance  {:>7.1} °", CUR_SPARK_ADVANCE.as_f32().unwrap());
    println!("  pulse    {:>7.2} ms", PULSE_WIDTH1.as_f32().unwrap());
    println!("  speed    {:>7.1} km/h", VSS.as_f32().unwrap());

    // A short reply from older firmware fills what it carries and no more.
    clear_all();
    let short = &packet[..speeduino::N_HEADER_LEN + 30];
    let mut header = short.to_vec();
    header[2] = 30;
    println!("\n30-byte payload: {} gauges", speeduino::parse_n(&header).unwrap());
    println!("  rpm     {:?}", RPM.get());
    println!("  speed   {:?}  (past the end of a short packet)", VSS.get());

    // Malformed input is an error, never a panic.
    println!("\nbad packets: {:?}", speeduino::parse_n(b"xx"));
    println!("             {:?}", speeduino::parse_n(&[b'n', 0x32, 200, 0, 0]));
}
