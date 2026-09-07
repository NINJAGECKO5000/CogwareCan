//! OBD2 in: the one source that is request/response rather than broadcast.
//!
//!     cargo run --example obd2_poll

use cogware_can::obd2;
use cogware_can::*;
use mcp2515::frame::CanFrame;

/// Stand-in for a car's ECU answering a mode 01 request.
fn car_answers(request: &CanFrame) -> Option<CanFrame> {
    use embedded_hal_0_2::can::Frame;
    let pid = request.data()[2];
    let payload: &[u8] = match pid {
        0x0C => &[0x36, 0xB0], // (256*54 + 176)/4 = 3500 rpm
        0x0B => &[95],         // 95 kPa
        0x05 => &[127],        // 127 - 40 = 87 °C
        0x0F => &[65],         // 25 °C
        0x11 => &[128],        // 128*100/255 = 50.2 %
        0x0D => &[88],         // 88 km/h
        0x42 => &[0x35, 0xE8], // 13800 mV
        0x06 => &[144],        // 144/1.28 - 100 = +12.5 % trim
        0x01 => &[0x83, 0, 0, 0], // lamp on, 3 stored codes
        _ => return None,      // this car does not support the PID
    };
    let mut d = vec![2 + payload.len() as u8, 0x41, pid];
    d.extend_from_slice(payload);
    d.resize(8, 0x55); // real ECUs pad the unused bytes
    protocol::frame(0x7E8, &d)
}

fn main() {
    clear_all();

    // A poll loop walks PIDS, sends each request, and decodes what comes back.
    // Unsupported PIDs simply never answer, so those gauges stay unset.
    let mut answered = 0;
    let mut unsupported = Vec::new();
    for p in obd2::PIDS {
        let req = obd2::request(p.pid).unwrap();
        match car_answers(&req) {
            Some(resp) => {
                obd2::feed_response(&resp);
                answered += 1;
            }
            None => unsupported.push(format!("{:#04x}", p.pid)),
        }
    }
    println!("polled {} PIDs, {answered} answered", obd2::PIDS.len());
    println!("this car does not support: {}\n", unsupported.join(" "));

    println!("  rpm       {:>7.0}", RPM.as_f32().unwrap());
    println!("  MAP       {:>7.1} kPa", MAP.as_f32().unwrap());
    println!("  coolant   {:>7.1} °C", CLNT.as_f32().unwrap());
    println!("  intake    {:>7.1} °C", IAT.as_f32().unwrap());
    println!("  throttle  {:>7.1} %", TPS.as_f32().unwrap());
    println!("  speed     {:>7.1} km/h  ({:.1} mph)", VSS.as_f32().unwrap(), VSS.to(Unit::MPH).unwrap());
    println!("  battery   {:>7.2} V", BAT_VOL.as_f32().unwrap());
    println!("  fuel trim {:>+7.1} %", EGO_CORRECT.as_f32().unwrap());
    println!("  DTC count {:>7}", ERROR_COUNT.get().unwrap());

    // Oil pressure has no standard PID, so a stock car can never supply it.
    println!("\n  oil press {:?}  (no mode 01 PID exists)", OIL_PRES.get());

    // Junk on the bus is rejected, not decoded.
    println!("\nrejected frames");
    println!("  a gauge frame:    {:?}", obd2::feed_response(&protocol::frame(0x2D, &[0, 1]).unwrap()));
    println!("  negative reply:   {:?}", obd2::feed_response(&protocol::frame(0x7E8, &[3, 0x7F, 0x01, 0x12]).unwrap()));
}
