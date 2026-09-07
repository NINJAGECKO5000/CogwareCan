//! CAN-broadcast ECUs in: the same gauges, whatever is attached.
//!
//!     cargo run --example ecu_can

use cogware_can::*;

fn be(id: u16, words: &[i16]) -> mcp2515::frame::CanFrame {
    let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_be_bytes()).collect();
    protocol::frame(id, &bytes).unwrap()
}

fn le(id: u16, words: &[i16]) -> mcp2515::frame::CanFrame {
    let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
    protocol::frame(id, &bytes).unwrap()
}

fn show(what: &str) {
    println!(
        "  {what:<12} rpm {:>5.0}  MAP {:>6.1} kPa  CLT {:>5.1} °C  AFR {:>5.2}",
        RPM.as_f32().unwrap_or(0.0),
        MAP.as_f32().unwrap_or(0.0),
        CLNT.as_f32().unwrap_or(0.0),
        AFR_PRI.as_f32().unwrap_or(0.0),
    );
}

fn main() {
    // Four ECUs, four wire formats, one gauge table. Each sends 3500 rpm,
    // 95.0 kPa, 87.0 °C and 13.20 AFR in its own encoding.
    println!("each ECU's own frames, decoded into the same gauges\n");

    clear_all();
    haltech::SOURCE.feed_default(&be(0x360, &[3500, 950, 500, 2213]));
    haltech::SOURCE.feed_default(&be(0x3E0, &[3601, 2981, 0, 0])); // Kelvin*10
    haltech::SOURCE.feed_default(&be(0x368, &[898, 0])); // lambda*1000
    show("Haltech");

    clear_all();
    megasquirt::SOURCE.feed_default(&be(1512, &[950, 3500, 1886, 500])); // °F*10
    megasquirt::SOURCE.feed_default(&protocol::frame(1514, &[147, 132, 0, 0, 0, 0]).unwrap());
    show("Megasquirt");

    clear_all();
    let mut d = 2850u16.to_le_bytes().to_vec(); // kPa*30
    d.extend([127, 65, 0, 0, 0, 0]);
    rusefi::SOURCE.feed_default(&d_frame(0x203, &d));
    rusefi::SOURCE.feed_default(&le(0x201, &[3500, 0, 0, 0]));
    rusefi::SOURCE.feed_default(&le(0x207, &[8980u16 as i16, 0, 0, 0])); // lambda*10000
    show("rusEFI");

    clear_all();
    maxxecu::SOURCE.feed_default(&le(0x520, &[3500, 0, 950, 0]));
    maxxecu::SOURCE.feed_default(&le(0x521, &[898, 0, 0, 0]));
    maxxecu::SOURCE.feed_default(&le(0x530, &[0, 0, 0, 870]));
    show("MaxxECU");

    // A server that does not know which ECU is wired in can try each source.
    // Only the one that owns the ID decodes the frame.
    println!("\nauto-detecting the source from the frame ID");
    clear_all();
    for f in [be(0x360, &[3500, 0, 0, 0]), be(1512, &[0, 4000, 0, 0]), le(0x520, &[4500, 0, 0, 0])] {
        let hit = CAN_SOURCES.iter().find_map(|s| s.feed_default(&f).map(|n| (s.name, n)));
        match hit {
            Some((name, n)) => println!("  {:#05x} -> {name}, {n} gauges, rpm now {}", protocol::id_of(&f).unwrap(), RPM.get_or(0)),
            None => println!("  {:#05x} -> no source claims this ID", protocol::id_of(&f).unwrap()),
        }
    }
    let stray = protocol::frame(0x123, &[0; 8]).unwrap();
    println!("  0x123 -> {:?}", CAN_SOURCES.iter().find_map(|s| s.feed_default(&stray)));

    println!("\nID ranges each source listens on (at its default base)");
    for s in CAN_SOURCES {
        let ids: Vec<String> = s.ids(s.default_base).map(|i| format!("{i:#05x}")).collect();
        println!("  {:<11} {}", s.name, ids.join(" "));
    }
}

fn d_frame(id: u16, data: &[u8]) -> mcp2515::frame::CanFrame {
    protocol::frame(id, data).unwrap()
}
