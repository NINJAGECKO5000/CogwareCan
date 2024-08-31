
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Gauge {
    Name, //what it is, IE RPM, oilpres, fuellevel
    Bitlen, // how big it is, IE u8, u16
    Addr, // what address for it, such as 0 for RPM, 5 for MAP/Boost up to 255
}
impl Gauge{
    fn value(&self) -> u32{

    }
}