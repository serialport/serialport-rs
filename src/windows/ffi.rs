use winapi::DWORD;

// BaudRate values
// To be removed by adding these constants to winapi, see:
// https://github.com/retep998/winapi-rs/issues/380
pub const CBR_110: DWORD = 110;
pub const CBR_300: DWORD = 300;
pub const CBR_600: DWORD = 600;
pub const CBR_1200: DWORD = 1200;
pub const CBR_2400: DWORD = 2400;
pub const CBR_4800: DWORD = 4800;
pub const CBR_9600: DWORD = 9600;
pub const CBR_14400: DWORD = 14400;
pub const CBR_19200: DWORD = 19200;
pub const CBR_38400: DWORD = 38400;
pub const CBR_57600: DWORD = 57600;
pub const CBR_115200: DWORD = 115200;
pub const CBR_128000: DWORD = 128000;
pub const CBR_256000: DWORD = 256000;

// EscapeCommFunction values
// To be removed by adding these constants to winapi, see:
// https://github.com/retep998/winapi-rs/issues/381
pub const SETRTS: DWORD = 3;
pub const CLRRTS: DWORD = 4;
pub const SETDTR: DWORD = 5;
pub const CLRDTR: DWORD = 6;

// Modem status masks
// To be removed by adding these constants to winapi, see:
// https://github.com/retep998/winapi-rs/issues/384
pub const MS_CTS_ON: DWORD = 0x0010;
pub const MS_DSR_ON: DWORD = 0x0020;
pub const MS_RING_ON: DWORD = 0x0040;
pub const MS_RLSD_ON: DWORD = 0x0080;
