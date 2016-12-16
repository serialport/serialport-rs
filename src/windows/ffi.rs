#![allow(non_snake_case,non_camel_case_types,non_upper_case_globals,dead_code)]

extern crate libc;

use std::mem;

use self::libc::{c_void,c_char,c_int,c_ulong,wchar_t};

pub type BYTE = u8;
pub type WORD = u16;
pub type DWORD = c_ulong;
pub type BOOL = c_int;
pub type WCHAR = wchar_t;

pub type LPDWORD = *mut DWORD;
pub type LPVOID = *mut c_void;

pub type LPCWSTR = *const WCHAR;
pub type LPWSTR = *mut WCHAR;

pub type HANDLE = *mut LPVOID;

pub const GENERIC_READ: DWORD = 0x80000000;
pub const GENERIC_WRITE: DWORD = 0x40000000;
pub const OPEN_EXISTING: DWORD = 3;
pub const FILE_ATTRIBUTE_NORMAL: DWORD = 0x80;
pub const INVALID_HANDLE_VALUE: HANDLE = !0 as HANDLE;

#[repr(C)]
pub struct SECURITY_ATTRIBUTES {
    pub nLength: DWORD,
    pub lpSecurityDescriptor: LPVOID,
    pub bInheritHandle: BOOL,
}

pub type LPSECURITY_ATTRIBUTES = *mut SECURITY_ATTRIBUTES;

#[repr(C)]
pub struct OVERLAPPED {
    pub Internal: *mut c_ulong,
    pub InternalHigh: *mut c_ulong,
    pub Offset: DWORD,
    pub OffsetHigh: DWORD,
    pub hEvent: HANDLE,
}

pub type LPOVERLAPPED = *mut OVERLAPPED;

#[derive(Copy,Clone,Debug)]
#[repr(C)]
pub struct DCB {
    pub DCBlength:  DWORD,
    pub BaudRate:   DWORD,
    pub fBits:      DWORD,
    pub wReserved:  WORD,
    pub XonLim:     WORD,
    pub XoffLim:    WORD,
    pub ByteSize:   BYTE,
    pub Parity:     BYTE,
    pub StopBits:   BYTE,
    pub XonChar:    c_char,
    pub XoffChar:   c_char,
    pub ErrorChar:  c_char,
    pub EofChar:    c_char,
    pub EvtChar:    c_char,
    pub wReserved1: WORD
}

// BaudRate values
pub const CBR_110:    DWORD = 110;
pub const CBR_300:    DWORD = 300;
pub const CBR_600:    DWORD = 600;
pub const CBR_1200:   DWORD = 1200;
pub const CBR_2400:   DWORD = 2400;
pub const CBR_4800:   DWORD = 4800;
pub const CBR_9600:   DWORD = 9600;
pub const CBR_14400:  DWORD = 14400;
pub const CBR_19200:  DWORD = 19200;
pub const CBR_38400:  DWORD = 38400;
pub const CBR_56000:  DWORD = 56000;
pub const CBR_57600:  DWORD = 57600;
pub const CBR_115200: DWORD = 115200;
pub const CBR_128000: DWORD = 128000;
pub const CBR_256000: DWORD = 256000;

// fBits masks
pub const fBinary:           DWORD = 0x00000001;
pub const fParity:           DWORD = 0x00000002;
pub const fOutxCtsFlow:      DWORD = 0x00000004;
pub const fOutxDsrFlow:      DWORD = 0x00000008;
pub const fDtrControl:       DWORD = 0x00000030;
pub const fDsrSensitivity:   DWORD = 0x00000040;
pub const fTXContinueOnXoff: DWORD = 0x00000080;
pub const fOutX:             DWORD = 0x00000100;
pub const fInX:              DWORD = 0x00000200;
pub const fErrorChar:        DWORD = 0x00000400;
pub const fNull:             DWORD = 0x00000800;
pub const fRtsControl:       DWORD = 0x00003000;
pub const fAbortOnError:     DWORD = 0x00004000;
pub const fDummy2:           DWORD = 0xFFFF8000;

// Parity values
pub const NOPARITY:    BYTE = 0;
pub const ODDPARITY:   BYTE = 1;
pub const EVENPARITY:  BYTE = 2;
pub const MARKPARITY:  BYTE = 3;
pub const SPACEPARITY: BYTE = 4;

// StopBits values
pub const ONESTOPBIT:   BYTE = 0;
pub const ONE5STOPBITS: BYTE = 1;
pub const TWOSTOPBITS:  BYTE = 2;

impl DCB {
    pub fn new() -> Self {
        let mut dcb: DCB = unsafe { mem::zeroed() };
        dcb.DCBlength = mem::size_of_val(&dcb) as DWORD;
        dcb
    }
}

// EscapeCommFunction values
pub const SETXOFF:  DWORD = 1;
pub const SETXON:   DWORD = 2;
pub const SETRTS:   DWORD = 3;
pub const CLRRTS:   DWORD = 4;
pub const SETDTR:   DWORD = 5;
pub const CLRDTR:   DWORD = 6;
pub const SETBREAK: DWORD = 8;
pub const CLRBREAK: DWORD = 9;

// Modem status masks
pub const MS_CTS_ON:  DWORD = 0x0010;
pub const MS_DSR_ON:  DWORD = 0x0020;
pub const MS_RING_ON: DWORD = 0x0040;
pub const MS_RLSD_ON: DWORD = 0x0080;

#[derive(Copy,Clone,Debug)]
#[repr(C)]
pub struct COMMTIMEOUTS {
    pub ReadIntervalTimeout: DWORD,
    pub ReadTotalTimeoutMultiplier: DWORD,
    pub ReadTotalTimeoutConstant: DWORD,
    pub WriteTotalTimeoutMultiplier: DWORD,
    pub WriteTotalTimeoutConstant: DWORD
}

extern "system" {
    pub fn CreateFileW(lpFileName: LPCWSTR,
                       dwDesiredAccess: DWORD,
                       dwSharedMode: DWORD,
                       lpSecurityAttributes: LPSECURITY_ATTRIBUTES,
                       dwCreationDisposition: DWORD,
                       dwFlagsAndAttributes: DWORD,
                       hTemplmateFile: HANDLE) -> HANDLE;
    pub fn CloseHandle(hObject: HANDLE) -> BOOL;
    pub fn ReadFile(hFile: HANDLE,
                    lpBuffer: LPVOID,
                    nNumberOfBytesToRead: DWORD,
                    lpNumberOfBytesRead: LPDWORD,
                    lpOverlapped: LPOVERLAPPED) -> BOOL;
    pub fn WriteFile(hFile: HANDLE,
                     lpBuffer: LPVOID,
                     nNumberOfBytesToWrite: DWORD,
                     lpNumberOfBytesWritten: LPDWORD,
                     lpOverlapped: LPOVERLAPPED) -> BOOL;
    pub fn FlushFileBuffers(hFile: HANDLE) -> BOOL;

    pub fn GetCommState(hFile: HANDLE, lpDCB: *mut DCB) -> BOOL;
    pub fn SetCommState(hFile: HANDLE, lpDCB: *const DCB) -> BOOL;
    pub fn GetCommTimeouts(hFile: HANDLE, lpCommTimeouts: *mut COMMTIMEOUTS) -> BOOL;
    pub fn SetCommTimeouts(hFile: HANDLE, lpCommTimeouts: *const COMMTIMEOUTS) -> BOOL;
    pub fn EscapeCommFunction(hFile: HANDLE, dwFunc: DWORD) -> BOOL;
    pub fn GetCommModemStatus(hFile: HANDLE, lpModemStat: *mut DWORD) -> BOOL;

    pub fn GetLastError() -> DWORD;
}
