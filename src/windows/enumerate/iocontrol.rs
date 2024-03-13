use std::{ffi::OsStr, os::windows::ffi::OsStrExt, ptr};
use winapi::ctypes::c_void;
use winapi::shared::minwindef::{UCHAR, ULONG, USHORT};
use winapi::shared::usbioctl::IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION;
use winapi::shared::usbioctl::IOCTL_USB_GET_NODE_CONNECTION_INFORMATION;
use winapi::shared::usbioctl::USB_CONNECTION_STATUS;
use winapi::shared::usbspec::USB_STRING_DESCRIPTOR_TYPE;
use winapi::um::fileapi::CreateFileW;
use winapi::um::fileapi::OPEN_EXISTING;
use winapi::um::handleapi::INVALID_HANDLE_VALUE;
use winapi::um::ioapiset::DeviceIoControl;
use winapi::um::winbase::SECURITY_ANONYMOUS;
use winapi::um::winnt::FILE_GENERIC_WRITE;
use winapi::um::winnt::FILE_SHARE_WRITE;
use winapi::um::winnt::{BOOLEAN, HANDLE, LPCWSTR};

use crate::windows::error;
use crate::Error;

#[derive(Default)]
#[allow(non_snake_case)]
#[repr(C)]
#[repr(packed)]
struct UsbEndpointDescriptor {
    bLength: UCHAR,
    bDescriptorType: UCHAR,
    bEndpointAddress: UCHAR,
    bmAttributes: UCHAR,
    wMaxPacketSize: USHORT,
    bInterval: UCHAR,
}

#[derive(Default)]
#[allow(non_snake_case)]
#[repr(C)]
#[repr(packed)]
struct UsbPipeInfo {
    EndpointDescriptor: UsbEndpointDescriptor,
    ScheduleOffset: ULONG,
}

#[derive(Default)]
#[allow(non_snake_case)]
#[repr(C)]
#[repr(packed)]
struct UsbDeviceDescriptor {
    bLength: UCHAR,
    bDescriptorType: UCHAR,
    bcdUSB: USHORT,
    bDeviceClass: UCHAR,
    bDeviceSubClass: UCHAR,
    bDeviceProtocol: UCHAR,
    bMaxPacketSize0: UCHAR,
    idVendor: USHORT,
    idProduct: USHORT,
    bcdDevice: USHORT,
    iManufacturer: UCHAR,
    iProduct: UCHAR,
    iSerialNumber: UCHAR,
    bNumConfigurations: UCHAR,
}

#[derive(Default)]
#[allow(non_snake_case)]
#[repr(C)]
#[repr(packed)]
struct UsbNodeConnectionInformation {
    ConnectionIndex: ULONG,
    DeviceDescriptor: UsbDeviceDescriptor,
    CurrentConfigurationValue: UCHAR,
    LowSpeed: BOOLEAN,
    DeviceIsHub: BOOLEAN,
    DeviceAddress: USHORT,
    NumberOfOpenPipes: ULONG,
    ConnectionStatus: USB_CONNECTION_STATUS,
    PipeList: [UsbPipeInfo; 0],
}

#[derive(Default)]
#[allow(non_snake_case)]
#[repr(C)]
#[repr(packed)]
struct UsbDescriptorRequestSetupPacket {
    bmRequest: UCHAR,
    bRequest: UCHAR,
    wValue: USHORT,
    wIndex: USHORT,
    wLength: USHORT,
}

#[derive(Default)]
#[allow(non_snake_case)]
#[repr(C)]
#[repr(packed)]
struct UsbDescriptorRequest {
    ConnectionIndex: ULONG,
    SetupPacket: UsbDescriptorRequestSetupPacket,
    Data: [UCHAR; 0],
}

#[derive(Debug)]
pub enum IoDescriptor {
    Manufacturer,
    Product,
    #[allow(dead_code)]
    SerialNumber,
}

pub struct IoControl;

impl IoControl {
    fn get_descriptor_id(
        hdevice: &HANDLE,
        port_number: u8,
        descriptor: &IoDescriptor,
    ) -> Result<u8, Error> {
        let mut lpinbuffer = UsbNodeConnectionInformation {
            ConnectionIndex: port_number as ULONG,
            ..Default::default()
        };
        let lpinbuffer_ptr: *mut c_void = Self::get_mut_ptr(&mut lpinbuffer);

        let mut lpoutbuffer = UsbNodeConnectionInformation::default();
        let lpoutbuffer_ptr: *mut c_void = Self::get_mut_ptr(&mut lpoutbuffer);

        if unsafe {
            // https://learn.microsoft.com/en-us/windows/win32/api/ioapiset/nf-ioapiset-deviceiocontrol
            DeviceIoControl(
                *hdevice,
                IOCTL_USB_GET_NODE_CONNECTION_INFORMATION,
                lpinbuffer_ptr,
                (std::mem::size_of::<UsbNodeConnectionInformation>()) as u32,
                lpoutbuffer_ptr,
                (std::mem::size_of::<UsbNodeConnectionInformation>()) as u32,
                ptr::null_mut(),
                ptr::null_mut(),
            )
        } != 0
        {
            let descriptor_id: u8 = match descriptor {
                IoDescriptor::Manufacturer => lpoutbuffer.DeviceDescriptor.iManufacturer,
                IoDescriptor::Product => lpoutbuffer.DeviceDescriptor.iProduct,
                IoDescriptor::SerialNumber => lpoutbuffer.DeviceDescriptor.iSerialNumber,
            };
            Ok(descriptor_id)
        } else {
            Err(error::last_os_error())
        }
    }

    pub fn get_string_descriptor(
        hdevice: &HANDLE,
        port_number: u8,
        descriptor: &IoDescriptor,
    ) -> Result<String, Error> {
        let descriptor_id: u8 = Self::get_descriptor_id(hdevice, port_number, descriptor)?;

        const MAX_USB_STRING_LENGTH: u32 = 255u32;

        let mut lpinbuffer = UsbDescriptorRequest {
            // Indicate the port from which the descriptor will be requested
            ConnectionIndex: port_number as ULONG,
            ..Default::default()
        };

        // wValue = Descriptor Type (high) and Descriptor Index (low byte)
        lpinbuffer.SetupPacket.wValue =
            (((USB_STRING_DESCRIPTOR_TYPE as u32) << 8) | descriptor_id as u32) as USHORT;
        // wIndex = Zero (or Language ID for String Descriptors)
        lpinbuffer.SetupPacket.wIndex = 0;
        // wLength = Length of descriptor buffer
        lpinbuffer.SetupPacket.wLength = MAX_USB_STRING_LENGTH as USHORT;

        let lpinbuffer_ptr = Self::get_mut_ptr(&mut lpinbuffer);
        let ninbuffersize = (std::mem::size_of::<UsbDescriptorRequest>()) as u32;

        let mut lpoutbuffer: [u16; (MAX_USB_STRING_LENGTH) as usize] =
            [0; (MAX_USB_STRING_LENGTH) as usize];
        let lpoutbuffer_ptr = Self::get_mut_ptr(&mut lpoutbuffer);
        let noutbuffersize = (lpoutbuffer.len() * std::mem::size_of::<u16>()) as u32;

        let result = unsafe {
            // https://learn.microsoft.com/en-us/windows/win32/api/ioapiset/nf-ioapiset-deviceiocontrol
            DeviceIoControl(
                *hdevice,
                IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION,
                lpinbuffer_ptr,
                ninbuffersize,
                lpoutbuffer_ptr,
                noutbuffersize,
                ptr::null_mut(),
                ptr::null_mut(),
            )
        };

        if result != 0 {
            let start = (std::mem::size_of::<UsbDescriptorRequest>() as u32)
                / std::mem::size_of::<u16>() as u32;
            let b = String::from_utf16_lossy(&lpoutbuffer[(start + 1) as usize..]); // +1 for alignment
            let b = b.trim_end_matches('\0');
            Ok(b.to_string())
        } else {
            Err(error::last_os_error())
        }
    }

    pub fn get_handle(device_name: &mut String) -> Result<HANDLE, Error> {
        device_name.insert_str(0, r"\\.\");
        let lpfilename = OsStr::new(device_name)
            .encode_wide()
            .chain(Some(0))
            .collect::<Vec<_>>();
        let lpfilename_ptr: LPCWSTR = lpfilename.as_ptr();

        let handle = unsafe {
            // https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilew
            CreateFileW(
                lpfilename_ptr,
                FILE_GENERIC_WRITE,
                FILE_SHARE_WRITE,
                ptr::null_mut(),
                OPEN_EXISTING,
                SECURITY_ANONYMOUS,
                0 as HANDLE,
            )
        };

        if handle == INVALID_HANDLE_VALUE {
            Err(error::last_os_error())
        } else {
            Ok(handle)
        }
    }

    fn get_mut_ptr<T>(buf: &mut T) -> *mut c_void {
        let ptr: *mut c_void = buf as *mut _ as *mut c_void;
        ptr
    }
}
