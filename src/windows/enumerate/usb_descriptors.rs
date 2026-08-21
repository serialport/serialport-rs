use std::ffi::c_void;
use std::fs::{File, OpenOptions};
use std::mem::{size_of, size_of_val};
use std::os::windows::fs::OpenOptionsExt;
use std::os::windows::io::AsRawHandle;
use std::ptr;

use windows_sys::Win32::Devices::DeviceAndDriverInstallation::{
    CM_Get_DevNode_PropertyW, CM_Get_Device_IDW, CM_Get_Device_Interface_ListW,
    CM_Get_Device_Interface_List_SizeW, CM_Get_Parent, CM_GET_DEVICE_INTERFACE_LIST_PRESENT,
    CR_SUCCESS, MAX_DEVICE_ID_LEN,
};
use windows_sys::Win32::Devices::Properties::{
    DEVPKEY_Device_Address, DEVPKEY_Device_BusReportedDeviceDesc, DEVPROPTYPE, DEVPROP_TYPE_STRING,
    DEVPROP_TYPE_UINT32,
};
use windows_sys::Win32::Devices::Usb::{
    GUID_DEVINTERFACE_USB_HUB, IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION,
    IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX, USB_DESCRIPTOR_REQUEST_0, USB_DEVICE_DESCRIPTOR,
    USB_NODE_CONNECTION_INFORMATION_EX, USB_STRING_DESCRIPTOR_TYPE,
};
use windows_sys::Win32::Foundation::{DEVPROPKEY, FALSE, HANDLE, MAX_PATH};
use windows_sys::Win32::Storage::FileSystem::{FILE_SHARE_READ, FILE_SHARE_WRITE};
use windows_sys::Win32::System::IO::DeviceIoControl;

use super::{as_utf16, from_utf16_lossy_trimmed};

// A USB CDC serial port is a child device handled by Microsoft's usbser.sys. Its normal
// manufacturer and product properties therefore describe the driver, not the USB device. The
// physical USB parent has the bus-reported product description, but its manufacturer string must
// be read from the USB descriptor through the hub to which it is connected.

#[derive(Default)]
pub(super) struct DescriptorStrings {
    pub(super) manufacturer: Option<String>,
    pub(super) product: Option<String>,
}

struct UsbHub(File);

// USB_DESCRIPTOR_REQUEST ends in a variable-length byte array. This is the same structure with
// enough storage for the largest possible USB string descriptor (its length is one byte).
#[repr(C, packed)]
struct StringDescriptorRequest {
    connection_index: u32,
    setup_packet: USB_DESCRIPTOR_REQUEST_0,
    data: [u8; USB_STRING_DESCRIPTOR_MAX_LEN],
}

const USB_DESCRIPTOR_REQUEST_HEADER_LEN: u32 =
    (size_of::<u32>() + size_of::<USB_DESCRIPTOR_REQUEST_0>()) as u32;
const USB_STRING_DESCRIPTOR_HEADER_LEN: usize = 2;
const USB_STRING_DESCRIPTOR_MAX_LEN: usize = u8::MAX as usize;
const USB_STRING_DESCRIPTOR_INDEX_LANGUAGES: u8 = 0;
const USB_LANGID_NONE: u16 = 0;
const USB_LANGID_EN_US: u16 = 0x0409;
const USB_GET_DESCRIPTOR_REQUEST: u8 = 6;
const USB_GET_DESCRIPTOR_REQUEST_TYPE: u8 = 0x80;

pub(super) fn parent(devinst: u32) -> Option<u32> {
    let mut parent = 0;
    let result = unsafe { CM_Get_Parent(&mut parent, devinst, 0) };
    (result == CR_SUCCESS).then_some(parent)
}

pub(super) fn instance_id(devinst: u32) -> Option<String> {
    let mut buffer = [0u16; MAX_DEVICE_ID_LEN as usize];
    let result =
        unsafe { CM_Get_Device_IDW(devinst, buffer.as_mut_ptr(), (buffer.len() - 1) as u32, 0) };
    (result == CR_SUCCESS).then(|| from_utf16_lossy_trimmed(&buffer))
}

pub(super) fn strings(devinst: u32, expected_vid: u16, expected_pid: u16) -> DescriptorStrings {
    DescriptorStrings {
        manufacturer: manufacturer(devinst, expected_vid, expected_pid),
        product: string_property(devinst, &DEVPKEY_Device_BusReportedDeviceDesc),
    }
}

fn manufacturer(devinst: u32, expected_vid: u16, expected_pid: u16) -> Option<String> {
    let connection_index = u32_property(devinst, &DEVPKEY_Device_Address)?;
    let hub_instance_id = instance_id(parent(devinst)?)?;
    let hub = UsbHub::open(&hub_device_path(&hub_instance_id)?)?;
    let descriptor = hub.device_descriptor(connection_index)?;

    if descriptor.idVendor != expected_vid || descriptor.idProduct != expected_pid {
        return None;
    }
    if descriptor.iManufacturer == 0 {
        return None;
    }

    let language_id = hub
        .string_descriptor(
            connection_index,
            USB_STRING_DESCRIPTOR_INDEX_LANGUAGES,
            USB_LANGID_NONE,
        )
        .and_then(|descriptor| {
            string_descriptor_payload(&descriptor)?
                .get(..size_of::<u16>())?
                .try_into()
                .ok()
                .map(u16::from_le_bytes)
        })
        .unwrap_or(USB_LANGID_EN_US);

    let descriptor =
        hub.string_descriptor(connection_index, descriptor.iManufacturer, language_id)?;
    decode_string_descriptor(&descriptor)
}

fn string_property(devinst: u32, property_key: &DEVPROPKEY) -> Option<String> {
    let mut property_type: DEVPROPTYPE = 0;
    let mut buffer = [0u16; MAX_PATH as usize];
    let mut byte_len = size_of_val(&buffer) as u32;
    let result = unsafe {
        CM_Get_DevNode_PropertyW(
            devinst,
            property_key,
            &mut property_type,
            buffer.as_mut_ptr().cast(),
            &mut byte_len,
            0,
        )
    };

    if result != CR_SUCCESS
        || property_type != DEVPROP_TYPE_STRING
        || byte_len == 0
        || byte_len % size_of::<u16>() as u32 != 0
        || byte_len > size_of_val(&buffer) as u32
    {
        return None;
    }

    let len = byte_len as usize / size_of::<u16>();
    non_empty(from_utf16_lossy_trimmed(&buffer[..len]))
}

fn u32_property(devinst: u32, property_key: &DEVPROPKEY) -> Option<u32> {
    let mut property_type: DEVPROPTYPE = 0;
    let mut value = 0u32;
    let mut byte_len = size_of_val(&value) as u32;
    let result = unsafe {
        CM_Get_DevNode_PropertyW(
            devinst,
            property_key,
            &mut property_type,
            (&mut value as *mut u32).cast(),
            &mut byte_len,
            0,
        )
    };

    (result == CR_SUCCESS
        && property_type == DEVPROP_TYPE_UINT32
        && byte_len == size_of_val(&value) as u32)
        .then_some(value)
}

// A descriptor request is sent to the device's hub together with the device's connection index.
// This function maps the hub's device instance ID to the device path that DeviceIoControl opens.
fn hub_device_path(hub_instance_id: &str) -> Option<String> {
    let hub_instance_id = as_utf16(hub_instance_id);
    let mut list_len = 0;
    let result = unsafe {
        CM_Get_Device_Interface_List_SizeW(
            &mut list_len,
            &GUID_DEVINTERFACE_USB_HUB,
            hub_instance_id.as_ptr(),
            CM_GET_DEVICE_INTERFACE_LIST_PRESENT,
        )
    };
    if result != CR_SUCCESS || list_len <= 1 {
        return None;
    }

    let mut list = vec![0u16; list_len as usize];
    let result = unsafe {
        CM_Get_Device_Interface_ListW(
            &GUID_DEVINTERFACE_USB_HUB,
            hub_instance_id.as_ptr(),
            list.as_mut_ptr(),
            list_len,
            CM_GET_DEVICE_INTERFACE_LIST_PRESENT,
        )
    };
    if result != CR_SUCCESS {
        return None;
    }

    let path_len = list.iter().position(|&character| character == 0)?;
    non_empty(from_utf16_lossy_trimmed(&list[..path_len]))
}

impl UsbHub {
    fn open(device_path: &str) -> Option<Self> {
        OpenOptions::new()
            .read(true)
            .write(true)
            .share_mode(FILE_SHARE_READ | FILE_SHARE_WRITE)
            .open(device_path)
            .ok()
            .map(Self)
    }

    fn device_descriptor(&self, connection_index: u32) -> Option<USB_DEVICE_DESCRIPTOR> {
        let mut connection = USB_NODE_CONNECTION_INFORMATION_EX {
            ConnectionIndex: connection_index,
            ..Default::default()
        };
        self.device_io_control(
            IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX,
            (&mut connection as *mut USB_NODE_CONNECTION_INFORMATION_EX).cast(),
            size_of_val(&connection) as u32,
        )?;
        Some(connection.DeviceDescriptor)
    }

    fn string_descriptor(
        &self,
        connection_index: u32,
        descriptor_index: u8,
        language_id: u16,
    ) -> Option<Vec<u8>> {
        let mut request = StringDescriptorRequest {
            connection_index,
            setup_packet: USB_DESCRIPTOR_REQUEST_0 {
                bmRequest: USB_GET_DESCRIPTOR_REQUEST_TYPE,
                bRequest: USB_GET_DESCRIPTOR_REQUEST,
                wValue: ((USB_STRING_DESCRIPTOR_TYPE as u16) << 8) | descriptor_index as u16,
                wIndex: language_id,
                wLength: USB_STRING_DESCRIPTOR_MAX_LEN as u16,
            },
            data: [0; USB_STRING_DESCRIPTOR_MAX_LEN],
        };
        let request_len = size_of_val(&request) as u32;
        let bytes_returned = self.device_io_control(
            IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION,
            (&mut request as *mut StringDescriptorRequest).cast(),
            request_len,
        )?;
        let data_len = bytes_returned.checked_sub(USB_DESCRIPTOR_REQUEST_HEADER_LEN)? as usize;
        Some(request.data[..data_len.min(USB_STRING_DESCRIPTOR_MAX_LEN)].to_vec())
    }

    fn device_io_control(&self, code: u32, buffer: *mut c_void, len: u32) -> Option<u32> {
        let mut bytes_returned = 0;
        let result = unsafe {
            DeviceIoControl(
                self.0.as_raw_handle() as HANDLE,
                code,
                buffer,
                len,
                buffer,
                len,
                &mut bytes_returned,
                ptr::null_mut(),
            )
        };
        (result != FALSE).then_some(bytes_returned)
    }
}

fn decode_string_descriptor(descriptor: &[u8]) -> Option<String> {
    let bytes = string_descriptor_payload(descriptor)?;
    if bytes.len() % size_of::<u16>() != 0 {
        return None;
    }

    let utf16 = bytes
        .chunks_exact(size_of::<u16>())
        .map(|bytes| u16::from_le_bytes(bytes.try_into().unwrap()))
        .collect::<Vec<_>>();
    non_empty(from_utf16_lossy_trimmed(&utf16))
}

// A USB string descriptor starts with its byte length and descriptor type, followed by UTF-16LE.
fn string_descriptor_payload(descriptor: &[u8]) -> Option<&[u8]> {
    let (&descriptor_len, descriptor) = descriptor.split_first()?;
    let (&descriptor_type, payload) = descriptor.split_first()?;
    if descriptor_type != USB_STRING_DESCRIPTOR_TYPE as u8
        || descriptor_len as usize <= USB_STRING_DESCRIPTOR_HEADER_LEN
    {
        return None;
    }

    payload.get(..descriptor_len as usize - USB_STRING_DESCRIPTOR_HEADER_LEN)
}

fn non_empty(value: String) -> Option<String> {
    (!value.is_empty()).then_some(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_usb_string_descriptor() {
        let descriptor = [
            18,
            USB_STRING_DESCRIPTOR_TYPE as u8,
            b'T',
            0,
            b'e',
            0,
            b's',
            0,
            b't',
            0,
            b' ',
            0,
            b'A',
            0,
            b'B',
            0,
            b'C',
            0,
        ];

        assert_eq!(
            decode_string_descriptor(&descriptor).as_deref(),
            Some("Test ABC")
        );
    }

    #[test]
    fn reject_invalid_usb_string_descriptors() {
        assert!(decode_string_descriptor(&[]).is_none());
        assert!(decode_string_descriptor(&[2, 1]).is_none());
        assert!(decode_string_descriptor(&[3, USB_STRING_DESCRIPTOR_TYPE as u8, b'A']).is_none());
    }
}
