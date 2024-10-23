use std::str::FromStr;

use wasm_bindgen::{JsCast, UnwrapThrowExt};
use wasm_bindgen_futures::JsFuture;
use web_sys::{ReadableStreamByobReader, ReadableStreamGetReaderOptions, SerialOutputSignals};

use crate::{DataBits, FlowControl, Parity, SerialPort, StopBits};

/// Provides a blocking interface into the WebSerial api
#[derive(Debug)]
pub struct WebPort {
    inner: web_sys::SerialPort,
    name: String,

    options: SerialPortOptions,
    timeout: std::time::Duration,
}

#[derive(Debug, Clone)]
struct SerialPortOptions {
    baud_rate: u32,
    data_bits: DataBits,
    flow_control: FlowControl,
    parity: Parity,
    stop_bits: StopBits,
    buffer_size: u32,
}

impl Default for SerialPortOptions {
    fn default() -> Self {
        SerialPortOptions {
            baud_rate: 9600,
            data_bits: DataBits::Eight,
            flow_control: FlowControl::None,
            parity: Parity::None,
            stop_bits: StopBits::One,
            buffer_size: 255,
        }
    }
}

impl From<SerialPortOptions> for web_sys::SerialOptions {
    fn from(value: SerialPortOptions) -> Self {
        let options = web_sys::SerialOptions::new(value.baud_rate);
        options.set_buffer_size(value.buffer_size);
        options.set_data_bits(value.data_bits.into());
        options.set_stop_bits(value.stop_bits.into());
        options.set_flow_control(match value.flow_control {
            FlowControl::None => web_sys::FlowControlType::None,
            FlowControl::Software => web_sys::FlowControlType::None,
            FlowControl::Hardware => web_sys::FlowControlType::Hardware,
        });
        options.set_parity(match value.parity {
            Parity::None => web_sys::ParityType::None,
            Parity::Odd => web_sys::ParityType::Odd,
            Parity::Even => web_sys::ParityType::Even,
        });
        options
    }
}

unsafe impl Send for WebPort {}

impl WebPort {
    fn write_signals(&self, signals: SerialOutputSignals) {
        futures_executor::block_on(async {
            JsFuture::from(self.inner.set_signals_with_signals(&signals))
                .await
                .unwrap_throw();
        });
    }

    fn reopen(&self) -> crate::Result<()> {
        futures_executor::block_on(async {
            JsFuture::from(self.inner.close()).await.map_err(|v| {
                crate::Error::new(crate::ErrorKind::InvalidInput, v.as_string().unwrap_throw())
            })?;
            JsFuture::from(self.inner.open(&self.options.clone().into()))
                .await
                .map_err(|v| {
                    crate::Error::new(crate::ErrorKind::InvalidInput, v.as_string().unwrap_throw())
                })?;
            Ok(())
        })
    }
}

impl std::io::Write for WebPort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let writable = self.inner.writable();
        let writer = writable
            .get_writer()
            .map_err(|_| std::io::ErrorKind::Unsupported)?;

        let writable_size = writer
            .desired_size()
            .unwrap_or_default()
            .map(|v| v as usize)
            .unwrap_or(255);

        futures_executor::block_on(async {
            let buffer = unsafe { js_sys::Uint8Array::view(&buf[..writable_size]) };
            // FIXME: native promises are not cancelable and thus cannot not have a timeout, this needs to be done by storing the future for the next call instead

            // This should instantly resolve, given desired_size; so we don't have a timeout
            JsFuture::from(writer.write_with_chunk(&buffer))
                .await
                .unwrap_throw();
        });
        Ok(writable_size)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let writable = self.inner.writable();
        let writer = writable
            .get_writer()
            .map_err(|_| std::io::ErrorKind::Unsupported)?;

        futures_executor::block_on(async {
            JsFuture::from(writer.ready()).await.unwrap_throw();
        });

        Ok(())
    }
}

impl std::io::Read for WebPort {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let readable = self.inner.readable();
        let reader_options = ReadableStreamGetReaderOptions::new();
        reader_options.set_mode(web_sys::ReadableStreamReaderMode::Byob);
        let reader = readable.get_reader_with_options(&reader_options);
        let reader = ReadableStreamByobReader::unchecked_from_js_ref(&reader);

        let buffer = unsafe { js_sys::Uint8Array::view(buf) };
        let result = futures_executor::block_on(async {
            // FIXME: native promises are not cancelable and thus cannot not have a timeout, this needs to be done by storing the future for the next call instead
            JsFuture::from(reader.read_with_array_buffer_view(&buffer))
                .await
                .unwrap_throw()
        });

        let result = js_sys::Object::unchecked_from_js(result);
        let done =
            js_sys::Reflect::get(&result, &js_sys::JsString::from_str("done").unwrap_throw())
                .unwrap_throw();

        if done.is_truthy() {
            todo!("Stream closed")
        }

        let value =
            js_sys::Reflect::get(&result, &js_sys::JsString::from_str("value").unwrap_throw())
                .unwrap_throw();

        let value = js_sys::Uint8Array::unchecked_from_js(value);
        Ok(value.byte_length() as usize)
    }
}

impl SerialPort for WebPort {
    fn name(&self) -> Option<String> {
        Some(self.name.clone())
    }

    fn baud_rate(&self) -> crate::Result<u32> {
        Ok(self.options.baud_rate)
    }

    fn data_bits(&self) -> crate::Result<crate::DataBits> {
        Ok(self.options.data_bits)
    }

    fn flow_control(&self) -> crate::Result<crate::FlowControl> {
        Ok(self.options.flow_control)
    }

    fn parity(&self) -> crate::Result<crate::Parity> {
        Ok(self.options.parity)
    }

    fn stop_bits(&self) -> crate::Result<crate::StopBits> {
        Ok(self.options.stop_bits)
    }

    fn timeout(&self) -> std::time::Duration {
        self.timeout.clone()
    }

    fn write_request_to_send(&mut self, level: bool) -> crate::Result<()> {
        let signals = SerialOutputSignals::new();
        signals.set_request_to_send(level);
        self.write_signals(signals);
        Ok(())
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> crate::Result<()> {
        let signals = SerialOutputSignals::new();
        signals.set_data_terminal_ready(level);
        self.write_signals(signals);
        Ok(())
    }

    fn clear_break(&self) -> crate::Result<()> {
        let signals = SerialOutputSignals::new();
        signals.set_break(false);
        self.write_signals(signals);
        Ok(())
    }

    fn set_break(&self) -> crate::Result<()> {
        let signals = SerialOutputSignals::new();
        signals.set_break(true);
        self.write_signals(signals);
        Ok(())
    }

    fn set_timeout(&mut self, timeout: std::time::Duration) -> crate::Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> crate::Result<()> {
        self.options.baud_rate = baud_rate;
        self.reopen()
    }

    fn set_data_bits(&mut self, data_bits: crate::DataBits) -> crate::Result<()> {
        self.options.data_bits = data_bits;
        self.reopen()
    }

    fn set_flow_control(&mut self, flow_control: crate::FlowControl) -> crate::Result<()> {
        self.options.flow_control = flow_control;
        self.reopen()
    }

    fn set_parity(&mut self, parity: crate::Parity) -> crate::Result<()> {
        self.options.parity = parity;
        self.reopen()
    }

    fn set_stop_bits(&mut self, stop_bits: crate::StopBits) -> crate::Result<()> {
        self.options.stop_bits = stop_bits;
        self.reopen()
    }

    fn read_clear_to_send(&mut self) -> crate::Result<bool> {
        todo!()
    }

    fn read_data_set_ready(&mut self) -> crate::Result<bool> {
        todo!()
    }

    fn read_ring_indicator(&mut self) -> crate::Result<bool> {
        todo!()
    }

    fn read_carrier_detect(&mut self) -> crate::Result<bool> {
        todo!()
    }

    // TODO: in oredr to implement *_to_read you'd either need to implement your own {Read,Writ}ableStream or greedily read using an interval poll
    fn bytes_to_read(&self) -> crate::Result<u32> {
        // Not exposed functionality
        Ok(0)
    }

    fn bytes_to_write(&self) -> crate::Result<u32> {
        // Not exposed functionality
        Ok(0)
    }

    fn clear(&self, _buffer_to_clear: crate::ClearBuffer) -> crate::Result<()> {
        // We don't store an internal buffer, but the easiest way to to close() & open()
        self.reopen()
    }

    fn try_clone(&self) -> crate::Result<Box<dyn SerialPort>> {
        // TODO: serial is clonable, but also is locked so not sure if we should
        Err(crate::Error::new(
            crate::ErrorKind::NoDevice,
            "WebSerial device is not clonable",
        ))
    }
}
