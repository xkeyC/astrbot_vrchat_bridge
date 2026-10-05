//! VRChat's OSC: inputs and parameters sent over UDP (port 9000), and
//! parameters read over OSCQuery (HTTP on a port VRChat picks and writes to
//! its log).
//!
//! The bot needs few of them: `/avatar/eyeheight` (world metres, readable
//! and writable) to turn stereo distances into world ones, the avatar's own
//! velocity (`VelocityX/Y/Z`, `Grounded`) for odometry, and the movement
//! inputs.

use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{bail, Context, Result};

/// Where VRChat's logs are for the bot user (Proton prefix), under `$HOME`.
pub const LOG_DIR: &str =
    ".local/share/Steam/steamapps/compatdata/438100/pfx/drive_c/users/steamuser/AppData/LocalLow/VRChat/VRChat";

pub struct Osc {
    udp: UdpSocket,
    send_to: String,
    query_port: u16,
}

impl Osc {
    /// OSC to `127.0.0.1:9000`, OSCQuery on the port the newest log names.
    pub fn connect() -> Result<Osc> {
        let home = std::env::var("HOME").context("no HOME")?;
        let port = oscquery_port(&Path::new(&home).join(LOG_DIR))?;
        Osc::with_ports("127.0.0.1:9000", port)
    }

    pub fn with_ports(send_to: &str, query_port: u16) -> Result<Osc> {
        Ok(Osc { udp: UdpSocket::bind("127.0.0.1:0")?, send_to: send_to.into(), query_port })
    }

    /// Another handle to the same OSC and OSCQuery.
    pub fn with_ports_from(other: &Osc) -> Result<Osc> {
        Osc::with_ports(&other.send_to, other.query_port)
    }

    /// Sends a float (`/input/Vertical`, `/avatar/eyeheight`, ...).
    pub fn send_f32(&self, address: &str, value: f32) -> Result<()> {
        self.udp.send_to(&message(address, b",f", &value.to_be_bytes()), &self.send_to)?;
        Ok(())
    }

    /// Sends an int (`/input/Jump`, buttons).
    pub fn send_i32(&self, address: &str, value: i32) -> Result<()> {
        self.udp.send_to(&message(address, b",i", &value.to_be_bytes()), &self.send_to)?;
        Ok(())
    }

    /// The first value of an OSCQuery node (`/avatar/eyeheight`,
    /// `/avatar/parameters/VelocityZ`, ...), as a number (booleans as 0/1).
    pub fn query(&self, path: &str) -> Result<f64> {
        let mut s = TcpStream::connect(("127.0.0.1", self.query_port)).context("OSCQuery is not up")?;
        s.set_read_timeout(Some(Duration::from_secs(2)))?;
        write!(s, "GET {path} HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n")?;
        let mut body = String::new();
        s.read_to_string(&mut body)?;
        first_value(&body).with_context(|| format!("no VALUE in OSCQuery {path}"))
    }

    /// The avatar's eye height, world metres.
    pub fn eye_height(&self) -> Result<f64> {
        self.query("/avatar/eyeheight")
    }
}

/// An OSC message with one argument.
fn message(address: &str, tag: &[u8], arg: &[u8]) -> Vec<u8> {
    let pad = |b: &[u8]| {
        let mut v = b.to_vec();
        v.push(0);
        while v.len() % 4 != 0 {
            v.push(0);
        }
        v
    };
    let mut out = pad(address.as_bytes());
    out.extend(pad(tag));
    out.extend_from_slice(arg);
    out
}

/// `VALUE`'s first element in an OSCQuery JSON reply.
fn first_value(reply: &str) -> Option<f64> {
    let at = reply.find("\"VALUE\"")?;
    let rest = &reply[at + 7..];
    let start = rest.find('[')? + 1;
    let item = rest[start..].split([',', ']']).next()?.trim();
    match item {
        "true" => Some(1.0),
        "false" => Some(0.0),
        n => n.parse().ok(),
    }
}

/// The OSCQuery port in the newest VRChat log of `dir`.
pub fn oscquery_port(dir: &Path) -> Result<u16> {
    let newest: PathBuf = std::fs::read_dir(dir)
        .with_context(|| format!("no VRChat logs in {}", dir.display()))?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().starts_with("output_log_"))
        .max_by_key(|e| e.metadata().and_then(|m| m.modified()).ok())
        .map(|e| e.path())
        .context("no VRChat log")?;
    let log = String::from_utf8_lossy(&std::fs::read(&newest)?).into_owned();
    let marker = "of type OSCQuery on ";
    let Some(at) = log.rfind(marker) else { bail!("VRChat has not started OSCQuery yet") };
    let digits: String = log[at + marker.len()..].chars().take_while(char::is_ascii_digit).collect();
    Ok(digits.parse()?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_messages() {
        let m = message("/input/Jump", b",i", &1i32.to_be_bytes());
        assert_eq!(m.len(), 12 + 4 + 4);
        assert_eq!(&m[..12], b"/input/Jump\0");
        assert_eq!(&m[12..16], b",i\0\0");
    }

    #[test]
    fn reads_values() {
        let reply = "HTTP/1.0 200 OK\r\n\r\n{\"FULL_PATH\":\"/avatar/eyeheight\",\"TYPE\":\"f\",\"VALUE\":[1.4050436]}";
        assert_eq!(first_value(reply), Some(1.4050436));
        assert_eq!(first_value("{\"VALUE\":[true]}"), Some(1.0));
        assert_eq!(first_value("{\"TYPE\":\"f\"}"), None);
    }
}
