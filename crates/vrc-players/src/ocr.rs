//! Text lines of a frame, from local-multimodal-infra's direct OCR endpoint:
//! `POST <url>?model=<model>` with the image as the body, answering
//! `{"lines": [{"text", "confidence", "bbox": {"x", "y", "width", "height"}}]}`.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use jpeg_encoder::{ColorType, Encoder};

#[derive(Clone, Debug, PartialEq)]
pub struct OcrLine {
    pub text: String,
    pub confidence: f32,
    /// Pixels of the image sent: left, top, width, height.
    pub bbox: [f32; 4],
}

#[derive(Clone)]
pub struct OcrClient {
    host: String,
    port: u16,
    path: String,
    pub model: String,
    pub token: Option<String>,
}

impl OcrClient {
    /// `url` like `http://10.88.0.1:17890/v1/ocr/lines`.
    pub fn new(url: &str, model: &str) -> Result<OcrClient> {
        let rest = url.strip_prefix("http://").context("only http:// OCR URLs")?;
        let (authority, path) = rest.split_at(rest.find('/').unwrap_or(rest.len()));
        let (host, port) = match authority.rsplit_once(':') {
            Some((h, p)) => (h.to_string(), p.parse()?),
            None => (authority.to_string(), 80),
        };
        Ok(OcrClient { host, port, path: path.to_string(), model: model.into(), token: None })
    }

    /// The lines of an RGB8 image (sent as JPEG).
    pub fn lines_rgb(&self, rgb: &[u8], width: u16, height: u16) -> Result<Vec<OcrLine>> {
        let mut jpeg = Vec::new();
        Encoder::new(&mut jpeg, 90).encode(rgb, width, height, ColorType::Rgb)?;
        self.lines(&jpeg, "image/jpeg")
    }

    /// The lines of an encoded image.
    pub fn lines(&self, image: &[u8], content_type: &str) -> Result<Vec<OcrLine>> {
        let mut s = TcpStream::connect((self.host.as_str(), self.port))
            .with_context(|| format!("OCR at {}:{} is not up", self.host, self.port))?;
        s.set_read_timeout(Some(Duration::from_secs(10)))?;
        let auth = self.token.as_ref().map(|t| format!("Authorization: Bearer {t}\r\n")).unwrap_or_default();
        write!(
            s,
            "POST {}?model={} HTTP/1.0\r\nHost: {}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\n{auth}\r\n",
            self.path,
            self.model,
            self.host,
            image.len()
        )?;
        s.write_all(image)?;
        let mut reply = Vec::new();
        s.read_to_end(&mut reply)?;
        let reply = String::from_utf8_lossy(&reply);
        let (head, body) = reply.split_once("\r\n\r\n").context("not an HTTP reply")?;
        let status = head.split_whitespace().nth(1).unwrap_or("");
        if status != "200" {
            bail!("OCR answered {status}: {}", body.chars().take(200).collect::<String>());
        }
        parse(body)
    }
}

fn parse(body: &str) -> Result<Vec<OcrLine>> {
    let v: serde_json::Value = serde_json::from_str(body)?;
    let lines = v["lines"].as_array().context("no lines in the OCR answer")?;
    Ok(lines
        .iter()
        .filter_map(|l| {
            let b = &l["bbox"];
            Some(OcrLine {
                text: l["text"].as_str()?.to_string(),
                confidence: l["confidence"].as_f64().unwrap_or(0.0) as f32,
                bbox: [
                    b["x"].as_f64()? as f32,
                    b["y"].as_f64()? as f32,
                    b["width"].as_f64()? as f32,
                    b["height"].as_f64()? as f32,
                ],
            })
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_answer() {
        let body = r#"{"lines": [{"text": "xkeyC", "confidence": 0.93, "bbox": {"x": 10, "y": 20, "width": 60, "height": 14}}, {"text": "?"}]}"#;
        let lines = parse(body).unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].text, "xkeyC");
        assert_eq!(lines[0].bbox, [10.0, 20.0, 60.0, 14.0]);
        let c = OcrClient::new("http://10.88.0.1:17890/v1/ocr/lines", "ppocrv5-mobile-onnx").unwrap();
        assert_eq!((c.host.as_str(), c.port, c.path.as_str()), ("10.88.0.1", 17890, "/v1/ocr/lines"));
    }
}
