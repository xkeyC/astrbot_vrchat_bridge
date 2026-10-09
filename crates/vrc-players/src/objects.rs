//! Things in a frame (a sofa, a chair, a plant...), from local-multimodal-
//! infra's direct detection endpoint (`POST /v1/detect/objects?model=<id>`
//! with the image as the body, answering `{"objects": [{"label",
//! "confidence", "bbox": {"x", "y", "width", "height"}}]}`). The caller
//! places them (`vrc_nav::pano`: the panorama's depth inside their boxes).
//!
//! The detector is YOLO11n on COCO's 80 kinds for now: players are found by
//! their name tags, not as "person".

use anyhow::{Context, Result};
use jpeg_encoder::{ColorType, Encoder};

use crate::ocr::{post, OcrClient};

pub const DETECT_MODEL: &str = "yolo11n.onnx";
/// The kinds worth a place on the map (COCO's names).
pub const KEPT: &[&str] = &[
    "couch", "chair", "bed", "dining table", "toilet", "tv", "potted plant", "refrigerator", "sink", "oven", "microwave", "bench",
    "clock", "vase", "laptop",
];
/// Sightings less sure than this are left out. Low: on VRChat's rendering
/// right ones often score 0.25-0.35 (a coffee table, a sofa seen close, a
/// microwave: first judged set, 8 frames); the map confirms a thing seen
/// again ([`vrc_map::Object::confirmed`]).
pub const MIN_SCORE: f32 = 0.25;
/// Images go at most this wide (the detector itself looks at 640).
const SEND_WIDTH: u32 = 960;

#[derive(Clone, Debug, PartialEq)]
pub struct Detection {
    pub label: String,
    pub confidence: f32,
    /// Frame pixels: left, top, width, height.
    pub bbox: [f32; 4],
}

impl Detection {
    /// Worth a place on the map: a kind KEPT, at least MIN_SCORE sure.
    pub fn kept(&self) -> bool {
        self.confidence >= MIN_SCORE && KEPT.contains(&self.label.as_str())
    }
}

/// A detection placed.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectSighting {
    pub label: String,
    pub score: f32,
    /// Its middle, at its foot (tracking space).
    pub at: [f32; 3],
    /// Its size across and up (tracking units).
    pub size: [f32; 2],
    /// Frame pixels: left, top, width, height.
    pub bbox: [f32; 4],
}

#[derive(Clone)]
pub struct DetectClient {
    host: String,
    port: u16,
    path: String,
    pub model: String,
}

impl DetectClient {
    /// The detection endpoint of the same service as `ocr`.
    pub fn from_ocr(ocr: &OcrClient, model: &str) -> DetectClient {
        DetectClient { host: ocr.host.clone(), port: ocr.port, path: "/v1/detect/objects".into(), model: model.into() }
    }

    /// What is in an RGB8 image `w` x `h` (sent smaller, boxes in its pixels).
    pub fn detect_rgb(&self, rgb: &[u8], w: u32, h: u32) -> Result<Vec<Detection>> {
        let f = w.div_ceil(SEND_WIDTH).max(1);
        let (sw, sh) = (w / f, h / f);
        let small: Vec<u8> = if f == 1 {
            rgb.to_vec()
        } else {
            let mut out = vec![0u8; (sw * sh * 3) as usize];
            for y in 0..sh {
                for x in 0..sw {
                    let mut sum = [0u32; 3];
                    for dy in 0..f {
                        for dx in 0..f {
                            let i = (((y * f + dy) * w + x * f + dx) * 3) as usize;
                            for k in 0..3 {
                                sum[k] += rgb[i + k] as u32;
                            }
                        }
                    }
                    let o = ((y * sw + x) * 3) as usize;
                    for k in 0..3 {
                        out[o + k] = (sum[k] / (f * f)) as u8;
                    }
                }
            }
            out
        };
        let mut jpeg = Vec::new();
        Encoder::new(&mut jpeg, 90).encode(&small, sw as u16, sh as u16, ColorType::Rgb)?;
        let body = post(&self.host, self.port, &self.path, &self.model, None, &jpeg, "image/jpeg", "detection")?;
        Ok(parse(&body)?
            .into_iter()
            .map(|d| Detection { bbox: d.bbox.map(|v| v * f as f32), ..d })
            .collect())
    }
}

fn parse(body: &str) -> Result<Vec<Detection>> {
    let v: serde_json::Value = serde_json::from_str(body)?;
    let objects = v["objects"].as_array().context("no objects in the detection answer")?;
    Ok(objects
        .iter()
        .filter_map(|o| {
            let b = &o["bbox"];
            Some(Detection {
                label: o["label"].as_str()?.to_string(),
                confidence: o["confidence"].as_f64().unwrap_or(0.0) as f32,
                bbox: [b["x"].as_f64()? as f32, b["y"].as_f64()? as f32, b["width"].as_f64()? as f32, b["height"].as_f64()? as f32],
            })
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_answer_and_the_endpoint() {
        let body = r#"{"objects": [{"label": "couch", "confidence": 0.8, "bbox": {"x": 10, "y": 20, "width": 300, "height": 150}}, {"label": "x"}]}"#;
        let d = parse(body).unwrap();
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].bbox, [10.0, 20.0, 300.0, 150.0]);
        assert!(d[0].kept());
        assert!(!Detection { label: "person".into(), ..d[0].clone() }.kept());
        assert!(!Detection { confidence: 0.1, ..d[0].clone() }.kept());
        let o = OcrClient::new("http://10.88.0.1:17890/v1/ocr/lines", "m").unwrap();
        let c = DetectClient::from_ocr(&o, DETECT_MODEL);
        assert_eq!((c.host.as_str(), c.port, c.path.as_str()), ("10.88.0.1", 17890, "/v1/detect/objects"));
    }
}
