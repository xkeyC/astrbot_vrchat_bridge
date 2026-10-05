//! VRChat Web API, the way VRCX uses it: the bot account's session cookie,
//! a few REST calls and the pipeline WebSocket (notifications, friends'
//! locations).
//!
//! Login is interactive and done by the owner (`vrc-bridge login`): only
//! the resulting cookies are kept, never the password. When they expire the
//! bridge reports `auth_required` and the owner logs in again.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, bail, Context, Result};
use futures_util::StreamExt;
use serde_json::{json, Value};

pub const API: &str = "https://api.vrchat.cloud/api/1";
pub const PIPELINE: &str = "wss://pipeline.vrchat.cloud/";
/// VRChat asks API clients to identify themselves with a contact.
pub const USER_AGENT: &str = "astrbot-vrchat-bridge/0.2 (github.com/xkeyC/astrbot_vrchat_bridge)";
const COOKIES: [&str; 2] = ["auth", "twoFactorAuth"];
/// What the bot may join: friends+ (hidden), friends, invite / invite+ (private).
const JOINABLE_KINDS: [&str; 3] = ["hidden", "friends", "private"];

/// The session cookie is missing or expired: the owner must log in.
#[derive(Debug)]
pub struct AuthRequired(pub String);

impl std::fmt::Display for AuthRequired {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "VRChat login required: {}", self.0)
    }
}

impl std::error::Error for AuthRequired {}

/// `text` without session tokens (error messages may carry URLs).
pub fn redact(text: &str) -> String {
    let mut out = String::new();
    let mut rest = text;
    while let Some(at) = ["authToken=", "auth="].iter().filter_map(|k| rest.find(k).map(|i| (i, k.len()))).min() {
        let (i, len) = at;
        out.push_str(&rest[..i + len]);
        out.push_str("***");
        rest = &rest[i + len..];
        let end = rest.find(|c: char| c == '&' || c.is_whitespace() || "'\";".contains(c)).unwrap_or(rest.len());
        rest = &rest[end..];
    }
    out.push_str(rest);
    out
}

/// The access type of an instance location, or None when it is no instance
/// (`offline`, `private` - hidden from us - or `traveling`): `public`,
/// `hidden` (friends+), `friends`, `private` (invite, invite+),
/// `group-public` or `group` (members / plus).
pub fn instance_kind(location: &str) -> Option<&'static str> {
    // wrld_<hex and dashes>:<name>(~Tag or ~Tag(value))*
    let (world, rest) = location.split_once(':')?;
    let id = world.strip_prefix("wrld_")?;
    if id.is_empty() || !id.chars().all(|c| c.is_ascii_hexdigit() || c == '-') {
        return None;
    }
    let name_end = rest.find('~').unwrap_or(rest.len());
    let name = &rest[..name_end];
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
        return None;
    }
    let mut tags = &rest[name_end..];
    let mut seen: Vec<(String, Option<String>)> = Vec::new();
    while !tags.is_empty() {
        let t = tags.strip_prefix('~')?;
        let word_end = t.find(|c: char| !c.is_ascii_alphabetic()).unwrap_or(t.len());
        if word_end == 0 {
            return None;
        }
        let word = &t[..word_end];
        let mut after = &t[word_end..];
        let mut value = None;
        if let Some(v) = after.strip_prefix('(') {
            let close = v.find(')')?;
            let inner = &v[..close];
            if !inner.chars().all(|c| c.is_ascii_alphanumeric() || "_.-".contains(c)) {
                return None;
            }
            value = Some(inner.to_string());
            after = &v[close + 1..];
        }
        seen.push((word.to_string(), value));
        tags = after;
    }
    if seen.iter().any(|(w, _)| w == "group") {
        let public = seen.iter().any(|(w, v)| w == "groupAccessType" && v.as_deref() == Some("public"));
        return Some(if public { "group-public" } else { "group" });
    }
    for kind in ["hidden", "friends", "private"] {
        if seen.iter().any(|(w, v)| w == kind && v.is_some()) {
            return Some(match kind {
                "hidden" => "hidden",
                "friends" => "friends",
                _ => "private",
            });
        }
    }
    Some("public")
}

/// Whether the bot may go there: friends+, friends or invite instances only.
pub fn joinable(location: &str) -> bool {
    instance_kind(location).is_some_and(|k| JOINABLE_KINDS.contains(&k))
}

/// The `vrchat://launch` URL of a joinable instance.
pub fn launch_url(location: &str) -> Result<String> {
    if !joinable(location) {
        bail!("not a joinable instance: {location:?}");
    }
    Ok(format!("vrchat://launch?ref=vrchat.com&id={location}"))
}

/// The instance of a `vrchat://launch` URL: `id` and at most `ref`, nothing
/// else, the instance joinable.
pub fn launch_location(url: &str) -> Result<String> {
    let url = url.trim();
    let rest = url.strip_prefix("vrchat://launch").context("only vrchat://launch?id=<instance> URLs")?;
    let rest = rest.strip_prefix('/').unwrap_or(rest);
    let query = rest.strip_prefix('?').unwrap_or(if rest.is_empty() { "" } else { "\u{0}" });
    if query.contains('\u{0}') || query.contains('#') {
        bail!("only vrchat://launch?id=<instance> URLs");
    }
    let mut ids = Vec::new();
    let mut refs = 0;
    for pair in query.split('&').filter(|p| !p.is_empty()) {
        let (k, v) = pair.split_once('=').unwrap_or((pair, ""));
        match k {
            "id" => ids.push(percent_decode(v)?),
            "ref" => refs += 1,
            _ => bail!("only vrchat://launch?id=<instance> URLs"),
        }
    }
    if ids.len() != 1 || refs > 1 {
        bail!("only vrchat://launch?id=<instance> URLs");
    }
    let location = ids.remove(0);
    if !joinable(&location) {
        bail!("only friends+, friends or invite instances");
    }
    Ok(location)
}

fn percent_decode(s: &str) -> Result<String> {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'%' if i + 2 < bytes.len() => {
                let hex = std::str::from_utf8(&bytes[i + 1..i + 3])?;
                out.push(u8::from_str_radix(hex, 16)?);
                i += 3;
            }
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            b => {
                out.push(b);
                i += 1;
            }
        }
    }
    Ok(String::from_utf8(out)?)
}

/// `s` percent-encoded for a URL path segment or basic-auth part.
pub fn percent_encode(s: &str) -> String {
    s.bytes()
        .map(|b| {
            if b.is_ascii_alphanumeric() || b"-_.~".contains(&b) {
                (b as char).to_string()
            } else {
                format!("%{b:02X}")
            }
        })
        .collect()
}

pub struct VrcApi {
    pub cookie_file: PathBuf,
    pub cookies: BTreeMap<String, String>,
    http: reqwest::Client,
}

impl VrcApi {
    pub fn new(cookie_file: PathBuf) -> VrcApi {
        let http = reqwest::Client::builder().user_agent(USER_AGENT).build().expect("an HTTP client");
        VrcApi { cookie_file, cookies: BTreeMap::new(), http }
    }

    /// Loads the saved cookies; whether there is a session.
    pub fn load(&mut self) -> bool {
        self.cookies = std::fs::read_to_string(&self.cookie_file)
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        self.cookies.contains_key("auth")
    }

    fn save(&self) -> Result<()> {
        if let Some(dir) = self.cookie_file.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let mut f = std::fs::OpenOptions::new().create(true).write(true).truncate(true).open(&self.cookie_file)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&self.cookie_file, std::fs::Permissions::from_mode(0o600))?;
        }
        f.write_all(serde_json::to_string(&self.cookies)?.as_bytes())?;
        Ok(())
    }

    fn cookie_header(&self) -> String {
        self.cookies
            .iter()
            .filter(|(k, _)| COOKIES.contains(&k.as_str()))
            .map(|(k, v)| format!("{k}={v}"))
            .collect::<Vec<_>>()
            .join("; ")
    }

    /// One REST call with the session cookies.
    pub async fn call(&self, method: reqwest::Method, path: &str, body: Option<Value>) -> Result<Value> {
        if !self.cookies.contains_key("auth") {
            return Err(AuthRequired("not logged in".into()).into());
        }
        let mut req = self
            .http
            .request(method, format!("{API}{path}"))
            .header("Cookie", self.cookie_header())
            .timeout(std::time::Duration::from_secs(20));
        if let Some(b) = body {
            req = req.json(&b);
        }
        let resp = req.send().await.map_err(|e| anyhow!(redact(&e.to_string())))?;
        let status = resp.status().as_u16();
        if status == 401 {
            return Err(AuthRequired("session expired".into()).into());
        }
        if status == 429 {
            bail!("rate limited by VRChat");
        }
        let data: Value = resp.json().await.unwrap_or(Value::Null);
        if status >= 300 {
            bail!("HTTP {status}: {data}");
        }
        Ok(data)
    }

    pub async fn me(&self) -> Result<Value> {
        let data = self.call(reqwest::Method::GET, "/auth/user", None).await?;
        if data.get("requiresTwoFactorAuth").is_some() {
            return Err(AuthRequired("two-factor verification pending".into()).into());
        }
        Ok(data)
    }

    /// Online friends (with their location) first, then offline ones.
    pub async fn friends(&self) -> Result<Vec<Value>> {
        let mut out = Vec::new();
        for offline in ["false", "true"] {
            let mut offset = 0;
            loop {
                let page = self
                    .call(reqwest::Method::GET, &format!("/auth/user/friends?offline={offline}&n=100&offset={offset}"), None)
                    .await?;
                let items = page.as_array().cloned().unwrap_or_default();
                let n = items.len();
                out.extend(items);
                if n < 100 {
                    break;
                }
                offset += 100;
            }
        }
        Ok(out)
    }

    pub async fn invite(&self, user_id: &str, instance: &str) -> Result<Value> {
        self.call(reqwest::Method::POST, &format!("/invite/{}", percent_encode(user_id)), Some(json!({"instanceId": instance})))
            .await
    }

    /// The session's auth cookie.
    pub fn auth(&self) -> Result<String> {
        Ok(self.cookies.get("auth").cloned().ok_or_else(|| AuthRequired("not logged in".into()))?)
    }
}

/// Follows the pipeline (with the session's `auth` cookie) until it closes,
/// handing each event (type, content) to `on_event`.
pub async fn pipeline<F, Fut>(auth: &str, mut on_event: F) -> Result<()>
where
    F: FnMut(String, Value) -> Fut,
    Fut: std::future::Future<Output = ()>,
{
    {
        let url = format!("{PIPELINE}?authToken={}", percent_encode(auth));
        let mut req = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(url.as_str())
            .map_err(|e| anyhow!(redact(&e.to_string())))?;
        req.headers_mut().insert("User-Agent", USER_AGENT.parse()?);
        let (mut ws, _) = tokio_tungstenite::connect_async(req).await.map_err(|e| anyhow!(redact(&e.to_string())))?;
        tracing::info!("pipeline connected");
        while let Some(msg) = ws.next().await {
            let msg = msg.map_err(|e| anyhow!(redact(&e.to_string())))?;
            let tokio_tungstenite::tungstenite::Message::Text(text) = msg else { continue };
            let Ok(data) = serde_json::from_str::<Value>(text.as_str()) else { continue };
            let kind = data["type"].as_str().unwrap_or("").to_string();
            let content = match &data["content"] {
                Value::String(s) if s.starts_with('{') => serde_json::from_str(s).unwrap_or(Value::Null),
                Value::String(s) => json!({"text": s}),
                v => v.clone(),
            };
            on_event(kind, content).await;
        }
    }
    Ok(())
}

impl VrcApi {
    /// Asks for the bot account, its password and the 2FA code on the
    /// terminal; keeps the cookies.
    pub async fn login(&mut self) -> Result<()> {
        print!("VRChat username or email: ");
        std::io::stdout().flush()?;
        let mut username = String::new();
        std::io::stdin().read_line(&mut username)?;
        let password = rpassword::prompt_password("Password (not stored): ")?;
        let resp = self
            .http
            .get(format!("{API}/auth/user"))
            .basic_auth(percent_encode(username.trim()), Some(percent_encode(&password)))
            .send()
            .await?;
        self.take(&resp);
        let status = resp.status();
        let data: Value = resp.json().await.unwrap_or(Value::Null);
        if !status.is_success() {
            bail!("login failed: HTTP {status} {data}");
        }
        let methods: Vec<String> = data["requiresTwoFactorAuth"]
            .as_array()
            .map(|a| a.iter().filter_map(|m| m.as_str().map(String::from)).collect())
            .unwrap_or_default();
        if !methods.is_empty() {
            let kind = if methods.iter().any(|m| m == "totp") {
                "totp"
            } else if methods.iter().any(|m| m == "emailOtp") {
                "emailotp"
            } else {
                "otp"
            };
            print!("Two-factor code ({}): ", methods.join(", "));
            std::io::stdout().flush()?;
            let mut code = String::new();
            std::io::stdin().read_line(&mut code)?;
            let resp = self
                .http
                .post(format!("{API}/auth/twofactorauth/{kind}/verify"))
                .header("Cookie", format!("auth={}", self.cookies.get("auth").cloned().unwrap_or_default()))
                .json(&json!({"code": code.trim()}))
                .send()
                .await?;
            self.take(&resp);
            let ok = resp.status().is_success();
            let result: Value = resp.json().await.unwrap_or(Value::Null);
            if !ok || result["verified"] != json!(true) {
                bail!("two-factor verification failed: {result}");
            }
        }
        let me = self.me().await?;
        self.save()?;
        println!(
            "Logged in as {} ({}); cookies saved to {}",
            me["displayName"].as_str().unwrap_or("?"),
            me["id"].as_str().unwrap_or("?"),
            self.cookie_file.display()
        );
        Ok(())
    }

    fn take(&mut self, resp: &reqwest::Response) {
        for c in resp.headers().get_all("set-cookie") {
            let Ok(s) = c.to_str() else { continue };
            let Some((kv, _)) = s.split_once(';').or(Some((s, ""))) else { continue };
            if let Some((k, v)) = kv.split_once('=') {
                if COOKIES.contains(&k.trim()) {
                    self.cookies.insert(k.trim().to_string(), v.trim().to_string());
                }
            }
        }
    }
}

/// `vrc-bridge login`: interactive, by the owner.
pub async fn login_main(cookie_file: &Path) -> Result<()> {
    let mut api = VrcApi::new(cookie_file.to_path_buf());
    api.login().await
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: &str = "wrld_4432ea9b-729c-46e3-8eaf-846aa0a37fdd";

    #[test]
    fn kinds_of_instances() {
        assert_eq!(instance_kind(&format!("{W}:12345")), Some("public"));
        assert_eq!(instance_kind(&format!("{W}:12345~hidden(usr_abc)~region(jp)")), Some("hidden"));
        assert_eq!(instance_kind(&format!("{W}:12345~friends(usr_abc)")), Some("friends"));
        assert_eq!(instance_kind(&format!("{W}:12345~private(usr_abc)~canRequestInvite")), Some("private"));
        assert_eq!(instance_kind(&format!("{W}:1~group(grp_x)~groupAccessType(public)")), Some("group-public"));
        assert_eq!(instance_kind(&format!("{W}:1~group(grp_x)~groupAccessType(members)")), Some("group"));
        assert_eq!(instance_kind("offline"), None);
        assert_eq!(instance_kind("traveling"), None);
        assert_eq!(instance_kind(&format!("{W}:1~friends(usr x)")), None);
        assert!(joinable(&format!("{W}:12345~friends(usr_abc)")));
        assert!(!joinable(&format!("{W}:12345")));
    }

    #[test]
    fn launch_urls() {
        let loc = format!("{W}:12345~friends(usr_abc)");
        assert_eq!(launch_location(&launch_url(&loc).unwrap()).unwrap(), loc);
        assert!(launch_location(&format!("vrchat://launch?id={W}:12345")).is_err()); // public
        assert!(launch_location(&format!("vrchat://launch?id={loc}&evil=1")).is_err());
        assert!(launch_location(&format!("https://launch?id={loc}")).is_err());
        assert!(launch_url(&format!("{W}:12345")).is_err());
    }

    #[test]
    fn redacts_tokens() {
        assert_eq!(redact("wss://x/?authToken=abc123&x=1"), "wss://x/?authToken=***&x=1");
        assert_eq!(redact("cookie auth=secret; other"), "cookie auth=***; other");
    }
}
