"""VRChat Web API for the bridge, the way VRCX uses it: the bot account's
session cookie, a few REST calls and the pipeline WebSocket (notifications,
friends' locations).

Login is interactive and done by the owner (``vrc_bridge.py login``): only
the resulting cookies are kept, never the password. When they expire the
bridge reports ``auth_required`` and the owner logs in again.
"""

from __future__ import annotations

import asyncio
import getpass
import json
import logging
import re
from collections.abc import Awaitable, Callable
from pathlib import Path
import urllib.parse
from urllib.parse import quote

import aiohttp

log = logging.getLogger("vrc-bridge.api")

API = "https://api.vrchat.cloud/api/1"
PIPELINE = "wss://pipeline.vrchat.cloud/"
# VRChat asks API clients to identify themselves with a contact.
USER_AGENT = "astrbot-vrchat-bridge/0.1 (github.com/xkeyC/astrbot_vrchat_bridge)"
COOKIES = ("auth", "twoFactorAuth")
# A world, an instance name, then tags such as ~friends(usr_...),
# ~region(jp) or ~canRequestInvite: nothing else.
RE_INSTANCE = re.compile(
    r"^(wrld_[0-9a-f-]+):([A-Za-z0-9_-]+)((?:~[A-Za-z]+(?:\([A-Za-z0-9_.-]*\))?)*)$"
)
# What the bot may join: friends+ (hidden), friends, invite / invite+ (private).
JOINABLE_KINDS = ("hidden", "friends", "private")
RE_AUTH_TOKEN = re.compile(r"(authToken=|auth=)[^&\s'\";]+")


def redact(text: object) -> str:
    """``text`` without session tokens (error messages may carry URLs)."""
    return RE_AUTH_TOKEN.sub(r"\1***", str(text))


class AuthRequired(RuntimeError):
    """The session cookie is missing or expired: the owner must log in."""


def instance_kind(location: str) -> str | None:
    """The access type of an instance location, or None when it is no
    instance (``offline``, ``private`` - hidden from us - or ``traveling``).

    Returns:
        ``public``, ``hidden`` (friends+), ``friends``, ``private`` (invite,
        invite+), ``group-public`` or ``group`` (members / plus).
    """
    m = RE_INSTANCE.match(location or "")
    if not m:
        return None
    tags = m.group(3) or ""
    if "~group(" in tags:
        return "group-public" if "groupAccessType(public)" in tags else "group"
    for kind in ("hidden", "friends", "private"):
        if f"~{kind}(" in tags:
            return kind
    return "public"


def joinable(location: str) -> bool:
    """Whether the bot may go there: friends+, friends or invite instances
    only (never public, group or anything else)."""
    return instance_kind(location) in JOINABLE_KINDS


def launch_url(location: str) -> str:
    """The ``vrchat://launch`` URL of a joinable instance.

    Raises:
        ValueError: The location is not a joinable instance.
    """
    if not joinable(location):
        raise ValueError(f"not a joinable instance: {location!r}")
    return f"vrchat://launch?ref=vrchat.com&id={location}"


def launch_location(url: str) -> str:
    """The instance of a ``vrchat://launch`` URL: ``id`` and at most ``ref``,
    nothing else, the instance joinable.

    Raises:
        ValueError: Anything else.
    """
    parts = urllib.parse.urlsplit(url.strip())
    query = urllib.parse.parse_qs(parts.query, keep_blank_values=True, strict_parsing=False)
    if (
        parts.scheme != "vrchat"
        or parts.netloc != "launch"
        or parts.path not in ("", "/")
        or parts.fragment
        or set(query) - {"id", "ref"}
        or len(query.get("id", [])) != 1
        or len(query.get("ref", [])) > 1
    ):
        raise ValueError("only vrchat://launch?id=<instance> URLs")
    location = query["id"][0]
    if not joinable(location):
        raise ValueError("only friends+, friends or invite instances")
    return location


class VRChatApi:
    def __init__(self, cookie_file: Path) -> None:
        self.cookie_file = cookie_file
        self.cookies: dict[str, str] = {}
        self._http: aiohttp.ClientSession | None = None

    def load(self) -> bool:
        try:
            self.cookies = json.loads(self.cookie_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            self.cookies = {}
        return bool(self.cookies.get("auth"))

    def _save(self) -> None:
        self.cookie_file.parent.mkdir(parents=True, exist_ok=True)
        self.cookie_file.touch(mode=0o600)
        self.cookie_file.chmod(0o600)  # touch leaves an existing file's mode
        self.cookie_file.write_text(json.dumps(self.cookies), encoding="utf-8")

    def http(self) -> aiohttp.ClientSession:
        if self._http is None or self._http.closed:
            self._http = aiohttp.ClientSession(headers={"User-Agent": USER_AGENT})
        return self._http

    async def close(self) -> None:
        if self._http is not None:
            await self._http.close()

    async def call(self, method: str, path: str, **kwargs):
        """One REST call with the session cookies.

        Raises:
            AuthRequired: The session is not valid (HTTP 401).
            RuntimeError: Any other failure.
        """
        if not self.cookies.get("auth"):
            raise AuthRequired("not logged in")
        cookie = "; ".join(f"{k}={v}" for k, v in self.cookies.items() if k in COOKIES)
        async with self.http().request(
            method, API + path, headers={"Cookie": cookie},
            timeout=aiohttp.ClientTimeout(total=20), **kwargs,
        ) as resp:
            if resp.status == 401:
                raise AuthRequired("session expired")
            if resp.status == 429:
                raise RuntimeError("rate limited by VRChat")
            data = await resp.json(content_type=None)
            if resp.status >= 300:
                raise RuntimeError(f"HTTP {resp.status}: {data}")
            return data

    async def me(self) -> dict:
        data = await self.call("GET", "/auth/user")
        if "requiresTwoFactorAuth" in data:
            raise AuthRequired("two-factor verification pending")
        return data

    async def friends(self) -> list[dict]:
        """Online friends (with their location) first, then offline ones."""
        result = []
        for offline in ("false", "true"):
            offset = 0
            while True:
                page = await self.call(
                    "GET", f"/auth/user/friends?offline={offline}&n=100&offset={offset}")
                result += page
                if len(page) < 100:
                    break
                offset += 100
        return result

    async def user(self, user_id: str) -> dict:
        return await self.call("GET", f"/users/{quote(user_id)}")

    async def pipeline(self, on_event: Callable[[str, dict], Awaitable[None]]) -> None:
        """Follows the pipeline until it closes; ``on_event(type, content)``.

        Raises:
            AuthRequired: Not logged in.
        """
        if not self.cookies.get("auth"):
            raise AuthRequired("not logged in")
        url = f"{PIPELINE}?authToken={quote(self.cookies['auth'])}"
        async with self.http().ws_connect(url, heartbeat=30) as ws:
            log.info("pipeline connected")
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                    content = data.get("content")
                    if isinstance(content, str):
                        content = json.loads(content) if content.startswith("{") else {"text": content}
                    await on_event(str(data.get("type")), content or {})
                except Exception:  # noqa: BLE001 - one bad message
                    log.exception("pipeline message failed")

    # -- login (interactive, run by the owner) ----------------------------------

    async def login(self) -> None:
        """Asks for the bot account, its password and the 2FA code; keeps the cookies."""
        username = input("VRChat username or email: ").strip()
        password = getpass.getpass("Password (not stored): ")
        auth = aiohttp.BasicAuth(quote(username, safe=""), quote(password, safe=""))
        async with self.http().get(f"{API}/auth/user", auth=auth) as resp:
            data = await resp.json(content_type=None)
            if resp.status != 200:
                raise SystemExit(f"login failed: HTTP {resp.status} {data}")
            self._take(resp)
        methods = data.get("requiresTwoFactorAuth") or []
        if methods:
            kind = "totp" if "totp" in methods else "emailotp" if "emailOtp" in methods else "otp"
            code = input(f"Two-factor code ({', '.join(methods)}): ").strip()
            cookie = f"auth={self.cookies['auth']}"
            async with self.http().post(
                f"{API}/auth/twofactorauth/{kind}/verify", json={"code": code},
                headers={"Cookie": cookie},
            ) as resp:
                result = await resp.json(content_type=None)
                if resp.status != 200 or not result.get("verified"):
                    raise SystemExit(f"two-factor verification failed: {result}")
                self._take(resp)
        me = await self.me()
        self._save()
        print(f"Logged in as {me.get('displayName')} ({me.get('id')}); cookies saved to {self.cookie_file}")

    def _take(self, resp: aiohttp.ClientResponse) -> None:
        for name in COOKIES:
            if name in resp.cookies:
                self.cookies[name] = resp.cookies[name].value


async def login_main(cookie_file: Path) -> None:
    api = VRChatApi(cookie_file)
    try:
        await api.login()
    finally:
        await api.close()


if __name__ == "__main__":
    asyncio.run(login_main(Path("~/.config/vrc-bridge/cookies.json").expanduser()))
