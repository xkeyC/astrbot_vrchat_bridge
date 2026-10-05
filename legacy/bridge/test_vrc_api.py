"""Unit tests of the instance rules (run: python -m pytest bridge)."""
import pytest

from vrc_api import joinable, launch_location, launch_url, redact

W = "wrld_4cf554b4-430c-4f8f-b53e-1f294eed230b"


@pytest.mark.parametrize(("location", "ok"), [
    (f"{W}:12345~friends(usr_a)~region(jp)", True),
    (f"{W}:12345~hidden(usr_a)~region(jp)", True),
    (f"{W}:12345~private(usr_a)~canRequestInvite~region(jp)", True),
    (f"{W}:12345~region(jp)", False),  # public
    (f"{W}:12345", False),
    (f"{W}:12345~group(grp_a)~groupAccessType(members)", False),
    (f"{W}:12345~group(grp_a)~groupAccessType(plus)", False),
    (f"{W}:12345~group(grp_a)~groupAccessType(public)", False),
    ("offline", False),
    (f"{W}:1~friends(usr_a)&id={W}:2", False),
    (f"{W}:1~friends(usr_a) extra", False),
    (f"{W}:ab_1-2~friends(usr_a-b_c)~nonce(x-1.2)", True),
    (f"{W}:1~friends(usr_a#x)", False),
])
def test_only_friends_friends_plus_and_invite_instances_are_joinable(location, ok):
    assert joinable(location) is ok


def test_launch_urls_take_one_joinable_id_and_nothing_else():
    here = f"{W}:12345~friends(usr_a)~region(jp)"
    assert launch_location(f"vrchat://launch?ref=vrchat.com&id={here}") == here
    assert launch_location(launch_url(here)) == here
    for bad in (
        f"vrchat://launch?id={here}&id={W}:2",
        f"vrchat://launch?id={here}&extra=1",
        f"vrchat://launch?id={W}:2",  # public
        f"https://launch?id={here}",
        f"vrchat://launch/x?id={here}",
    ):
        with pytest.raises(ValueError):
            launch_location(bad)


def test_tokens_are_redacted():
    text = "403, url='wss://pipeline.vrchat.cloud/?authToken=authcookie_abc123&x=1'"
    assert "authcookie_abc123" not in redact(text)
    assert "authToken=***" in redact(text)
