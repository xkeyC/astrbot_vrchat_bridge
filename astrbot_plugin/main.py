"""VRChat for AstrBot: the bot's own VRChat client as a platform.

The game runs on a Linux host (VR mode on a virtual headset) next to the Rust
bridge (``crates/vrc-bridge``); this plugin
registers the ``vrchat`` platform adapter (``vrchat_adapter.py``: realtime
voice in the room, quick actions for the voice model, text to the chatbox)
and the tools the room's chat uses to move the avatar, follow friends and go
places.

Needs the AstrBot Codex fork (``astrbot.core.voice``).
"""

from __future__ import annotations

import base64
import json

import mcp.types

from astrbot.api import llm_tool
from astrbot.api.event import AstrMessageEvent, filter
from astrbot.api.star import Context, Star

from .vrchat_adapter import MOTIONS, find_adapter, goto_words, motion_body, posture_body, survey_words


def _json(data) -> str:
    return json.dumps(data, ensure_ascii=False)


class VRChatPlugin(Star):
    """VRChat 平台：bot 自己的 VRChat 客户端，房间里实时语音对话，能移动、做表情、写 Chatbox、跟随白名单好友。"""

    def __init__(self, context: Context, config: dict | None = None) -> None:
        super().__init__(context)
        self.config = config or {}

    async def _act(self, event: AstrMessageEvent, action) -> str:
        """Runs ``action(adapter)`` for a tool of the VRChat chat; returns the
        tool result. Voice turns come as synthetic events of another platform
        name, so the chat is told by its platform id."""
        adapter = find_adapter(self.context)
        if adapter is None:
            return _json({"error": "VRChat 平台未运行"})
        if event.get_platform_id() != adapter.meta().id:
            return _json({"error": "只能在 VRChat 会话中使用"})
        try:
            return _json(await action(adapter))
        except Exception as exc:  # noqa: BLE001 - reported to the model
            return _json({"error": str(exc)})

    async def _picture(self, event: AstrMessageEvent, action) -> mcp.types.CallToolResult | str:
        """Like ``_act`` for a tool that shows the model a picture:
        ``action(adapter)`` returns (text, jpeg) or a text."""
        adapter = find_adapter(self.context)
        if adapter is None:
            return _json({"error": "VRChat 平台未运行"})
        if event.get_platform_id() != adapter.meta().id:
            return _json({"error": "只能在 VRChat 会话中使用"})
        try:
            result = await action(adapter)
        except Exception as exc:  # noqa: BLE001 - reported to the model
            return _json({"error": str(exc)})
        if isinstance(result, str):
            return result
        text, jpeg = result
        return mcp.types.CallToolResult(content=[
            mcp.types.TextContent(type="text", text=text),
            mcp.types.ImageContent(type="image", data=base64.b64encode(jpeg).decode(),
                                   mimeType="image/jpeg"),
        ])

    @llm_tool("vrchat_who")
    async def vrchat_who(self, event: AstrMessageEvent) -> str:
        """列出 VRChat 房间里现在的其他玩家（显示名）。
        """
        adapter = find_adapter(self.context)
        if adapter is not None and event.unified_msg_origin != adapter.room_umo:
            return _json({"error": "只能在 VRChat 语音线程配套的文字会话中使用"})

        async def who(adapter):
            return {"players": adapter.players()}

        return await self._act(event, who)

    @llm_tool("vrchat_last_seen")
    async def vrchat_last_seen(self, event: AstrMessageEvent, name: str = ""):
        """最后一次看到某位白名单好友时的画面，附多久以前、在哪个世界。

        Args:
            name(string): 好友显示名；留空则取最近看到的那位。
        """
        adapter = find_adapter(self.context)
        if adapter is not None and event.unified_msg_origin != adapter.room_umo:
            return _json({"error": "只能在 VRChat 语音线程配套的文字会话中使用"})

        async def last_seen(adapter):
            found = await adapter.last_seen(name)
            if found is None:
                return "还没有看到过白名单好友。"
            sighting, jpeg = found
            return (f"最后一次看到 {sighting['name']}：{sighting['age_s']:.0f} 秒前，"
                    f"在 {sighting['world'] or '未知世界'}"), jpeg

        return await self._picture(event, last_seen)

    # -- looking around and walking ----------------------------------------

    async def _vr(self, event: AstrMessageEvent, action) -> mcp.types.CallToolResult | str:
        """Runs a VR action for the room's own chat: ``action(adapter)``
        returns (text, view or panorama JPEG, map PNG or None)."""
        adapter = find_adapter(self.context)
        if adapter is None:
            return _json({"error": "VRChat 平台未运行"})
        if event.unified_msg_origin != adapter.room_umo:
            return _json({"error": "只能在 VRChat 语音线程配套的文字会话中使用"})
        try:
            text, pano, top = await action(adapter)
        except Exception as exc:  # noqa: BLE001 - reported to the model
            return _json({"error": str(exc)})
        content = [
            mcp.types.TextContent(type="text", text=text),
            mcp.types.ImageContent(type="image", data=base64.b64encode(pano).decode(), mimeType="image/jpeg"),
        ]
        if top is not None:
            content.append(mcp.types.ImageContent(type="image", data=base64.b64encode(top).decode(), mimeType="image/png"))
        return mcp.types.CallToolResult(content=content)

    @llm_tool("vrchat_view")
    async def vrchat_view(self, event: AstrMessageEvent, around: bool = False, players: bool = True):
        """看正前方的画面，带编号地点（玩家按名牌认出，白名单好友标出；可走的地面、已见区域的边缘、可跳上去的高台），各带距离和方位（相对正前方，正数在右）。用 vrchat_walk_to 走到某个编号。around 为 true 时原地转一圈（要几秒，房间里的人都看得到你在四处张望），返回全景图（中间是正前方，两边是身后）和俯视地图（你在中间、朝上；绿色地面、红色障碍、暗色未知）。尽量少用 around：只在要找的东西不在前方、又不知道在哪个方向时才用；否则先转向（vrchat_step）再看前方。

        Args:
            around(boolean): 是否环视一圈（慢，少用；默认 false，只看前方）。
            players(boolean): 是否读名牌找玩家（默认 true；false 稍快）。
        """

        async def look(adapter):
            data, pano, top = await adapter.vr_survey(bool(players), who="text", around=bool(around))
            return survey_words(data), pano, top

        return await self._vr(event, look)

    @llm_tool("vrchat_walk_to")
    async def vrchat_walk_to(self, event: AstrMessageEvent, place: int = -1, bearing: float = 0.0,
                             distance: float = 0.0, pace: str = "walk", around: bool = False):
        """走到上一次 vrchat_view 的某个编号地点（绕开障碍、分段走），或按方位走一段距离；走完返回正前方的新画面和编号地点（around 为 true 时返回环视全景图和地图，少用）。

        Args:
            place(number): 上一次环视里的地点编号；不按编号走时留空（-1）。
            bearing(number): 不按编号时：相对正前方的角度（正数向右，180 为身后）。
            distance(number): 配合 bearing：走多少米（默认 2）。
            pace(string): walk（走，默认）或 run（跑）。
            around(boolean): 到了以后是否环视一圈（慢，少用；默认 false，只看前方）。
        """

        async def walk(adapter):
            if place >= 1:
                body = {"candidate": int(place)}
            elif distance > 0 or bearing:
                body = {"bearing": float(bearing), "distance": float(distance or 2.0)}
            else:
                raise RuntimeError("给一个地点编号，或者方位和距离")
            if pace in ("walk", "run"):
                body["pace"] = pace
            if around:
                body["around"] = True
            data, pano, top = await adapter.vr_goto(body, who="text")
            return goto_words(data) + " " + survey_words(data["after"]), pano, top

        return await self._vr(event, walk)

    # -- admin commands, from any platform ---------------------------------

    @filter.command_group("vrc")
    def vrc(self):
        """VRChat 客户端管理（管理员）。"""

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("status")
    async def vrc_status(self, event: AstrMessageEvent):
        """VRChat 状态：桥接、客户端、游戏内账号、所在房间、Web API 登录、跟随。"""
        adapter = find_adapter(self.context)
        if adapter is None:
            yield event.plain_result("VRChat 平台未启用。")
            return
        lines = [f"桥接：{'已连接' if adapter.bridge_connected else '未连接'}（{adapter.base_url}）"]
        try:
            status = await adapter.request("GET", "/v1/status")
            social = await adapter.request("GET", "/v1/social")
        except Exception as exc:  # noqa: BLE001 - shown to the admin
            lines.append(f"无法读取状态：{exc}")
            yield event.plain_result("\n".join(lines))
            return
        me = status.get("self") or {}
        lines.append("客户端：" + (f"运行中，显存 {status.get('vram_mib')} MiB" if status.get("running")
                                  else "未运行"))
        if status.get("running"):
            lines.append(f"游戏内账号：{me.get('name') or '未登录/加载中'}")
            world = status.get("world") or "（不在世界中）"
            lines.append(f"所在：{world} {status.get('instance') or ''}".rstrip())
            players = [p["name"] for p in status.get("players", [])]
            lines.append(f"房间里：{', '.join(players) if players else '无其他玩家'}")
        account = social.get("me") or {}
        lines.append("Web API：" + (f"已登录 {account.get('name')}" if social.get("logged_in")
                                   else "未登录（需在服务器上运行 vrc_bridge.py login）"))
        follow = status.get("follow") or {}
        if follow.get("state", "idle") != "idle":
            lines.append(f"跟随：{follow.get('target')}（{follow.get('state')}）")
        yield event.plain_result("\n".join(lines))

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("start")
    async def vrc_start(self, event: AstrMessageEvent, url: str = ""):
        """启动 VRChat 客户端；可带 vrchat://launch 链接直接进房间（公开房间拒绝）。"""
        yield event.plain_result(await self._admin(lambda a: a.request(
            "POST", "/v1/game/start", {"url": url} if url else {}), "已发出启动指令，约半分钟进入游戏。"))

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("stop")
    async def vrc_stop(self, event: AstrMessageEvent):
        """关闭 VRChat 客户端。"""
        yield event.plain_result(await self._admin(
            lambda a: a.request("POST", "/v1/game/stop"), "已关闭 VRChat 客户端。"))

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("restart")
    async def vrc_restart(self, event: AstrMessageEvent):
        """重启 VRChat 客户端（回到默认的家）。"""

        async def restart(adapter):
            await adapter.request("POST", "/v1/game/stop")
            return await adapter.request("POST", "/v1/game/start", {})

        yield event.plain_result(await self._admin(restart, "已重启 VRChat 客户端，约半分钟进入游戏。"))

    async def _admin(self, action, done: str) -> str:
        adapter = find_adapter(self.context)
        if adapter is None:
            return "VRChat 平台未启用。"
        try:
            await action(adapter)
        except Exception as exc:  # noqa: BLE001 - shown to the admin
            return f"失败：{exc}"
        return done

    @llm_tool("vrchat_status")
    async def vrchat_status(self, event: AstrMessageEvent) -> str:
        """VRChat 当前状态：游戏是否在运行、所在世界和房间实例、房间里的其他玩家（显示名）。"""
        return await self._act(event, lambda a: a.request("GET", "/v1/status"))

    @llm_tool("vrchat_step")
    async def vrchat_step(self, event: AstrMessageEvent, turn: float = 0.0, direction: str = "forward",
                          meters: float = 0.0, jump: bool = False, pace: str = "walk") -> str:
        """小而精确的动作（不是赶路，赶路用 vrchat_walk_to）：先按角度转身（正为右），再朝某个方向走几米（按角色自身速度计量，被挡住就停），或者跳。

        Args:
            turn(number): 先转多少度，正为右、负为左（180 为掉头）。
            direction(string): 往哪走（相对转身后的朝向）：forward、back、left、right。
            meters(number): 走多少米，0 到 5（0 不走）。
            jump(boolean): 起步时跳（不走时原地跳）。
            pace(string): walk（走，默认）或 run（跑）。
        """
        return await self._act(event, lambda a: a.step(turn, direction, meters, jump, pace))

    @llm_tool("vrchat_height")
    async def vrchat_height(self, event: AstrMessageEvent, metres: float | None = None,
                            change_cm: float | None = None) -> str:
        """设置 VR 头显离地的高度，也就是角色的站姿：踮脚说明太高，屈膝说明太低（几厘米就有差别）。不给参数时返回当前高度。

        Args:
            metres(number): 高度，米（1.2 到 1.9；现在 1.56 左右合适）。
            change_cm(number): 或者相对现在的变化，厘米（正为升高）。
        """
        if metres is not None:
            body = {"metres": metres}
        elif change_cm is not None:
            body = {"change_cm": change_cm}
        else:
            return await self._act(event, lambda a: a.request("GET", "/v1/vr/height"))
        return await self._act(event, lambda a: a.request("POST", "/v1/vr/height", body))

    @llm_tool("vrchat_vr_reset")
    async def vrchat_vr_reset(self, event: AstrMessageEvent) -> str:
        """像 SteamVR 的重置那样重置虚拟头显：停止行走和跟随，重新连接头显，平视前方、双手放回身侧，并重新居中。画面或身体看起来卡住、不对劲时用。"""
        return await self._act(event, lambda a: a.request("POST", "/v1/vr/reset"))

    @llm_tool("vrchat_jump")
    async def vrchat_jump(self, event: AstrMessageEvent) -> str:
        """在 VRChat 里跳一下。"""
        return await self._act(event, lambda a: a.request("POST", "/v1/jump"))

    @llm_tool("vrchat_motion")
    async def vrchat_motion(self, event: AstrMessageEvent, name: str, other_hand: bool = False, times: int = 1,
                            seconds: float = 0.0) -> str:
        """在 VRChat 里用全身做一个动作（挥手、再见、点头、摇头、摆手拒绝、思考、环顾、转身、后空翻、跳舞等）。

        Args:
            name(string): wave、bye、byebye、nod、shake_head、refuse、think、look_around、turn、backflip、dance、dance_short、chicken_dance、idle_talk、fold_arms 之一。
            other_hand(boolean): 换另一只手（镜像）。
            times(number): 做几次（1-5）。
            seconds(number): 跳舞、待机类动作持续多久（秒）。
        """
        try:
            body = motion_body({"name": name, "other_hand": other_hand, "times": times, "seconds": seconds})
        except ValueError as e:
            return _json({"error": str(e), "names": list(MOTIONS)})
        return await self._act(event, lambda a: a.request("POST", "/v1/motion", body))

    @llm_tool("vrchat_posture")
    async def vrchat_posture(self, event: AstrMessageEvent, posture: str, way: str = "back") -> str:
        """在 VRChat 里坐下、躺下或站起来；躺着时可以翻身换方向。保持该姿势直到被要求改变或开始移动。

        Args:
            posture(string): stand（站起来）、sit（坐下，翘二郎腿）、lie（躺下）之一。
            way(string): 躺下时的方向：back（仰躺）、left（左侧躺）、right（右侧躺）、front（趴着）。
        """
        try:
            body = posture_body({"posture": posture, "way": way})
        except ValueError as e:
            return _json({"error": str(e)})
        return await self._act(event, lambda a: a.request("POST", "/v1/motion", body))

    @llm_tool("vrchat_stop")
    async def vrchat_stop(self, event: AstrMessageEvent) -> str:
        """立即停止 VRChat 里正在进行的移动、转身和跟随。"""
        return await self._act(event, lambda a: a.request("POST", "/v1/stop"))

    @llm_tool("vrchat_chatbox")
    async def vrchat_chatbox(self, event: AstrMessageEvent, text: str) -> str:
        """在 VRChat 头顶的 Chatbox 里显示文字（每条最多 144 字，长文自动分条）。

        Args:
            text(string): 要显示的文字，纯文本。
        """
        return await self._act(event, lambda a: a.request("POST", "/v1/chatbox", {"text": text}))

    @llm_tool("vrchat_social")
    async def vrchat_social(self, event: AstrMessageEvent) -> str:
        """VRChat 白名单好友的情况：各自在线与否、所在房间能否进入（公开房间不进）、是否开启跨房间跟随及正在跟随谁、Web API 是否已登录。"""

        async def social(adapter):
            status = await adapter.request("GET", "/v1/social")
            status["whitelist"] = [
                {key: entry.get(key) for key in ("name", "friend", "kind", "joinable")}
                for entry in status.get("whitelist", [])
            ]
            status.pop("me", None)
            return status

        return await self._act(event, social)

    @llm_tool("vrchat_follow_player")
    async def vrchat_follow_player(
        self, event: AstrMessageEvent, name: str = "", stop: bool = False, distance: float = 0.0
    ) -> str:
        """在当前 VRChat 房间里跟随某位玩家：始终面向对方并保持一定距离（看头顶名牌找人），直到叫停或对方离开房间。

        Args:
            name(string): 对方显示名；留空则跟白名单里优先级最高、且在本房间的好友。
            stop(boolean): true 表示停止跟随。
            distance(number): 与对方保持的距离，米（0.8 到 7）；不填或 0 用默认（约 1.5 米）。
        """
        return await self._act(event, lambda a: a.follow(name, stop, distance or None))

    @llm_tool("vrchat_follow_adjust")
    async def vrchat_follow_adjust(self, event: AstrMessageEvent, change: str) -> str:
        """调整正在进行的 VRChat 跟随：靠近一点、离远一点、原地别动（仍面向对方）、继续跟随。没在跟随时，stay 只是停下不动。

        Args:
            change(string): closer（靠近一点）、farther（离远一点）、stay（原地别动）、resume（继续跟）之一。
        """
        return await self._act(event, lambda a: a.follow_adjust(change))

    @llm_tool("vrchat_follow_rooms")
    async def vrchat_follow_rooms(self, event: AstrMessageEvent, enabled: bool) -> str:
        """开启或关闭跨房间跟随：开启后 bot 会去白名单里优先级最高、且在可进入房间（非公开）的好友那里，对方换房间时跟过去（每次换房间要重启游戏，约半分钟）。

        Args:
            enabled(boolean): true 开启，false 关闭。
        """

        async def follow(adapter):
            adapter.social_config["follow"] = bool(enabled)
            return await adapter.request("POST", "/v1/social/config", {"follow": bool(enabled)})

        return await self._act(event, follow)

    @llm_tool("vrchat_join")
    async def vrchat_join(self, event: AstrMessageEvent, url: str) -> str:
        """重启 VRChat 进入某位白名单好友此刻所在的房间（其他房间一律拒绝）。约需半分钟，期间语音中断。

        Args:
            url(string): 该好友房间的 vrchat://launch?ref=vrchat.com&id=wrld_...:实例~... 链接。
        """

        async def join(adapter):
            # Anyone in the room may ask: only to where a whitelisted friend is
            # now (the bridge checks it, and the link, before it stops the
            # game). Elsewhere: an admin's /vrc start.
            return await adapter.request(
                "POST",
                "/v1/game/start",
                {"url": url, "restart": True, "whitelisted_only": True},
            )

        return await self._act(event, join)
